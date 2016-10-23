#include <avr/io.h>
#include <avr/cpufunc.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

/*         0 1 2 3 4 5 6 7 8 9
          ---------------------
  Q7 : g   1 1 0 0 0 0 0 1 0 0
  Q6 : c   0 0 1 0 0 0 0 0 0 0
  Q5 : DP  1 1 1 1 1 1 1 1 1 1
  Q4 : e   0 1 0 1 1 1 0 1 0 1
  Q3 : d   0 1 0 0 1 0 0 1 0 0
  Q2 : a   0 1 0 0 1 0 0 0 0 0
  Q1 : f   0 1 1 1 0 0 0 1 0 0
  Q0 : b   0 0 0 0 0 1 1 0 0 0
*/

volatile const uint8_t digit[] = { 0xa0, 0xbe, 0x62, 0x32, 0x3c, 0x31, 0x21, 0xba, 0x20, 0x30};
volatile uint8_t digits[4] = {0xff, 0xff, 0xff, 0xff};
volatile uint8_t dot = 0xdf;

volatile const uint16_t buzz_sampl[] = {
	0xC20, 0xEF0, 0xC20, 0x380, 0x010, 0x3A0, 0xC30, 0xFF0, 0xBC0, 0x430,
	0x110, 0x5E0, 0xC00, 0xD80, 0x860, 0x3A0, 0x440, 0x950, 0xC00, 0x9F0,
	0x640, 0x4D0, 0x6A0, 0x960, 0xAD0, 0x8D0, 0x680, 0x5C0, 0x730, 0xAD0,
	0x9F0, 0x620, 0x400, 0x960, 0xB60, 0x8C0, 0x5F0, 0x8F0, 0xA80, 0x640,
	0x2B0, 0x4A0};

volatile uint8_t buzz_cnt;

volatile uint16_t counts[10] = {0};
volatile uint8_t  count_num;

volatile uint8_t tcc4_intflags = 0;

void main_clocksource_select(uint8_t clkCtrl);
void set_led_digit(uint16_t v);

void play_wav(void);

void setup_XOSC_EXT(void)
{
	/* Select XTAL1 for external oscillator. */
	OSC.XOSCCTRL = OSC_FRQRANGE_9TO12_gc | OSC_XOSCSEL_EXTCLK_gc;

	/* Enable External Clock Sounrce */
	OSC.CTRL |= OSC_XOSCEN_bm;

	/* wait until stable. */
	do {} while ( !(OSC.STATUS & OSC_XOSCRDY_bm) );
	main_clocksource_select(CLK_SCLKSEL_XOSC_gc);
}

/*-----------------------------------------------------------------------*/
/* Xorshift pseudo random generator                                      */
/*-----------------------------------------------------------------------*/
volatile uint32_t y = 2463534242;

uint32_t xorshift(void)
{
  y ^= (y<<13); y ^= (y >> 17); y ^= (y << 5);
  return y;
}

inline
void init_usart_spi_led(void)
{
	USARTD0.CTRLC = USART_CMODE_MSPI_gc;	/* SPI Mode 0, MSB first */
	USARTD0.CTRLB = USART_TXEN_bm | USART_RXEN_bm;
	USARTD0.BAUDCTRLA = 0;

	/* Enable TXD0 for USART-SPI Master, Send to Segment Register */
	do {} while ( !(USARTD0.STATUS & USART_DREIF_bm) );
	USARTD0.DATA = 0xff;
	do {} while ( !(USARTD0.STATUS & USART_RXCIF_bm) );

	USARTD0.CTRLB = 0;
	PORTD.DIRSET = PIN5_bm | PIN6_bm | PIN7_bm;

	PORTD.OUTCLR = PIN7_bm;	/* MOSI = L to 74AC164 */

	PORTD.OUTSET = PIN6_bm; /* MISO = H, load to 74AC164 */
	PORTD.OUTTGL = PIN6_bm;	/* MISO = L */
	PORTD.OUTTGL = PIN6_bm; /* MISO = H, load to 74AC164 */
	PORTD.OUTTGL = PIN6_bm;	/* MISO = L */
	PORTD.OUTTGL = PIN6_bm; /* MISO = H, load to 74AC164 */
	PORTD.OUTTGL = PIN6_bm;	/* MISO = L */
	PORTD.OUTTGL = PIN6_bm; /* MISO = H, load to 74AC164 */
	PORTD.OUTTGL = PIN6_bm;	/* MISO = L, ratch & enable 74HC595 */
}

inline
void output_usart_spi_led(uint8_t data, uint8_t flag)
{
	USARTD0.CTRLB = USART_TXEN_bm | USART_RXEN_bm;

	/* Enable TXD0 for USART-SPI Master, Send to Segment Register */
	do {} while ( !(USARTD0.STATUS & USART_DREIF_bm) );
	USARTD0.DATA = data;
	do {} while ( !(USARTD0.STATUS & USART_RXCIF_bm) );

	USARTD0.CTRLB = 0;
	PORTD.DIRSET = PIN5_bm | PIN6_bm | PIN7_bm;

	if (flag) {
		PORTD.OUTSET = PIN7_bm;
	} else {
		PORTD.OUTCLR = PIN7_bm;
	}

	PORTD.OUTSET = PIN6_bm; /* MISO = H, load to 74AC164 */
	PORTD.OUTTGL = PIN6_bm;	/* MISO = L */
}

inline
void update_led(uint8_t intf)
{
	if (intf & TC4_CCDIF_bm) {
		output_usart_spi_led(digits[3], 0);
	}
	if (intf & TC4_CCCIF_bm) {
		output_usart_spi_led(digits[2] & dot, 0);
	}
	if (intf & TC4_CCBIF_bm) {
		output_usart_spi_led(digits[1], 0);
	}
	if (intf & TC4_CCAIF_bm) {
		output_usart_spi_led(digits[0], 1);
	}
	tcc4_intflags = 0;
}

inline
void setupTCC4_10ms(void)
{
	EVSYS.CH0MUX = EVSYS_CHMUX_TCC4_OVF_gc;

	/* Normal Operation */
	/* EV CH0 */
	/* TOP = 60000 */
	TCC5.CTRLB = TC45_BYTEM_NORMAL_gc | TC45_CIRCEN_DISABLE_gc | TC45_WGMODE_NORMAL_gc;
	TCC5.CNT = 0;
	TCC5.PER = 60000 - 1;
	TCC5.INTFLAGS = TC5_OVFIF_bm;
	TCC5.CTRLA = TC45_CLKSEL_EVCH0_gc;

	/* Normal Operation */
	/* 10MHz div8 / 12500 = 100Hz */
	/* Interrupt at 1562, 4687, 7812, 10937 (Level = Lo) */
	TCC4.CTRLB = TC45_BYTEM_NORMAL_gc | TC45_CIRCEN_DISABLE_gc | TC45_WGMODE_NORMAL_gc;
	TCC4.CNT = 0;
	TCC4.PER = 12500 - 1;		/* 10MHz div8 / 12500 = 100hz */
	TCC4.CCA =  1562 - 1;
	TCC4.CCB =  4687 - 1;
	TCC4.CCC =  7812 - 1;
	TCC4.CCD = 10937 - 1;
//	TCC4.INTCTRLA = TC45_OVFINTLVL_LO_gc;
	TCC4.INTCTRLB = TC45_CCDINTLVL_LO_gc | TC45_CCCINTLVL_LO_gc | TC45_CCBINTLVL_LO_gc | TC45_CCAINTLVL_LO_gc;
	TCC4.INTFLAGS = TC4_CCDIF_bm | TC4_CCCIF_bm | TC4_CCBIF_bm | TC4_CCAIF_bm | TC4_OVFIF_bm;
	TCC4.CTRLA = TC45_CLKSEL_DIV8_gc;
}

void timer_proc(void)
{
	uint8_t intf = tcc4_intflags;

	if (TCC5.INTFLAGS & TC5_OVFIF_bm) {
		TCC5.INTFLAGS = TC5_OVFIF_bm;
		TCC5.CNT += 10000;
	}

	if (intf) update_led(intf);
}

/*-----------------------------------------------------------------------*/
/* Buzzer                                                                */
/*-----------------------------------------------------------------------*/
ISR(EDMA_CH1_vect)
{
	if (--buzz_cnt) {
		EDMA.CH1.CTRLB    = EDMA_CH_TRNIF_bm | 0x02;
		EDMA.CH1.CTRLA    = EDMA_CH_ENABLE_bm | EDMA_CH_REPEAT_bm
							| EDMA_CH_SINGLE_bm | EDMA_CH_BURSTLEN_bm;
	} else {
		PORTA.OUTSET = PIN3_bm;

		EDMA.CH1.CTRLB    = EDMA_CH_TRNIF_bm;

		EDMA.CH1.CTRLA = 0;
		EDMA.CH1.CTRLA = 0;
		EDMA.CTRL = 0;

		TCD5.INTCTRLA = 0;
		TCD5.CTRLA = TC45_CLKSEL_OFF_gc;

		DACA.CTRLA = 0;
		DACA.CTRLB = 0;
		DACA.CTRLC = 0;
	}
}

static
void output_buzzer(void)
{
	/* (0.4(sec) / (42(sample) / 16000Hz)) / 2(repeat) = 76 */
	buzz_cnt = 76;

	DACA.EVCTRL  = DAC_EVSEL_1_gc;
	DACA.CTRLB   = DAC_CHSEL_SINGLE_gc | DAC_CH0TRIG_bm;
	DACA.CTRLC   = DAC_REFSEL_AVCC_gc;
	DACA.CTRLA   = DAC_CH0EN_bm | DAC_ENABLE_bm;

	PORTA.OUTCLR = PIN3_bm;

	EDMA.CTRL = EDMA_ENABLE_bm | EDMA_CHMODE_PER0123_gc
				| EDMA_DBUFMODE_DISABLE_gc | EDMA_PRIMODE_CH0123_gc;

	EDMA.CH1.ADDRCTRL = EDMA_CH_RELOAD_BLOCK_gc | EDMA_CH_DIR_INC_gc;
	EDMA.CH1.TRFCNT   = sizeof(buzz_sampl);
	EDMA.CH1.ADDR     = (uint16_t)(buzz_sampl);

	EDMA.CH1.TRIGSRC  = EDMA_CH_TRIGSRC_DACA_CH0_gc;

	EDMA.CH1.CTRLB    = EDMA_CH_TRNIF_bm | 0x02;
	EDMA.CH1.CTRLA    = EDMA_CH_ENABLE_bm | EDMA_CH_REPEAT_bm
						| EDMA_CH_SINGLE_bm | EDMA_CH_BURSTLEN_bm;

	TCD5.CTRLB = TC45_BYTEM_NORMAL_gc | TC45_CIRCEN_DISABLE_gc | TC45_WGMODE_NORMAL_gc;
	TCD5.CNT   = 0;
	TCD5.PER   = (10000000/16000) - 1;
	TCD5.CTRLA = TC45_CLKSEL_DIV1_gc;
	EVSYS.CH1MUX = EVSYS_CHMUX_TCD5_OVF_gc;
}

/*-----------------------------------------------------------------------*/
/* IDLE Mode                                                             */
/*-----------------------------------------------------------------------*/
static
uint8_t idle(void)
{
	uint8_t count_pos;
	uint8_t s1_status = 0xff;
	uint8_t s2_status = 0xff;

	TCC5.CNT = 0;
	TCC4.INTFLAGS = TC4_OVFIF_bm;

	dot = 0xdf;
	count_pos = (count_num) ? 1 : 0;
	set_led_digit(counts[count_pos]);

	set_sleep_mode(SLEEP_MODE_IDLE);
	do {
		sleep_mode();
		timer_proc();

		if (TCC4.INTFLAGS & TC4_OVFIF_bm) {
			TCC4.INTFLAGS = TC4_OVFIF_bm;

			xorshift();

			s1_status <<= 1;
			if ( !(PORTC.IN & PIN6_bm) ) ++s1_status;

			s2_status <<= 1;
			if ( !(PORTC.IN & PIN4_bm) ) ++s2_status;

			if (s2_status == 1) {
				if (count_num) {
					if (++count_pos > count_num) count_pos = 1;
					set_led_digit(counts[count_pos]);
				}
			}
		}
	} while (s1_status != 1);

	return 0;
}

/*-----------------------------------------------------------------------*/
/* DELAY Mode                                                            */
/*-----------------------------------------------------------------------*/
static
void delay(void)
{
	uint16_t count;
	uint16_t delay;

	dot = 0xff;
	digits[0] = digits[1] = digits[2] = digits[3] = 0x7f;

	play_wav();

	TCC5.CNT = 0;

	delay = 100;

	do {
		sleep_mode();
		timer_proc();

		xorshift();

		count = TCC5.CNT;
	} while (count < delay);
}

/*-----------------------------------------------------------------------*/
/* RUN Mode                                                              */
/*-----------------------------------------------------------------------*/
static
uint8_t run(void)
{
	uint8_t s1_status = 0xff;
	uint8_t s2_status = 0xff;
	uint8_t dotcnt = 50;

	/* stop & reset counter */
	TCC4.CTRLA = 0;
	TCC4.CNT = 0;
	TCC5.CNT = 0;

	/* clear 7seg shift-reg */
	PORTD.OUTCLR = PIN7_bm;	/* MOSI = L to 74AC164 */

	PORTD.OUTSET = PIN6_bm; /* MISO = H, load to 74AC164 */
	PORTD.OUTTGL = PIN6_bm;	/* MISO = L */
	PORTD.OUTTGL = PIN6_bm; /* MISO = H, load to 74AC164 */
	PORTD.OUTTGL = PIN6_bm;	/* MISO = L */
	PORTD.OUTTGL = PIN6_bm; /* MISO = H, load to 74AC164 */
	PORTD.OUTTGL = PIN6_bm;	/* MISO = L */
	PORTD.OUTTGL = PIN6_bm; /* MISO = H, load to 74AC164 */
	PORTD.OUTTGL = PIN6_bm;	/* MISO = L, ratch & enable 74HC595 */

	/* setup capture for AC */
	TCC5.CTRLD = TC45_EVSEL_CH4_gc;
	TCC5.CTRLE = TC45_CCAMODE_CAPT_gc;
	TCC5.INTFLAGS = TC5_CCAIF_bm;

	ACA.AC0MUXCTRL = AC_MUXPOS_PIN5_gc | AC_MUXNEG_BANDGAP_gc;
	ACA.AC0CTRL    = AC_INTMODE_RISING_gc | AC_HYSMODE_LARGE_gc | AC_ENABLE_bm;
	EVSYS.CH4MUX   = EVSYS_CHMUX_ACA_CH0_gc;

	dot = 0xdf;
	count_num = 0;

	/* start buzzer & restart counter! */
	output_buzzer();
	TCC4.CTRLA = TC45_CLKSEL_DIV8_gc;

	do {
		uint8_t ac_blank;
		uint16_t count;

		sleep_mode();
		timer_proc();

		if (ac_blank) {
			if ( !(--ac_blank) ) {
				EVSYS.CH4MUX  = EVSYS_CHMUX_ACA_CH0_gc;;
			}
		}

		/* check capture flag */
		if (TCC5.INTFLAGS & TC5_CCAIF_bm) {
			uint8_t tmp;

			TCC5.INTFLAGS = TC5_CCAIF_bm;

			count = TCC5.CCA;
			tmp = count_num + 1;
			counts[tmp] = count;
			count_num = tmp;
			if (tmp == 1) {
				set_led_digit(count);
			}

			/* make blank gap */
			EVSYS.CH4MUX  = 0;

			/* if capture 9 hits. */
			if (tmp == 9) break;

			/* blank gap timer (10ms * 10) */
			ac_blank = 10;
		}

		/* check 10ms interval */
		if (TCC4.INTFLAGS & TC4_OVFIF_bm) {
			TCC4.INTFLAGS = TC4_OVFIF_bm;

			if (count_num == 0) {
				count = TCC5.CNT;
				set_led_digit(count);
			}

			xorshift();

			/* blink dot */
			if ( !--dotcnt ) {
				dotcnt = 50;
				dot ^= 0x20;
			}

			/* check button status */
			s1_status <<= 1;
			if ( !(PORTC.IN & PIN6_bm) ) ++s1_status;

			s2_status <<= 1;
			if ( !(PORTC.IN & PIN4_bm) ) ++s2_status;

			if (s1_status == 1 || s2_status == 1) break;
		}

	} while (1);

	TCC5.CTRLD    = 0;
	TCC5.CTRLE    = 0;
	TCC5.INTFLAGS = TC5_CCAIF_bm;

	ACA.AC0CTRL    = 0;
	ACA.AC0MUXCTRL = 0;
	EVSYS.CH4MUX  = 0;

	return s1_status == 1;
}

/*-----------------------------------------------------------------------*/

int main(void)
{
	PORTD.DIRSET = PIN6_bm;
	PORTD.OUTSET = PIN6_bm;

	setup_XOSC_EXT();
	OSC.CTRL &= ~OSC_RC2MEN_bm;

	/* PA[0..1] : Dip-sw    */
	/* PA4      : NC        */
	/* PA5      : Sensor In */
	/* PA6      : NC        */
	/* PA7      : NC        */
	PORTA.DIR = PIN3_bm | PIN2_bm;
	PORTA.OUT = PIN3_bm;		/* AMP_en = H (disable) */
	PORTA.PIN1CTRL = PORT_OPC_PULLUP_gc;
	PORTA.PIN0CTRL = PORT_OPC_PULLUP_gc;
	PORTA.PIN4CTRL = PORT_OPC_PULLDOWN_gc;
	PORTA.PIN6CTRL = PORT_OPC_PULLDOWN_gc;
	PORTA.PIN7CTRL = PORT_OPC_PULLDOWN_gc;

	/* PC0      : uSD CS    */
	/* PC1      : uSD CLK   */
	/* PC2      : uSD DO    */
	/* PC3      : uSD DI    */
	/* PC4      : forward   */
	/* PC5      : Int/Delay */
	/* PC6      : Start     */
	/* PC7      : NC        */
	PORTC.DIR = PIN3_bm | PIN1_bm | PIN0_bm;
	PORTC.OUT = PIN0_bm;
	PORTC.PIN4CTRL = PORT_OPC_PULLUP_gc;
	PORTC.PIN5CTRL = PORT_OPC_PULLUP_gc;
	PORTC.PIN6CTRL = PORT_OPC_PULLUP_gc;
	PORTC.PIN7CTRL = PORT_OPC_PULLDOWN_gc;

	/* PD[0..4] : NC        */
	/* PD5      : LED CLK   */
	/* PD6      : LED ratch */
	/* PD7      : LED DI    */
	PORTD.DIR = PIN7_bm | PIN6_bm | PIN5_bm;
	PORTD.OUT = PIN6_bm | PIN5_bm;
	PORTD.REMAP = PORT_USART0_bm;

	PORTD.PIN0CTRL = PORT_OPC_PULLDOWN_gc;
	PORTD.PIN1CTRL = PORT_OPC_PULLDOWN_gc;
	PORTD.PIN2CTRL = PORT_OPC_PULLDOWN_gc;
	PORTD.PIN3CTRL = PORT_OPC_PULLDOWN_gc;
	PORTD.PIN4CTRL = PORT_OPC_PULLDOWN_gc;

	init_usart_spi_led();
	set_led_digit(0);

	count_num = 0;

	setupTCC4_10ms();

	/* Enable interrpts. */
	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;

	sei();

	do {
		do {
		} while (idle());

		do {
			if (PORTC.IN & PIN5_bm) delay();
		} while (run());
	} while (1);
}