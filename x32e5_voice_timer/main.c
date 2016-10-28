#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "ff.h"

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

volatile uint16_t counts[10] = {0};
volatile uint8_t  count_num;

void main_clocksource_select(uint8_t clkCtrl);
void set_led_digit(uint16_t v);

FRESULT play_wav(void);

/*-----------------------------------------------------------------------*/
/* Clock source select                                                   */
/*-----------------------------------------------------------------------*/
static
void setup_RC32K(void)
{
	if (CLK_CTRL == CLK_SCLKSEL_RC32K_gc) return;

	/* Enable 32.768Hz Internal Oscillator */
	OSC.CTRL |= OSC_RC32KEN_bm;

	/* wait until stable. */
	do {} while ( !(OSC.STATUS & OSC_RC32KRDY_bm) );
	main_clocksource_select(CLK_SCLKSEL_RC32K_gc);

	OSC.CTRL = OSC_RC32KEN_bm;
}

static
void setup_RC2M(void)
{
	if (CLK_CTRL == CLK_SCLKSEL_RC2M_gc) return;

	/* Enable 2MHz Internal Oscillator */
	OSC.CTRL |= OSC_RC2MEN_bm;

	/* wait until stable. */
	do {} while ( !(OSC.STATUS & OSC_RC2MRDY_bm) );
	main_clocksource_select(CLK_SCLKSEL_RC2M_gc);

	OSC.CTRL = OSC_RC2MEN_bm;
}

static
void setup_XOSC_EXT(void)
{
	if (CLK_CTRL == CLK_SCLKSEL_XOSC_gc) return;

	/* Select XTAL1 for external oscillator. */
	OSC.XOSCCTRL = OSC_FRQRANGE_9TO12_gc | OSC_XOSCSEL_EXTCLK_gc;

	/* Enable External Clock Sounrce */
	OSC.CTRL |= OSC_XOSCEN_bm;

	/* wait until stable. */
	do {} while ( !(OSC.STATUS & OSC_XOSCRDY_bm) );
	main_clocksource_select(CLK_SCLKSEL_XOSC_gc);

	OSC.CTRL = OSC_XOSCEN_bm;
}

/*-----------------------------------------------------------------------*/
/* Xorshift pseudo random generator                                      */
/*-----------------------------------------------------------------------*/
volatile uint32_t y = 2463534242;

uint16_t xorshift(void)
{
  y ^= (y<<13); y ^= (y >> 17); y ^= (y << 5);
  return (uint16_t)y;
}

/*-----------------------------------------------------------------------*/
/* 7Seg dynamic driver with shift regigster                              */
/*-----------------------------------------------------------------------*/
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
void output_led(uint8_t data, uint8_t flag)
{
	for (uint8_t c=0; c<8; ++c, data<<=1) {
		PORTD.OUTCLR = PIN5_bm;
		if (data & 0x80) {
			PORTD.OUTSET = PIN7_bm;
		} else {
			PORTD.OUTCLR = PIN7_bm;
		}
		PORTD.OUTSET = PIN5_bm;
	}

	if (flag) {
		PORTD.OUTSET = PIN7_bm;
	} else {
		PORTD.OUTCLR = PIN7_bm;
	}

	PORTD.OUTSET = PIN6_bm; /* MISO = H, load to 74AC164 */
	PORTD.OUTTGL = PIN6_bm;	/* MISO = L */
}

/*-----------------------------------------------------------------------*/
/* RTC ticks                                                             */
/*-----------------------------------------------------------------------*/
volatile uint8_t rtccnt;
volatile uint8_t rtcflag;

extern uint8_t Timer1, Timer2;	/* 100Hz decrement timer */

ISR(RTC_OVF_vect)
{
	uint8_t n = rtccnt;

	n = (n + 1) & 0x03;
	rtccnt = n;
	rtcflag = n;

	n = Timer1;				/* 100Hz decrement timer */
	if (n) Timer1 = --n;
	n = Timer2;
	if (n) Timer2 = --n;
}

/*-----------------------------------------------------------------------*/
/* SLEEP Mode                                                            */
/*-----------------------------------------------------------------------*/
ISR(PORTC_INT_vect)
{
	PORTC.INTCTRL = 0;
	PORTC.INTMASK  = 0;
	PORTC.PIN4CTRL = PORT_OPC_PULLUP_gc;
	PORTC.PIN6CTRL = PORT_OPC_PULLUP_gc;
}

static
uint8_t sleep(void)
{
	output_led(0xff, 0);
	PORTD.OUTSET = PIN6_bm; /* MISO = H, load to 74AC164 */
	PORTD.OUTTGL = PIN6_bm;	/* MISO = L */
	PORTD.OUTTGL = PIN6_bm; /* MISO = H, load to 74AC164 */
	PORTD.OUTTGL = PIN6_bm;	/* MISO = L */
	PORTD.OUTSET = PIN6_bm; /* MISO = H, load to 74AC164 */
	PORTD.OUTTGL = PIN6_bm;	/* MISO = L */

	setup_RC32K();

	/* Use peripherals
	    RTC : dot blink timer
	 */

	/* enable RTC & clocl source select ULP 1kHz */
	PR.PRGEN = PR_XCL_bm | PR_EVSYS_bm | PR_EDMA_bm;
	PR.PRPA  = PR_DAC_bm | PR_ADC_bm | PR_AC_bm;
	PR.PRPC  = PR_TWI_bm | PR_USART0_bm | PR_SPI_bm | PR_HIRES_bm | PR_TC5_bm | PR_TC4_bm;
	PR.PRPD  = PR_USART0_bm | PR_TC5_bm;

	CLK.RTCCTRL = CLK_RTCSRC_ULP_gc | CLK_RTCEN_bm;
	do {} while ( RTC.STATUS & RTC_SYNCBUSY_bm );

	RTC.PER = 1000-1;		/* 1 Sec */
	RTC.CNT = 0;
	RTC.INTFLAGS = RTC_OVFIF_bm;
	RTC.INTCTRL  = RTC_OVFINTLVL_LO_gc;
	RTC.CTRL     = RTC_PRESCALER_DIV1_gc;

	/* setup PIN interrupt (PC4, PC6) */
	PORTC.PIN4CTRL = PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc;
	PORTC.PIN6CTRL = PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc;
	PORTC.INTFLAGS = 0xff;
	PORTC.INTMASK  = PIN4_bm | PIN6_bm;
	PORTC.INTCTRL  = PORT_INTLVL_LO_gc;

	set_sleep_mode(SLEEP_MODE_PWR_SAVE);

	do {
		/* display '.' to DIG3 */
		PORTD.OUTSET = PIN7_bm;	/* MOSI = L to 74AC164 */

		PORTD.OUTSET = PIN6_bm; /* MISO = H, load to 74AC164 */
		PORTD.OUTTGL = PIN6_bm;	/* MISO = L */

		PORTD.OUTCLR = PIN7_bm;

		PORTD.OUTSET = PIN6_bm; /* MISO = H, load to 74AC164 */
		PORTD.OUTTGL = PIN6_bm;	/* MISO = L */

		output_led(0xdf, 0);

		sleep_mode();

		/* switch interrupt -> exit */
		if (PORTC.INTFLAGS) break;

		/* all DIGs off */
		output_led(0xff, 0);

		sleep_mode();

		/* switch interrupt -> exit */
		if (PORTC.INTFLAGS) break;
	} while (1);

	do {} while ( RTC.STATUS & RTC_SYNCBUSY_bm );
	RTC.CTRL    = 0;
	do {} while ( RTC.STATUS & RTC_SYNCBUSY_bm );
	RTC.INTCTRL = 0;
	CLK.RTCCTRL = 0;

	return !(PORTC.INTFLAGS & PORT_INT6IF_bm);
}

/*-----------------------------------------------------------------------*/
/* IDLE Mode                                                             */
/*-----------------------------------------------------------------------*/
static
uint8_t idle(void)
{
	uint16_t idlecnt;
	uint8_t count_pos;
	uint8_t s1_status = 0xff;
	uint8_t s2_status = 0xff;

	output_led(0xff, 0);
	PORTD.OUTSET = PIN6_bm; /* MISO = H, load to 74AC164 */
	PORTD.OUTTGL = PIN6_bm;	/* MISO = L */
	PORTD.OUTTGL = PIN6_bm; /* MISO = H, load to 74AC164 */
	PORTD.OUTTGL = PIN6_bm;	/* MISO = L */
	PORTD.OUTSET = PIN6_bm; /* MISO = H, load to 74AC164 */
	PORTD.OUTTGL = PIN6_bm;	/* MISO = L */

	setup_RC2M();

	/* Use peripherals
	    RTC : 4ms LED drive timer
	 */

	/* enable RTC & clocl source select ULP 1kHz */
	PR.PRGEN = PR_XCL_bm | PR_EVSYS_bm | PR_EDMA_bm;
	PR.PRPA  = PR_DAC_bm | PR_ADC_bm | PR_AC_bm;
	PR.PRPC  = PR_TWI_bm | PR_USART0_bm | PR_SPI_bm | PR_HIRES_bm | PR_TC5_bm | PR_TC4_bm;
	PR.PRPD  = PR_USART0_bm | PR_TC5_bm;

	CLK.RTCCTRL = CLK_RTCSRC_ULP_gc | CLK_RTCEN_bm;
	do {} while ( RTC.STATUS & RTC_SYNCBUSY_bm );

	RTC.PER = 4-1;		/* 4ms */
	RTC.CNT = 0;
	RTC.INTFLAGS = RTC_OVFIF_bm;
	RTC.INTCTRL  = RTC_OVFINTLVL_LO_gc;
	RTC.CTRL     = RTC_PRESCALER_DIV1_gc;

	rtccnt = 0;
	rtcflag = 4;

	count_pos = (count_num) ? 1 : 0;
	set_led_digit(counts[count_pos]);

	idlecnt = 18750;	/* 5 * 60 * 1000 / 4 / 4 */

	set_sleep_mode(SLEEP_MODE_PWR_SAVE);

	do {
		uint8_t flag;

		sleep_mode();

		flag = rtcflag;
		rtcflag = 4;

		xorshift();

		if (flag == 4) continue;

		if (!flag && --idlecnt == 0) break;

		uint8_t dig = digits[flag];
		if (flag == 2) dig &= 0xdf;
		output_led(dig, (flag==0));

		s1_status = (s1_status << 1) & 0x3f;
		if ( !(PORTC.IN & PIN6_bm) ) ++s1_status;

		s2_status = (s2_status << 1) & 0x3f;
		if ( !(PORTC.IN & PIN4_bm) ) ++s2_status;

		if (s2_status == 1) {
			if (count_num) {
				if (++count_pos > count_num) count_pos = 1;
				set_led_digit(counts[count_pos]);
			}

			idlecnt = 18750;
		}
	} while (s1_status != 1);

	do {} while ( RTC.STATUS & RTC_SYNCBUSY_bm );
	RTC.CTRL    = 0;
	do {} while ( RTC.STATUS & RTC_SYNCBUSY_bm );
	RTC.INTCTRL = 0;
	CLK.RTCCTRL = 0;

	return s1_status != 1;
}

/*-----------------------------------------------------------------------*/
/* DELAY Mode                                                            */
/*-----------------------------------------------------------------------*/
static
void delay(void)
{
	uint16_t delay = 0;

	/* set dispay '----' (static) */
	output_led(0x7f, 1);
	PORTD.OUTSET = PIN6_bm; /* MISO = H, load to 74AC164 */
	PORTD.OUTTGL = PIN6_bm;	/* MISO = L */
	PORTD.OUTTGL = PIN6_bm; /* MISO = H, load to 74AC164 */
	PORTD.OUTTGL = PIN6_bm;	/* MISO = L */
	PORTD.OUTSET = PIN6_bm; /* MISO = H, load to 74AC164 */
	PORTD.OUTTGL = PIN6_bm;	/* MISO = L */

	/* 1st delay 0.5〜1.0 sec */

	setup_RC32K();

	/* Use peripherals
	    RTC : delay wait timer
	 */
	PR.PRGEN = PR_XCL_bm | PR_EVSYS_bm | PR_EDMA_bm;
	PR.PRPA  = PR_DAC_bm | PR_ADC_bm | PR_AC_bm;
	PR.PRPC  = PR_TWI_bm | PR_USART0_bm | PR_SPI_bm | PR_HIRES_bm | PR_TC5_bm | PR_TC4_bm;
	PR.PRPD  = PR_USART0_bm | PR_TC5_bm;

	CLK.RTCCTRL = CLK_RTCSRC_ULP_gc | CLK_RTCEN_bm;
	do {} while ( RTC.STATUS & RTC_SYNCBUSY_bm );

	RTC.PER      = (xorshift() & (512-1)) + 500;
	RTC.CNT      = 0;
	RTC.INTFLAGS = RTC_OVFIF_bm;
	RTC.INTCTRL  = RTC_OVFINTLVL_LO_gc;
	RTC.CTRL     = RTC_PRESCALER_DIV1_gc;

	set_sleep_mode(SLEEP_MODE_PWR_SAVE);
	sleep_mode();

	do {} while ( RTC.STATUS & RTC_SYNCBUSY_bm );
	RTC.CTRL    = 0;
	do {} while ( RTC.STATUS & RTC_SYNCBUSY_bm );

	/* play wav */

	setup_XOSC_EXT();

	/* Use peripherals
	    RTC    : 10ms timer
	    EDMA   : SPI master
	    DAC    : wav output
	    USARTC : SPI master
	    TCD5   : wav sampling rate
	 */
	PR.PRGEN = PR_XCL_bm | PR_EVSYS_bm;
	PR.PRPA  = PR_ADC_bm | PR_AC_bm;
	PR.PRPC  = PR_TWI_bm | PR_SPI_bm | PR_HIRES_bm | PR_TC5_bm | PR_TC4_bm;
	PR.PRPD  = PR_USART0_bm;

	set_sleep_mode(SLEEP_MODE_IDLE);

	RTC.PER      = 10-1;		/* 10 ms */
	RTC.CNT      = 0;
	RTC.INTFLAGS = RTC_OVFIF_bm;
	RTC.CTRL     = RTC_PRESCALER_DIV1_gc;

	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_HILVLEN_bm;
	if ( play_wav() ) {
		delay = 1500;
	}
	PMIC.CTRL = PMIC_LOLVLEN_bm;

	do {} while ( RTC.STATUS & RTC_SYNCBUSY_bm );
	RTC.CTRL    = 0;
	do {} while ( RTC.STATUS & RTC_SYNCBUSY_bm );

	/* 2nd delay 1.2 + rand(1.7) sec */

	setup_RC32K();

	/* Use peripherals
	    RTC : delay wait timer
	 */
	PR.PRGEN = PR_XCL_bm | PR_EVSYS_bm | PR_EDMA_bm;
	PR.PRPA  = PR_DAC_bm | PR_ADC_bm | PR_AC_bm;
	PR.PRPC  = PR_TWI_bm | PR_USART0_bm | PR_SPI_bm | PR_HIRES_bm | PR_TC5_bm | PR_TC4_bm;
	PR.PRPD  = PR_USART0_bm | PR_TC5_bm;

	RTC.PER      = (xorshift() & (1024-1)) + (xorshift() & (512-1)) + 1200 + delay;
	RTC.CNT      = 0;
	RTC.INTFLAGS = RTC_OVFIF_bm;
	RTC.CTRL     = RTC_PRESCALER_DIV1_gc;

	set_sleep_mode(SLEEP_MODE_PWR_SAVE);
	sleep_mode();

	do {} while ( RTC.STATUS & RTC_SYNCBUSY_bm );
	RTC.CTRL    = 0;
	do {} while ( RTC.STATUS & RTC_SYNCBUSY_bm );
	RTC.INTCTRL = 0;
	CLK.RTCCTRL = 0;
}

/*-----------------------------------------------------------------------*/
/* RUN Mode                                                              */
/*-----------------------------------------------------------------------*/
volatile const uint16_t buzz_sampl[] = {
	0xC20, 0xEF0, 0xC20, 0x380, 0x010, 0x3A0, 0xC30, 0xFF0, 0xBC0, 0x430,
	0x110, 0x5E0, 0xC00, 0xD80, 0x860, 0x3A0, 0x440, 0x950, 0xC00, 0x9F0,
	0x640, 0x4D0, 0x6A0, 0x960, 0xAD0, 0x8D0, 0x680, 0x5C0, 0x730, 0xAD0,
	0x9F0, 0x620, 0x400, 0x960, 0xB60, 0x8C0, 0x5F0, 0x8F0, 0xA80, 0x640,
	0x2B0, 0x4A0
};

volatile uint8_t buzz_cnt;

ISR(EDMA_CH1_vect)
{
	if (--buzz_cnt) {
		EDMA.CH1.CTRLB    = EDMA_CH_TRNIF_bm | 0x01;
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

		PR.PRGEN = PR_XCL_bm | PR_RTC_bm | PR_EDMA_bm;
		PR.PRPA  = PR_DAC_bm | PR_ADC_bm;
		PR.PRPC  = PR_TWI_bm | PR_USART0_bm | PR_SPI_bm | PR_HIRES_bm;
		PR.PRPD  = PR_TC5_bm;
	}
}

volatile uint8_t tcc4_intflags = 0;

static
uint8_t run(void)
{
	uint8_t s1_status = 0xff;
	uint8_t s2_status = 0xff;
	uint8_t dot = 0xdf;
	uint8_t dotcnt = 50;
	uint8_t ac_blank;
	uint16_t idlecnt;

	setup_XOSC_EXT();

	/* Use peripherals
	    EVSYS  : AC event
	    EDMA   : Buzzer -> auto off
	    DAC    : Buzzer -> auto off
	    AC     : Sensor
	    TCC4   : LED drive & 10ms timing
	    TCC5   : 10ms counter & AC event capture
	    USARTD : LED drive
	    TCD5   : Buzzer -> auto off
	 */
	PR.PRGEN = PR_XCL_bm | PR_RTC_bm;
	PR.PRPA  = PR_ADC_bm;
	PR.PRPC  = PR_TWI_bm | PR_USART0_bm | PR_SPI_bm | PR_HIRES_bm;
	PR.PRPD  = 0;

	set_sleep_mode(SLEEP_MODE_IDLE);

	init_usart_spi_led();

	/* setup capture for AC */
	TCC5.CTRLD = TC45_EVSEL_CH4_gc;
	TCC5.CTRLE = TC45_CCAMODE_CAPT_gc;
	TCC5.INTFLAGS = TC5_CCAIF_bm;

	ACA.AC0MUXCTRL = AC_MUXPOS_PIN5_gc | AC_MUXNEG_BANDGAP_gc;
	ACA.AC0CTRL    = AC_INTMODE_RISING_gc | AC_HYSMODE_LARGE_gc | AC_ENABLE_bm;
	EVSYS.CH4MUX   = EVSYS_CHMUX_ACA_CH0_gc;

	dot = 0xdf;
	count_num = 0;

	/* start buzzer & start counter! */

	/* (0.5(sec) / (42(sample) / 16000Hz)) / 2(repeat) = 95 */
	buzz_cnt = 95;

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

	EDMA.CH1.CTRLB    = EDMA_CH_TRNIF_bm | 0x01;
	EDMA.CH1.CTRLA    = EDMA_CH_ENABLE_bm | EDMA_CH_REPEAT_bm
						| EDMA_CH_SINGLE_bm | EDMA_CH_BURSTLEN_bm;

	TCD5.CTRLB = TC45_BYTEM_NORMAL_gc | TC45_CIRCEN_DISABLE_gc | TC45_WGMODE_NORMAL_gc;
	TCD5.CNT   = 0;
	TCD5.PER   = (10000000/16000) - 1;
	TCD5.CTRLA = TC45_CLKSEL_DIV1_gc;
	EVSYS.CH1MUX = EVSYS_CHMUX_TCD5_OVF_gc;


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
	TCC4.INTCTRLB = TC45_CCDINTLVL_LO_gc | TC45_CCCINTLVL_LO_gc | TC45_CCBINTLVL_LO_gc | TC45_CCAINTLVL_LO_gc;
	TCC4.INTFLAGS = TC4_CCDIF_bm | TC4_CCCIF_bm | TC4_CCBIF_bm | TC4_CCAIF_bm | TC4_OVFIF_bm;
	TCC4.CTRLA = TC45_CLKSEL_DIV8_gc;

	EVSYS.CH0MUX = EVSYS_CHMUX_TCC4_OVF_gc;

	/* Normal Operation */
	/* EV CH0 */
	/* TOP = 60000 */
	TCC5.CTRLB = TC45_BYTEM_NORMAL_gc | TC45_CIRCEN_DISABLE_gc | TC45_WGMODE_NORMAL_gc;
	TCC5.CNT = 0;
	TCC5.PER = 0xffff;
	TCC5.CTRLA = TC45_CLKSEL_EVCH0_gc;


	ac_blank = 0;
	idlecnt = 30000;

	do {
		uint16_t count;
		uint8_t intf;

		sleep_mode();

		intf = tcc4_intflags;

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
			ac_blank = 10 + 1;
		}

		if (ac_blank) {
			if ( !(--ac_blank) ) {
				EVSYS.CH4MUX  = EVSYS_CHMUX_ACA_CH0_gc;;
			}
		}

		/* check 10ms interval */
		if (TCC4.INTFLAGS & TC4_OVFIF_bm) {
			TCC4.INTFLAGS = TC4_OVFIF_bm;

			if (--idlecnt == 0) break;

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

			if (s1_status == 1 || s2_status == 1) {
				if (TCC5.CNT > 60) break;
			}
		}

		if (intf) {
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
	} while (1);

	ACA.AC0CTRL    = 0;
	ACA.AC0MUXCTRL = 0;
	EVSYS.CH4MUX  = 0;

	TCC5.CTRLD    = 0;
	TCC5.CTRLE    = 0;
	TCC5.INTFLAGS = TC5_CCAIF_bm;

	TCC4.INTCTRLB = 0;
	TCC4.CTRLA    = 0;
	TCD5.INTCTRLB = 0;
	TCD5.CTRLA    = 0;

	USARTD0.CTRLC = 0;

	if (idlecnt == 0) return !sleep();

	return s1_status == 1;
}

/*-----------------------------------------------------------------------*/

int main(void)
{
	PORTD.DIRSET = PIN6_bm;
	PORTD.OUTSET = PIN6_bm;

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
	PORTA.PIN5CTRL = PORT_ISC_INPUT_DISABLE_gc;
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
	USARTD0.CTRLC = 0;

	PR.PRGEN = PR_XCL_bm | PR_RTC_bm | PR_EVSYS_bm | PR_EDMA_bm;
	PR.PRPA  = PR_DAC_bm | PR_ADC_bm | PR_AC_bm;
	PR.PRPC  = PR_TWI_bm | PR_USART0_bm | PR_SPI_bm | PR_HIRES_bm | PR_TC5_bm | PR_TC4_bm;
	PR.PRPD  = PR_USART0_bm | PR_TC5_bm;

	count_num = 0;

	/* Enable interrpts. */
	PMIC.CTRL = PMIC_LOLVLEN_bm;

	sei();

	do {
		do {
		} while (idle() && sleep());

		do {
			if (PORTC.IN & PIN5_bm) delay();
		} while (run());
	} while (1);
}