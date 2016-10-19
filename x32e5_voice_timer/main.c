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

volatile uint16_t count = 0;

volatile const uint8_t digit[] = { 0xa0, 0xbe, 0x62, 0x32, 0x3c, 0x31, 0x21, 0xba, 0x20, 0x30};
volatile uint8_t digits[4] = {0xff, 0xff, 0xff, 0xff};

volatile uint8_t tcc4_intflags = 0;

volatile const uint16_t buzz_sampl[] = {
	0xC20, 0xEF0, 0xC20, 0x380, 0x010, 0x3A0, 0xC30, 0xFF0, 0xBC0, 0x430,
	0x110, 0x5E0, 0xC00, 0xD80, 0x860, 0x3A0, 0x440, 0x950, 0xC00, 0x9F0,
	0x640, 0x4D0, 0x6A0, 0x960, 0xAD0, 0x8D0, 0x680, 0x5C0, 0x730, 0xAD0,
	0x9F0, 0x620, 0x400, 0x960, 0xB60, 0x8C0, 0x5F0, 0x8F0, 0xA80, 0x640,
	0x2B0, 0x4A0};

void main_clocksource_select(uint8_t clkCtrl);
void set_led_digit(uint16_t v);

void play_wav(void);

extern unsigned char Timer1, Timer2;

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
	if (intf & TC4_CCAIF_bm) {
		output_usart_spi_led(digits[0], 1);
	}
	if (intf & TC4_CCBIF_bm) {
		output_usart_spi_led(digits[1], 0);
	}
	if (intf & TC4_CCCIF_bm) {
		output_usart_spi_led(digits[2], 0);
	}
	if (intf & TC4_CCDIF_bm) {
		output_usart_spi_led(digits[3], 0);
	}
	tcc4_intflags = 0;
}

ISR(TCC4_OVF_vect)
{
	unsigned char n;
	uint16_t cnt;

	TCC4.INTFLAGS = TC4_OVFIF_bm;

	cnt = TCC5.CNT;
	count = cnt;
	set_led_digit(cnt);

	n = Timer1;				/* 100Hz decrement timer */
	if (n) Timer1 = --n;
	n = Timer2;
	if (n) Timer2 = --n;
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
	TCC4.INTCTRLA = TC45_OVFINTLVL_LO_gc;
	TCC4.INTCTRLB = TC45_CCDINTLVL_LO_gc | TC45_CCCINTLVL_LO_gc | TC45_CCBINTLVL_LO_gc | TC45_CCAINTLVL_LO_gc;
	TCC4.INTFLAGS = TC4_CCDIF_bm | TC4_CCCIF_bm | TC4_CCBIF_bm | TC4_CCAIF_bm | TC4_OVFIF_bm;
	TCC4.CTRLA = TC45_CLKSEL_DIV8_gc;
}

void timer_proc(void)
{
	uint8_t intf = tcc4_intflags;

	if ( !(PORTC.IN & PIN6_bm) ) {
		TCC5.CNT = 0;
	}

	if (TCC5.INTFLAGS & TC5_OVFIF_bm) {
		TCC5.INTFLAGS = TC5_OVFIF_bm;
		TCC5.CNT -= 50000;
	}

	if (intf) update_led(intf);
}

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
	PORTD.OUT = PIN5_bm;
	PORTD.REMAP = PORT_USART0_bm;

	PORTD.PIN0CTRL = PORT_OPC_PULLDOWN_gc;
	PORTD.PIN1CTRL = PORT_OPC_PULLDOWN_gc;
	PORTD.PIN2CTRL = PORT_OPC_PULLDOWN_gc;
	PORTD.PIN3CTRL = PORT_OPC_PULLDOWN_gc;
	PORTD.PIN4CTRL = PORT_OPC_PULLDOWN_gc;

	init_usart_spi_led();

	setupTCC4_10ms();

	/* Enable interrpts. */
	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_HILVLEN_bm;
	set_sleep_mode(SLEEP_MODE_IDLE);
	sei();

	set_led_digit(0);
	play_wav();

	do {
		sleep_mode();
		timer_proc();
	} while (1);
}