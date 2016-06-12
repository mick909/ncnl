#include <avr/io.h>
#include <avr/cpufunc.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

volatile uint32_t counter10ms;

volatile uint8_t digits[4] = {0x28, 0x28, 0x28, 0x28};
volatile uint8_t col;

/*         0 1 2 3 4 5 6 7 8 9
          ---------------------
  Q7 : e   0 1 0 1 1 1 0 1 0 1
  Q6 : d   0 1 0 0 1 0 0 1 0 0
  Q5 : DP  1 1 1 1 1 1 1 1 1 1
  Q4 : c   0 0 1 0 0 0 0 0 0 0
  Q3 : g   1 1 0 0 0 0 0 1 0 0
  Q2 : a   0 1 0 0 1 0 0 0 0 0
  Q1 : f   0 1 1 1 0 0 0 1 0 0
  Q0 : b   0 0 0 0 0 1 1 0 0 0
*/

volatile const uint8_t digit[] = { 0x28, 0xee, 0x32, 0xa2, 0xe4, 0xa1, 0x21, 0xea, 0x20, 0xa0};

void main_clocksource_select(uint8_t clkCtrl);
void clock_prescaler_select(uint8_t psConfig);

void setup_RC8M_LPM(void)
{
	/* Enable internal 8MHz ring oscillator with low power mode, */
	OSC.CTRL |= OSC_RC8MEN_bm | OSC_RC8MLPM_bm;
	/* and wait until it's stable. */
	do { } while (!( OSC.STATUS & OSC_RC8MRDY_bm ));

	/* set the 8MHz ring oscillator as the main clock source. */
	main_clocksource_select(CLK_SCLKSEL_RC8M_gc);
}

void setup_RC32M_DIV4(void)
{
	OSC.XOSCCTRL = OSC_X32KLPM_bm | OSC_XOSCSEL_32KHz_gc;
	OSC.CTRL |= OSC_XOSCEN_bm;
	do { } while (!( OSC.STATUS & OSC_XOSCRDY_bm ));

	/* Enable internal 32MHz ring oscillator, */
	OSC.CTRL |= OSC_RC32MEN_bm;
	clock_prescaler_select(CLK_PSADIV_4_gc | CLK_PSBCDIV_1_1_gc);

	/* and wait until it's stable. */
	do { } while (!( OSC.STATUS & OSC_RC32MRDY_bm ));

	OSC.DFLLCTRL = OSC_RC32MCREF_XOSC32K_gc;
	DFLLRC32M.COMP1 = 0x12;
	DFLLRC32M.COMP2 = 0x7a;
	DFLLRC32M.CTRL |= DFLL_ENABLE_bm;

	/* set the 32MHz ring oscillator as the main clock source. */
	main_clocksource_select(CLK_SCLKSEL_RC32M_gc);
}

void init_spi_led(void)
{
	// PC4 : SS		/ Output, Low
	// PC5 : SCK	/ Output, Low
	// PC6 : MISO	/ Output, Low (with external PullDown)
	// PC7 : MOSI	/ Output, Low
	PORTC.DIRSET = PIN4_bm | PIN5_bm | PIN6_bm | PIN7_bm;
	SPIC.CTRL = SPI_CLK2X_bm | SPI_ENABLE_bm | SPI_MASTER_bm
				| SPI_MODE_0_gc | SPI_PRESCALER_DIV4_gc;

	SPIC.DATA = 0xff;
	do {} while ( !(SPIC.STATUS & SPI_IF_bm) );
	SPIC.CTRL = 0;

	PORTC.OUTCLR = PIN7_bm;	/* MOSI = L to 74AC164 */

	PORTC.OUTSET = PIN6_bm; /* MISO = H, load to 74AC164 */
	PORTC.OUTTGL = PIN6_bm;	/* MISO = L */
	PORTC.OUTTGL = PIN6_bm; /* MISO = H, load to 74AC164 */
	PORTC.OUTTGL = PIN6_bm;	/* MISO = L */
	PORTC.OUTTGL = PIN6_bm; /* MISO = H, load to 74AC164 */
	PORTC.OUTTGL = PIN6_bm;	/* MISO = L */
	PORTC.OUTTGL = PIN6_bm; /* MISO = H, load to 74AC164 */
	PORTC.OUTTGL = PIN6_bm;	/* MISO = L, ratch & enable 74HC595 */

	col = 0;
}

void output_spi_led(uint8_t d, uint8_t f)
{
	SPIC.CTRL = SPI_CLK2X_bm | SPI_ENABLE_bm | SPI_MASTER_bm
				| SPI_MODE_0_gc | SPI_PRESCALER_DIV4_gc;

	SPIC.DATA = d;
	do {} while ( !(SPIC.STATUS & SPI_IF_bm) );
	SPIC.CTRL = 0;

	if (f) {
		PORTC.OUTCLR = PIN7_bm;
	} else {
		PORTC.OUTSET = PIN7_bm;
	}
	PORTC.OUTSET = PIN6_bm; /* MISO = H, load to 74AC164 */
	PORTC.OUTTGL = PIN6_bm;	/* MISO = L, ratch & enable 74HC595 */
}

ISR(RTC_OVF_vect)
{
	output_spi_led(digits[col], col);
	col = (col + 1) & 0x03;
}

ISR(TCC4_OVF_vect)
{
	TCC4.INTFLAGS = TC4_OVFIF_bm;	/* Must do this at XMEGA? */

	uint32_t cnt = counter10ms + 1;

	if (cnt > 199999) cnt -= 100000;
	counter10ms = cnt;

	cnt /= 10;
	digits[3] = digit[cnt % 10];

	cnt /= 10;
	digits[2] = digit[cnt % 10];

	cnt /= 10;
	digits[1] = digit[cnt % 10];;

	cnt /= 10;
	digits[0] = digit[cnt % 10];;
}

void setupRTC_ULP(void)
{
	/* Set internal ULP 1kHz as clock source for RTC. */
	CLK.RTCCTRL = CLK_RTCSRC_ULP_gc | CLK_RTCEN_bm;

	/* Wait until RTC is not busy. */
	do {} while ( RTC.STATUS & RTC_SYNCBUSY_bm );

	/* Configure RTC period to 4ms. */
	RTC.PER  = 4 - 1;
	RTC.CNT  = 0;
	RTC.COMP = 0;
	RTC.CTRL = ( RTC.CTRL & ~RTC_PRESCALER_gm ) | RTC_PRESCALER_DIV1_gc;

	/* Enable overflow interrupt. */
	RTC.INTCTRL = RTC_OVFINTLVL_LO_gc;
}

void setupTCC4_10ms(void)
{
//	TCC4.CTRLB = TC45_BYTEM_NORMAL_gc | TC45_CIRCEN_DISABLE_gc | TC45_WGMODE_NORMAL_gc;
	TCC4.CNT = 0;
	TCC4.PER = 1250 - 1;		/* 8MHz div64 / 1250 = 100hz */
	TCC4.INTCTRLA = (uint8_t)TC45_OVFINTLVL_HI_gc;
	TCC4.CTRLA = TC45_CLKSEL_DIV64_gc;
}

int main(void)
{
	setup_RC32M_DIV4();
//	setup_RC8M_LPM();
	OSC.CTRL &= ~OSC_RC2MEN_bm;

	PORTC.PIN0CTRL = PORT_OPC_PULLUP_gc;

	init_spi_led();

	counter10ms = 0;

	setupRTC_ULP();
	/* Enable interrpts. */
	PMIC.CTRL |= PMIC_HILVLEN_bm | PMIC_LOLVLEN_bm;
	sei();

	_delay_ms(1);
	do {} while ( (PORTC.IN & PIN0_bm) == PIN0_bm );
	_delay_ms(1);
	do {} while ( (PORTC.IN & PIN0_bm) != PIN0_bm );

	cli();
	setupTCC4_10ms();

//	set_sleep_mode(SLEEP_MODE_PWR_SAVE);
	set_sleep_mode(SLEEP_MODE_IDLE);
	sei();
	do {
		sleep_mode();
		_NOP();
	} while (1);
}