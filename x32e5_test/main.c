#include <avr/io.h>
#include <avr/cpufunc.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

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
}

void output_spi_led(uint8_t d, uint8_t f)
{
	SPIC.CTRL = SPI_CLK2X_bm | SPI_ENABLE_bm | SPI_MASTER_bm
				| SPI_MODE_0_gc | SPI_PRESCALER_DIV4_gc;

	SPIC.DATA = d;
	do {} while ( !(SPIC.STATUS & SPI_IF_bm) );
	SPIC.CTRL = 0;

	if (f) {
		PORTC.OUTSET = PIN7_bm;
	} else {
		PORTC.OUTCLR = PIN7_bm;
	}
	PORTC.OUTSET = PIN6_bm; /* MISO = H, load to 74AC164 */
	PORTC.OUTTGL = PIN6_bm;	/* MISO = L, ratch & enable 74HC595 */
}

ISR(RTC_OVF_vect)
{

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

	/* Enable interrpts. */
	PMIC.CTRL |= PMIC_LOLVLEN_bm;
}

int main(void)
{
	/* Enable internal 8MHz ring oscillator with low power mode, */
	OSC.CTRL |= OSC_RC8MEN_bm | OSC_RC8MLPM_bm;
	/* and wait until it's stable. */
	do { } while (!( OSC.STATUS & OSC_RC8MRDY_bm ));

	/* set the 8MHz ring oscillator as the main clock source. */
	main_clocksource_select(CLK_SCLKSEL_RC8M_gc);

	init_spi_led();

	setupRTC_ULP();

	set_sleep_mode(SLEEP_MODE_PWR_SAVE);
	sei();
	do {
		output_spi_led(digit[4], 1);

		sleep_mode();
		output_spi_led(digit[3], 0);

		sleep_mode();
		output_spi_led(digit[6], 0);

		sleep_mode();
		output_spi_led(digit[2], 0);

		sleep_mode();
	} while (1);
}