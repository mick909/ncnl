#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#define SCK 5
#define RCK 4
#define DATA 3

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

EMPTY_INTERRUPT(WDT_vect);

void init_spi_led(void)
{
/* Clear 74HC595 */
	SPCR  = 0b01011100;	/* SPIE=0, SPE=1, MSTR=1, CPOL=SPHA=0, SPR=osc/2 */
	SPSR  = 0b00000001;	/* SPI2X=1 */

	SPDR  = 0xff;
	do {} while ( !(SPSR & 0b10000000) );
	SPCR = 0;			/* SPI Disable */

/* Clear 74HC164 */
	PORTB &= ~0b00001000;
	PINB  = 0b00010000;	/* MISO output L */
	PINB  = 0b00010000;	/* MISO output H -> load */
	PINB  = 0b00010000;	/* MISO output L */
	PINB  = 0b00010000;	/* MISO output H -> load  */
	PINB  = 0b00010000;	/* MISO output L */
	PINB  = 0b00010000;	/* MISO output H -> load  */
	PINB  = 0b00010000;	/* MISO output L */
	PINB  = 0b00010000;	/* MISO output H -> load  */
	PINB  = 0b00010000;	/* MISO output L */
}

void output_spi_led(uint8_t d, uint8_t f)
{
	SPCR  = 0b01011100;	/* SPIE=0, SPE=1, MSTR=1, CPOL=SPHA=0, SPR=osc/2 */
	SPSR  = 0b00000001;	/* SPI2X=1 */

	SPDR  = d;
	do {} while ( !(SPSR & 0b10000000) );
	SPCR = 0;			/* SPI Disable */

	PORTB &= ~0b00001000;			/* setup MOSI as 74HC164's data */
	if (f) PORTB |= 0b00001000;

	PINB  = 0b00010000;	/* MISO output H */
	PINB  = 0b00010000;	/* MISO output L */
}

int main(void)
{
	// PB7 : XTAL2
	// PB6 : XTAL1
	// PB5 : SCK  - 74HC595:SCK                / Output, High
	// PB4 : MISO - 74HC595:RCK,G  74HC164:CLK / Output, High, (& PullDown)
	// PB3 : MOSI - 74HC595:SI     74HC164:A,B / Output, Low
	// PB2 : SS   - Output, High ... not use
	// PB1 : x    - Input, PU
	// PB0 : x    - Input, PU
	PORTB = 0b00111111;
	DDRB  = 0b00111100;

	PORTC = 0b00111111;
	DDRC  = 0b00000000;

	PORTD = 0b11111111;
	DDRD  = 0b00000000;

	init_spi_led();

	do {
		output_spi_led(digit[8], 1);

		wdt_reset();
		WDTCSR = _BV(WDCE) | _BV(WDE);
		WDTCSR = _BV(WDIE) | 0b110;		// 1s
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);
		sei();
		sleep_mode();
		cli();

		output_spi_led(digit[9] & 0xdf, 0);

		wdt_reset();
		WDTCSR = _BV(WDCE) | _BV(WDE);
		WDTCSR = _BV(WDIE) | 0b110;		// 1s
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);
		sei();
		sleep_mode();
		cli();

		output_spi_led(digit[6], 0);

		wdt_reset();
		WDTCSR = _BV(WDCE) | _BV(WDE);
		WDTCSR = _BV(WDIE) | 0b110;		// 1s
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);
		sei();
		sleep_mode();
		cli();

		output_spi_led(digit[7], 0);

		wdt_reset();
		WDTCSR = _BV(WDCE) | _BV(WDE);
		WDTCSR = _BV(WDIE) | 0b110;		// 1s
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);
		sei();
		sleep_mode();
		cli();
	} while (1);

}