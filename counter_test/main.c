#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>

ISR(TIMER1_COMPA_vect)
{
	PINB = 0b00000010;
}

int main(void)
{
	/* PB1 : */
	PORTB = 0b00111101;
	DDRB  = 0b00000010;

	PORTC = 0b00111111;
	DDRC  = 0b00000000;

	/* PD5 : OC0B -> T1 */
	PORTD = 0b11011111;
	DDRD  = 0b00100000;

	/* Setup : count T1 UpEdge */
	TCNT1  = 0;
	OCR1A  = 100 - 1;
	TCCR1A = 0b00000000;  /* WGM1 = 0100 */
	TIFR1  = 0b00100111;
//	TIMSK1 = 0b00100000;
	TIMSK1 = 0b00000010;  /* COMPA(TOP) Interrupt */
	/* start cont later */

	/* Setup : Buzzer */

	/* 12,288,000 / 1024 / 60 / 2 = 100Hz(1ms) */
	/* output 50% duty pulse to OC0B */
	/* BOTTOM = L, COMPB = H */
	TCNT0  = 0;
	OCR0A  = 60- 1;
	OCR0B  = 30 - 1;
	TCCR0A = 0b00110001;  /* COM0B = 11, WGM0 = 101 */
	TIFR0  = 0b00000111;
//	TIMSK0 = 0b00000111;
	TIMSK0 = 0b00000000;
	TCCR0B = 0b00001101;  /* WGM0 = 111, CS = clk/1024 */

	do {} while ( !(TIFR0 & 0b00000010) );
	TIFR0  = 0b00000111;

	/* Buzzer On Here */

	/* Start : count T1 */
	TCCR1B = 0b00001111;  /* WGM1 = 0100, CS = 111 */

	sei();

	do {
		;
	} while (1);

}