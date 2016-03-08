/*----------------------------------------------------------------------------/
/  ATMega328p
/    8MHz internal RC
/    3.3V
/    BOD Disable
/    WDT Disalbe
/----------------------------------------------------------------------------*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

// PORTB
//		7 : x     : In-Pup
//		6 : x     : In-Pup
//		5 : SCK   : In-open
//		4 : MISO  : Out-High
//		3 : MOSI  : In-open
//		2 : SS    : In-Pup	<= PCINT2
//		1 : dig4  : Out-low
//		0 : x     : In-Pup

// PORTC
//		5 : e     : Out-High
//		4 : c     : Out-High
//		3 : dp    : Out-High
//		2 : d     : Out-High
//		1 : g     : Out-High
//		0 : x     : In-Pup

// PORTD
//		7 : b     : Out-High
//		6 : dig3  : Out-low
//		5 : dig2  : Out-low
//		4 : f     : Out-High
//		3 : a     : Out-High
//		2 : dig1  : Out-low
//		1 : x     : In-Pup
//		0 : x     : In-Pup

void set_display (uint16_t);
uint8_t read_spi (void);

/*---------------------------------------------------------*/
/* Work Area                                               */
/*---------------------------------------------------------*/
// 7セグメントLEDに表示するデータ（フォントデータ）
// PORTC, PORTDの順
uint8_t seg_data[2*4] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

volatile uint8_t row = 0;
volatile uint8_t *sdrp = seg_data;

// volatile uint8_t spi_rd;
// volatile uint8_t spi_rf;

// 7セグメント表示用 ==============================================

const uint8_t seg_font[2*10] = {
//       ecpdg-     b--fa---
	 0b11001011,  0b01100111,  // 0  ABCDEF
	 0b11101111,  0b01111111,  // 1  BC
	 0b11011001,  0b01110111,  // 2  ABDEG
	 0b11101001,  0b01110111,  // 3  ABCDG
	 0b11101101,  0b01101111,  // 4  BCFG
	 0b11101001,  0b11100111,  // 5  ACDFG
	 0b11001001,  0b11100111,  // 6  ACDEFG
	 0b11101111,  0b01110111,  // 7  ABC
	 0b11001001,  0b01100111,  // 8  ABCDEFG
	 0b11101001,  0b01100111   // 9  ABCDFG
};


/*-----------------------------------------------------------------------*/
/* Main                                                                  */
int main (void)
{
 	MCUSR = 0;

	PORTB = 0b11010101;
	DDRB  = 0b00010010;

	PORTC = 0b01111111;
	DDRC  = 0b01111110;

	PORTD = 0b10011011;
	DDRD  = 0b11111100;

	PRR = _BV(PRTWI) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRSPI) | _BV(PRUSART0) | _BV(ADC);


	// Timer2設定
	// LEDのダイナミック点灯(4ms)
	// CTC動作
	// 8MHz / 1024 / 31 = 252Hz (4ms)
	TCCR2A = 0;
	TCCR2B = 0;
	OCR2A = 30;
	TCCR2A = 0b00000010;
	TCCR2B = 0b00001111;
	TIMSK2 = 0b00000010;

	// SS pin interrupt
	PCMSK0 = _BV(PCINT2);
	PCICR = _BV(PCIE0);

	sei();

	for (;;) {
		uint8_t rb1, rb2;

		set_sleep_mode(SLEEP_MODE_PWR_SAVE);
		do {
			sleep_mode();
		} while ( PINB & _BV(2) );

		cli();

		PRR = _BV(PRTWI) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRUSART0) | _BV(ADC);
		SPCR = (1<<SPE);


		rb1 = read_spi();
		rb2 = read_spi();

		SPCR = 0;
		PRR = _BV(PRTWI) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRSPI) | _BV(PRUSART0) | _BV(ADC);

		sei();

 		if (rb2 >= 0xE0) continue;

 		set_display( (((uint16_t)rb2)<<8) + (uint16_t)rb1);
	}
}
