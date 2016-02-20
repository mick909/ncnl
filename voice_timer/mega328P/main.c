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
//		7 : x     : In-Hiz
//		6 : x     : In-Hiz
//		5 : SCK   : In-open
//		4 : MISO  : Out-High
//		3 : MOSI  : In-open
//		2 : SS    : In-open
//		1 : dig4  : Out-low
//		0 : x     : In-Hiz

// PORTC
//		5 : e     : Out-High
//		4 : c     : Out-High
//		3 : dp    : Out-High
//		2 : d     : Out-High
//		1 : g     : Out-High
//		0 : x     : In-Hiz

// PORTD
//		7 : b     : Out-High
//		6 : dig3  : Out-low
//		5 : dig2  : Out-low
//		4 : f     : Out-High
//		3 : a     : Out-High
//		2 : dig1  : Out-low
//		1 : x     : In-Hiz
//		0 : x     : In-Hiz

/*---------------------------------------------------------*/
/* Work Area                                               */
/*---------------------------------------------------------*/
volatile uint16_t counter;

// 7セグメントLEDに表示するデータ（フォントデータ）
// PORTC, PORTDの順
uint8_t segData[2*4] = {0};

// 7セグメント表示用 ==============================================

const uint8_t segNumFont[2*10] = {
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

static
void setupled(void)
{
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
}

// 7セグメント表示用配列に、数値のフォントを設定する
void setTime(uint8_t b4, uint8_t b3, uint8_t b2, uint8_t b1) {
	uint8_t* dp = segData;

	volatile const uint8_t* fp = segNumFont + (b1 * 2);
	*dp++ = *fp++;
	*dp++ = *fp;

	fp = segNumFont + (b2 * 2);
	*dp++ = *fp++;
	*dp++ = *fp;

	fp = segNumFont + (b3 * 2);
	*dp++ = *fp++ & (~0b00001000);
	*dp++ = *fp;

	if (b4 != 0) {
		fp = segNumFont + (b4 * 2);
		*dp++ = *fp++;
		*dp   = *fp;
	} else {
		*dp++ = 0xff;
		*dp   = 0xff;
	}
}

// Timer2割り込み
ISR(TIMER2_COMPA_vect) {
	static uint8_t row = 0;
	static uint8_t *sdrp = segData;

	// DIG-1〜4をオフ
	PORTB &= ~0b00000010;
	PORTD &= ~0b01100100;

	// 7セグフォントをピンに出力する
	// （アノードコモンなので、フォント側は不論理）
	PORTC |= 0b00111110;
	PORTD |= 0b10011000;

	PORTC &= *sdrp++;
	PORTD &= *sdrp++;

	// 該当の桁のアノードにHigh出力
	// ついでに2ndBeep設定中の桁の点滅と、2桁目のピリオドの点滅を処理
	switch (row++) {
	case 0:
		PORTB |= 0b00000010;
		break;
	case 1:
		PORTD |= 0b01000000;
		break;
	case 2:
		PORTD |= 0b00100000;
		break;
	case 3:
		PORTD |= 0b00000100;

		// 桁をオーバーラップする（注意！）
		row = 0;
		sdrp = segData;
		break;
	}

}

/*-----------------------------------------------------------------------*/
/* Main                                                                  */

static
void init(void)
{
  MCUSR = 0;
//		5 : SCK
//		4 : MISO
//		3 : MOSI
//		2 : SS
	PORTB = 0b11010011;
	DDRB  = 0b11010011;

	PORTC = 0b11111111;
	DDRC  = 0b11111111;

	PORTD = 0b11111111;
	DDRD  = 0b11111111;
}

int main (void)
{
	init();

	setupled();

	SPCR = _BV(SPE);
	SPSR = 0;

	sei();

	for (;;) {
		uint8_t rb1;
		uint8_t rb2;
		uint16_t rb;
		uint16_t b4;
		uint16_t b3;
		uint16_t b2;
		uint16_t b1;
		uint16_t b0;

		while (!(SPSR & (1<<SPIF))) {
//			set_sleep_mode(SLEEP_MODE_IDLE);
// 			sleep_enable();
// 			sleep_cpu();
// 			sleep_disable();
 		}

 		rb1 = SPDR;
 		while (!(SPSR & (1<<SPIF))) ;
 		rb2 = SPDR;
 		rb = ((uint16_t)rb1 << 8) + ((uint16_t)rb2 & 0x0ff);

 		b4 = rb / 10000;
 		rb -= b4 * 10000;

 		b3 = rb / 1000;
 		rb -= b3 * 1000;

 		b2 = rb / 100;
 		rb -= b2 * 100;

 		b1 = rb / 10;
 		b0 = rb - b1 * 10;

 		setTime(b3,b2,b1,b0);
	}
}
