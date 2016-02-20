/*
  ATtiny861a
    12MHz external xtal clock
    3.3V
    BOD Disable
    WDT Disalbe

  PA0 : DI/MOSI  : In-Pullup
  PA1 : DO/MISO  : Out-low
  PA2 : SCK      : OUT-low
  PA3 : MMC_CS   : Out-high
  PA4 : LED_SS   : Out-high
  PA5 : DIP sw1  : In-Pullup
  PA6 : DIP sw2  : In-Pullup
  PA7 : Sensor   : In-Open

  PB0 : StartSw  : In-Pullup
  PB1 : DispSw   : In-Pullup
  PB2 : ModeSw   : In-Pullup
  PB3 : SPK PWM  : Out-low
  PB4 : xtal
  PB5 : xtal
  PB6 : AMP En   : Out-low
  PB7 : reset
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

void delay_ms (uint16_t ms);

/* Low level SPI control functions */
void init_spi (void);
void xmit_spi_slow (uint8_t);

void setDisplay(uint16_t d)
{
	PORTA &= ~0b00010000;
	xmit_spi_slow( (uint8_t)(d >> 8) );
	xmit_spi_slow( (uint8_t)(d  & 0x0ff) );
	PORTB |=  0b00010000;
}

/*-----------------------------------------------------------------------*/
/* Work Area                                                             */
/*-----------------------------------------------------------------------*/
volatile uint16_t counter;

volatile uint8_t buzz_pos;
volatile uint8_t buzz_cnt;

/*-----------------------------------------------------------------------*/
/* Xorshift pseudo random generator                                      */
/*-----------------------------------------------------------------------*/
volatile uint32_t y = 2463534242;

uint32_t xorshift(void)
{
	y ^= (y<<13); y ^= (y >> 17); y ^= (y << 5);
	return y;
}

/*-----------------------------------------------------------------------*/
/* Reset Watchdog on boot                                                */
/*-----------------------------------------------------------------------*/
void get_mcusr(void) __attribute__((naked)) __attribute__((section(".init3")));
void get_mcusr(void)
{
	MCUSR = 0;
	wdt_disable();
}

/*-----------------------------------------------------------------------*/
/* Interrupts                                                            */
/*-----------------------------------------------------------------------*/

EMPTY_INTERRUPT(WDT_vect);

/*-----------------------------------------------------------------------*/
/* IDLE Mode                                                             */
/*-----------------------------------------------------------------------*/
/*
 * Wakeup per 16ms (WDT Interrupt)
 * Check switch
 * proceed random sequence
 *
 * Peripheral : WDT
 */
static
void idle(void)
{
	uint8_t disp_sw = 0;

	/* Powerdown {PRTIM1, PRTIM0, PRADC} */
	PRR = _BV(PRTIM1) | _BV(PRTIM0) | _BV(PRADC);

	/* Display " 0.00" */
	setDisplay(0 + 20000);

	/* Powerdown {PRTIM1, PRTIM0, PRUSI, PRADC} */
	PRR = _BV(PRTIM1) | _BV(PRTIM0) | _BV(PRUSI) | _BV(PRADC);

	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sei();

	do {
		/* Set WDT to interrupt mode in timeout of 16ms */
		wdt_reset();
		WDTCR = _BV(WDE) | _BV(WDIE) | 0b000;

		/* Enter powerdown mode */
		/* Only wakeup by WDT */
		sleep_mode();

		xorshift();

		disp_sw <<= 1;
		if (!(PINB & _BV(1))) ++disp_sw;
	} while ( PINB & _BV(0) );

	cli();

	/* Disable WDT */
	wdt_reset();
	WDTCR = _BV(WDCE) | _BV(WDE);
	WDTCR = 0;
 }

/*-----------------------------------------------------------------------*/
/* VOICE Mode                                                            */
/*-----------------------------------------------------------------------*/
/*
 * 1: speak voice from SD card
 *     Peripheral : Timer0(sampling), Timer1(Fast PWM), USI(MMC IF)
 *
 * 2: Random Delay
 *     Peripheral : No
 */

/*-----------------------------------------------------------------------*/
/* RUN Mode                                                              */
/*-----------------------------------------------------------------------*/
/*
 * 1: buzzer 0.3 sec
 *     Peripheral : Timer0(count), Timer1(BuzzerPWM), USI(LED)
 *
 * 2: Count
 *     Peripheral : Timer0(count), USI(LED), ADC
 */
static
uint8_t run(void)
{
	uint8_t start_sw = 0xff;
	uint8_t disp_sw = 0xff;

	uint16_t prev = 0;

	/* Powerdown {PRTIM0, PRADC} */
	PRR = _BV(PRTIM0) | _BV(PRADC);

	setDisplay(0);

	/* AMP Standby */
	OCR1B = 128;
	TCNT1 = 0;
	TCCR1A = _BV(COM1B1) | _BV(PWM1B);
	TCCR1B = _BV(CS10);

	delay_ms(1);

	PINB = _BV(6);

	delay_ms(30);
	/***************/

	/* Powerdown {} */
	PRR = 0;

	/* Set 10ms Timer Interrupt */
	TCCR0A = _BV(TCW0);			/* 16bit operation */
	TCCR0B = _BV(TSM) | _BV(PSR0) | _BV(CS01) | _BV(CS00);
	TCNT0H = ((-(F_CPU / 64 / 100)) >> 8);
	TCNT0L = ((-(F_CPU / 64 / 100)) & 0x00ff);

	counter = 0;
	buzz_cnt = buzz_pos = 0;

	TIMSK = _BV(TOIE0) | _BV(TOIE1);
	TCCR0B = _BV(CS01) | _BV(CS00);

	set_sleep_mode(SLEEP_MODE_IDLE);
	sei();
	do {
		sleep_mode();

		if (prev != counter) {
			prev = counter;
			setDisplay(prev);
		}
	} while (prev < 30);

	/* STOP BUZZER */
	TIMSK = _BV(TOIE0);
	OCR1B = 128;

	do {
		sleep_mode();

		if (prev != counter) {
			prev = counter;
			setDisplay(prev);

			if (prev == 33) {
				PINB = _BV(6);
			}
			if (prev == 34) {
				TCCR1A = TCCR1B = 0;
				PRR = _BV(PRTIM1);
			}
			if (prev > 200) {
				start_sw <<= 1; if (!(PINB & _BV(0))) ++start_sw;
				disp_sw <<= 1; if (!(PINB & _BV(1))) ++disp_sw;
			}
		}
	} while ( start_sw != 0b0111 && disp_sw != 0b0111);

	cli();

	delay_ms(1);

	TIMSK = 0;
	TCCR0A = TCCR0B = 0;

	if (start_sw == 0b0111) return 1;
	return 0;
}

/*-----------------------------------------------------------------------*/
/* Main                                                                  */
/*-----------------------------------------------------------------------*/
int main (void)
{
	PORTA = 0b01111001;
	DDRA  = 0b00011110;

	PORTB = 0b00000111;
	DDRB  = 0b01001000;

	delay_ms(500);

	init_spi();

	for (;;) {
		idle();

		do {

		} while ( run() );
	}
}
