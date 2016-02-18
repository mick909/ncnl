/*----------------------------------------------------------------------------/
/
/----------------------------------------------------------------------------*/

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

#define B_STSW	2
#define B_DISW	1

// 動作状態
#define IDLE    0
#define DELAY   1
#define RUNNING 2
#define SLEEP   5

/* Low level SPI control functions */
void init_spi (void);
void xmit_spi_slow (uint8_t);

/*---------------------------------------------------------*/
/* Work Area                                               */
/*---------------------------------------------------------*/
volatile uint16_t counter;

/*-----------------------------------------------------------------------*/
/* Main                                                                  */

static
void set_10ms_timer(void)
{
	TCCR0B = 0b00011011;		/* clkI/O / 64 */
	TCCR0A = 0;
	TIMSK  = 0;

	TCCR0A = _BV(TCW0);			/* 16bit operation */
	TCNT0H = ((-(F_CPU / 64 / 100)) >> 8);
	TCNT0L = ((-(F_CPU / 64 / 100)) & 0x00ff);

	TIFR   = _BV(TOV0);
	TIMSK  = _BV(TOIE0);

	counter = 0;
}

static
void init(void)
{
	MCUSR = 0;

	PORTA = 0b11111001;			/* PA0: DI-In-Pullup, PA1: DO Out-low */
	DDRA  = 0b11111110;			/* PA2: USCK: OUT-low PA4: SS Out-high*/

	PORTB = 0b11110111;
	DDRB  = 0b11111001;			/* PB1:In-PullUp, PB3: PWM: Out */

	set_10ms_timer();
	TCCR0B = 0b00000011;		/* Timer0 Start */
}

static
uint8_t toIdle(void)
{
	// disableComparator();

	return IDLE;
}

static
uint8_t toRun(void)
{
	set_10ms_timer();
	TCCR0B = 0b00000011;		/* Timer0 Start */

	return RUNNING;
}

static
void setDisplay(uint16_t d)
{
	PORTB &= ~0b01000000;
	xmit_spi_slow( (uint8_t)(d >> 8) );
	xmit_spi_slow( (uint8_t)(d  & 0x0ff) );
	PORTB |=  0b01000000;
}

int main (void)
{
	uint16_t prev;

	uint8_t startsw_state = 0;
	uint8_t dispsw_state = 0;

	uint8_t state = SLEEP, next = IDLE;

	init();

	init_spi();

	prev = counter;

	uint16_t test = 0;
	uint16_t test2 = 0;

	sei();

	for (;;) {
		do {
			set_sleep_mode(SLEEP_MODE_IDLE);
 			sleep_enable();
 			sleep_cpu();
 			sleep_disable();
		} while (counter == prev);
		prev = counter;

		startsw_state <<= 1;
		if (bit_is_clear(PINB, B_STSW)) startsw_state |= 1;
		dispsw_state <<= 1;
		if (bit_is_clear(PINB, B_DISW)) dispsw_state |= 1;

		if (++test >= 100) {
			test = 0;
			setDisplay(++test2);
			if (test2 >= 20000) test2 = 0;
		}

		switch (state) {
			case IDLE:
			if (startsw_state == 0x7f) {
				next = RUNNING;
			}
			if (dispsw_state == 0x7f) {
				// setDisplay(0);
			}
			break;

			case RUNNING:
			if (startsw_state == 0x7f) {
				state = IDLE;
				next = RUNNING;
			}
			if (dispsw_state == 0x7f) {
				next = IDLE;
			}

			setDisplay(prev);
			break;
		}

		if (state != next) {
			if (next == IDLE) {
				state = toIdle();
			}
			if (next == RUNNING) {
				state = toRun();
			}
		}
	}
}
