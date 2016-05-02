/*
  ATtiny861a
    12MHz external xtal clock
    3.3V
    BOD Disable
    WDT Disalbe

  LFUSE : 0xDF  : CKDIV8=dis, CKOUT=dis, SUT=01, CKSEL=1111
  HFUSE : 0xDD  : RST=1, DebugWire=dis, SPI=en, WDT=off, EEPROM=clear, BOD=dis
  EFUSE : 0x01  : SELFPRGEN - disable

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
#include "pff.h"

#define FCC(c1,c2,c3,c4)	(((DWORD)c4<<24)+((DWORD)c3<<16)+((WORD)c2<<8)+(BYTE)c1)	/* FourCC */

void delay_ms (uint16_t ms);
void delay_us (uint16_t us);

void enable_ac (void);
void disable_ac (void);


/*-----------------------------------------------------------------------*/
/* Work Area                                                             */
/*-----------------------------------------------------------------------*/
volatile uint16_t counter;

volatile uint16_t buzz_time;
volatile uint8_t buzz_pos;
volatile uint8_t buzz_cnt;
volatile uint8_t buzz_shut;

volatile uint8_t count_num;

volatile uint8_t ac_blank;

volatile BYTE FifoRi, FifoWi, FifoCt;	/* FIFO controls */

volatile uint16_t counts[10];

BYTE Buff[256];		/* Wave output FIFO */

FATFS Fs;			/* File system object */

WORD rb;			/* Return value. Put this here to avoid avr-gcc's bug */


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

ISR(PCINT_vect)
{
	PCMSK1 = GIMSK = 0;
}

/*-----------------------------------------------------------------------*/
/* SPI LED CONTROL                                                       */
/*-----------------------------------------------------------------------*/
/* Low level SPI control functions */
void init_spi (void);
void xmit_spi_slow (uint8_t);

void select_led (void);
void deselect_led (void);

void setDisplay(uint16_t d)
{
	select_led();
	delay_us(10);
	xmit_spi_slow( (uint8_t)(d  & 0x0ff) );
	xmit_spi_slow( (uint8_t)(d >> 8) );
	deselect_led();
}


/*-----------------------------------------------------------------------*/
/* MMC Access & Play Wav                                                 */
/*-----------------------------------------------------------------------*/

static
void my_lseek(uint16_t sz)
{
	while (sz) {
		pf_read(Buff, (sz > 256) ? 256 : sz, &rb);
		sz -= rb;
	}
}

static
DWORD load_header (void)	/* 0:Invalid format, 1:I/O error, >=1024:Number of samples */
{
	DWORD sz, f;
	BYTE b, al = 0;

	if (pf_read(Buff, 12, &rb)) return 1;	/* Load file header (12 bytes) */

	if (rb != 12 || LD_DWORD(Buff+8) != FCC('W','A','V','E')) return 0;

	for (;;) {
		pf_read(Buff, 8, &rb);			/* Get Chunk ID and size */
		if (rb != 8) return 0;
		sz = LD_DWORD(&Buff[4]);		/* Chunk size */

		switch (LD_DWORD(&Buff[0])) {	/* Switch by chunk ID */
		case FCC('f','m','t',' ') :					/* 'fmt ' chunk */
			if (sz & 1) sz++;						/* Align chunk size */
			if (sz > 100 || sz < 16) return 0;		/* Check chunk size */
			pf_read(Buff, sz, &rb);					/* Get content */
			if (rb != sz) return 0;
			if (Buff[0] != 1) return 0;				/* Check coding type (LPCM) */
			b = Buff[2];
			if (b != 1 && b != 2) return 0;			/* Check channels (1/2) */
			GPIOR0 = al = b;						/* Save channel flag */
			b = Buff[14];
			if (b != 8 && b != 16) return 0;		/* Check resolution (8/16 bit) */
			GPIOR0 |= b;							/* Save resolution flag */
			if (b & 16) al <<= 1;
			f = LD_DWORD(&Buff[4]);					/* Check sampling freqency (8k-48k) */
			if (f < 8000 || f > 48000) return 4;
			OCR0A = (BYTE)(F_CPU / 8 / f) - 1;		/* Set sampling interval */
			break;

		case FCC('d','a','t','a') :		/* 'data' chunk */
			if (!al) return 0;							/* Check if format is valid */
			if (sz < 1024 || (sz & (al - 1))) return 0;	/* Check size */
			if (Fs.fptr & (al - 1)) return 0;			/* Check word alignment */
			return sz;									/* Start to play */

		case FCC('D','I','S','P') :		/* 'DISP' chunk */
		case FCC('L','I','S','T') :		/* 'LIST' chunk */
		case FCC('f','a','c','t') :		/* 'fact' chunk */
			if (sz & 1) sz++;				/* Align chunk size */
			my_lseek(sz);
			break;

		default :						/* Unknown chunk */
			return 0;
		}
	}

	return 0;
}

static
FRESULT play (
	const char *fn		/* File */
)
{
	DWORD sz;
	FRESULT res;
	WORD btr;

	res = pf_open(fn);		/* Open sound file */
	if (res == FR_OK) {
		sz = load_header();			/* Check file format and ready to play */
		if (sz < 1024) {
			return 255;	/* Cannot play this file */
		}

		{
			set_sleep_mode(SLEEP_MODE_PWR_DOWN);

			/* Set WDT to interrupt mode in timeout of 1s */
			wdt_reset();
			WDTCR = _BV(WDE) | _BV(WDIE) | 0b110;

			sei();
			sleep_mode();
			cli();

			/* Disable WDT */
			wdt_reset();
			WDTCR = _BV(WDCE) | _BV(WDE);
			WDTCR = 0;
		}

		FifoCt = 0; FifoRi = 0; FifoWi = 0;	/* Reset audio FIFO */

		delay_ms(500);

		/* Start PWM Output */
		PLLCSR = 0b00000010;
		delay_us(110);
		loop_until_bit_is_set(PLLCSR, PLOCK);
		PLLCSR = 0b00000110;	/* Select PLL clock for TC1.ck */

		OCR1B  = 128;
		TCCR1A = 0b00100001;	/* Enable OC1B as PWM */
		TCCR1B = 0b00000001;	/* Start TC1 */

		delay_ms(50);

		// CE = L
		PORTB |= _BV(6);

		delay_ms(30);

		/* Sampling Interrupt Start */
		TCCR0A = 0b00000001;
		TCCR0B = 0b00000010;
		TIMSK = _BV(OCIE0A);

		sei();

		pf_read(0, 512 - (Fs.fptr % 512), &rb);	/* Snip sector unaligned part */
		sz -= rb;

		do {	/* Data transfer loop */
			btr = (sz > 1024) ? 1024 : (WORD)sz;/* A chunk of audio data */
			res = pf_read(0, btr, &rb);	/* Forward the data into audio FIFO */
			if (rb != 1024) break;		/* Break on error or end of data */

			sz -= rb;					/* Decrease data counter */
		} while (1); // ((PINB & 1) || ++sw != 1);
	}

	while (FifoCt) ;			/* Wait for audio FIFO empty */

	cli();

	/* Sampling Interrupt Stop */
	TIMSK = 0;
	TCCR0A = TCCR0B = 0;

	OCR1B = 128;				/* Return output to center level */

	// CE = H
	PORTB &= ~(_BV(6));

	delay_ms(30);

	PLLCSR = TCCR1A = TCCR1B = 0;

	return res;
}


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
/* Wait switch release                                                   */
/*-----------------------------------------------------------------------*/
void wait_sw_release (void)
{
	uint8_t disp_sw = 0;

	do {
		/* Set WDT to interrupt mode in timeout of 16ms */
		wdt_reset();
		WDTCR = _BV(WDE) | _BV(WDIE) | 0b000;

		sleep_mode();
		xorshift();

		disp_sw <<= 1;
		if (PINB & _BV(1)) ++disp_sw;
	} while (disp_sw != 0xff);
}


/*-----------------------------------------------------------------------*/
/* IDLE Mode (with SLEEP)                                                */
/*-----------------------------------------------------------------------*/
/*
 * Wakeup per 1s (WDT Interrupt)
 * Check switch
 * proceed random sequence
 *
 * Peripheral : WDT
 */
static
void idle(void)
{
//	uint8_t disp_sw = 0xff;
	uint8_t count_pos;
	uint8_t pwrdwn;
	uint8_t tmp;
	uint8_t blink = 1;

	/* Powerdown {PRTIM1, PRTIM0, PRADC} */
	PRR = _BV(PRTIM1) | _BV(PRTIM0) | _BV(PRADC);

	delay_ms(20);

	count_pos = (count_num) ? 1 : 0;
	if (GPIOR2) {
		setDisplay(0xff00);
		pwrdwn = 0;
	} else {
		setDisplay(counts[count_pos]);
		pwrdwn = 180;
	}
	GPIOR2 = 0;

	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sei();
	wait_sw_release();

	do {
		PCMSK1 = _BV(PCINT8) | _BV(PCINT9);
		GIMSK = _BV(PCIE0);

		/* Set WDT to interrupt mode in timeout of 1s */
		wdt_reset();
		WDTCR = _BV(WDE) | _BV(WDIE) | 0b110;

		/* Enter powerdown mode */
		sleep_mode();
		tmp = PINB;

		xorshift();

		if ( (tmp & 0b011) == 0b011 ) {
			if (pwrdwn) {
				--pwrdwn;
				setDisplay(counts[count_pos]);
			} else {
				setDisplay( (0xff ^ blink) << 8 );
				blink ^= 1;
			}
			continue;
		}

		if ( !(tmp & _BV(1)) ) {
			if (pwrdwn) {
				if (count_num) {
					if (++count_pos > count_num) count_pos = 1;
				}
			}
			setDisplay(counts[count_pos]);

			wait_sw_release();
			pwrdwn = 180;
		}
	} while ( tmp & _BV(0) );

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
static
void voice_delay(void)
{
	uint8_t delay_count = 48;

	/* Powerdown {PRADC} */
	PRR = _BV(PRADC);

	delay_ms(20);

	/* Display "- - - -" */
	setDisplay(0xf000);

	if (pf_mount(&Fs) == FR_OK && play("/STDBY.WAV") == FR_OK) {
		delay_count = 5;
	}

	/* Powerdown {PRTIM1, PRTIM0, PRADC} */
	PRR = _BV(PRTIM1) | _BV(PRTIM0) | _BV(PRADC);

	delay_count += (uint8_t)(xorshift() & 0x0f) + (uint8_t)(xorshift() & 0x07) + (uint8_t)(xorshift() & 0x03);

	TIMSK = 0;
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sei();

	while(delay_count-- > 0) {
		/* Set WDT to interrupt mode in timeout of 64ms */
		wdt_reset();
		WDTCR = _BV(WDE) | _BV(WDIE) | 0b010;

//		setDisplay(delay_count * 6.4);

		/* Enter powerdown mode */
		/* Only wakeup by WDT */
		sleep_mode();
	}

	cli();

	/* Disable WDT */
	wdt_reset();
	WDTCR = _BV(WDCE) | _BV(WDE);
	WDTCR = 0;
}

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
	uint8_t count50 = 50;
	uint8_t dot = 1;

	/* Powerdown {PRTIM0, PRADC} */
	PRR = _BV(PRTIM0) | _BV(PRADC);

	/* Display " 0.00" */
	setDisplay(0);

	/* AMP Standby */
	OCR1B = 128;
	TCCR1A = _BV(COM1B1) | _BV(PWM1B);
	TCCR1B = _BV(CS10);

	delay_ms(50);

	PORTB |= _BV(6);

	delay_ms(30);
	/***************/

//	/* Powerdown {} */
//	PRR = 0;
	/* Powerdown {PRADC} */
	PRR = _BV(PRADC);

	GPIOR1 = ((((PINA >> 5) & 0b00000011) + 2) % 4) + 1;

	/* Set 10ms Timer Interrupt */
	TCCR0A = _BV(TCW0);			/* 16bit operation */
	TCCR0B = _BV(TSM) | _BV(PSR0) | _BV(CS01) | _BV(CS00);
	TCNT0H = ((-(F_CPU / 64 / 100)) >> 8);
	TCNT0L = ((-(F_CPU / 64 / 100)) & 0x00ff);

	counter = 0;
	buzz_cnt = buzz_pos = buzz_shut = ac_blank = count_num = 0;
	buzz_time = (47875 * 0.4);

	TIMSK = _BV(TOIE0) | _BV(TOIE1);
	TCCR0B = _BV(CS01) | _BV(CS00);

	enable_ac();

	set_sleep_mode(SLEEP_MODE_IDLE);
	sei();
	do {
		uint16_t tmp;
		uint8_t num;

		sleep_mode();
		cli();
		tmp = counter;
		num = count_num;
		sei();

		if (prev != tmp) {
			if (num == 0) {
				setDisplay(tmp + (dot?0:20000));
			} else if (num == 1 && prev <= counts[num]) {
				setDisplay(counts[num] + (dot?0:20000));
			}

			prev = tmp;

			if (num == 9) {
				break;
			}

			if (prev == 18000) {
				GPIOR2 = 1;
				break;
			}

			if (--count50 == 0) {
				count50 = 50;
				dot ^= 1;
				if (num) {
					setDisplay(counts[1] + (dot?0:20000));
				}
			}

			if (prev > 200) {
				start_sw <<= 1; if (!(PINB & _BV(0))) ++start_sw;
				disp_sw <<= 1; if (!(PINB & _BV(1))) ++disp_sw;
			}
		}
	} while (start_sw != 0b0111 && disp_sw != 0b0111);

	cli();

	disable_ac();

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

	TIMSK = 0;

	ACSRA = 0b10000000;		/* Analog Compalator : Power down */
	ADMUX = 0b10000000;		/* REFS=010 : BG=1.1V */
	DIDR0 = 0b10000000;		/* AIN1 : digial input disable */

	counts[0] = 0;
	count_num = 0;

	delay_ms(500);

	init_spi();

	GPIOR2 = 0;

	for (;;) {
		idle();

		do {
			if ( (PINB & _BV(2)) ) voice_delay();

		} while ( run() );
	}
}
