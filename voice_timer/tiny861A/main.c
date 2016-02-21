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
#include <string.h>
#include "pff.h"

#define FCC(c1,c2,c3,c4)	(((DWORD)c4<<24)+((DWORD)c3<<16)+((WORD)c2<<8)+(BYTE)c1)	/* FourCC */

void delay_ms (uint16_t ms);
void delay_us (uint16_t us);

/* Low level SPI control functions */
void init_spi (void);
void xmit_spi_slow (uint8_t);

void select_led (void);
void deselect_led (void);

void setDisplay(uint16_t d)
{
	select_led();
	xmit_spi_slow( (uint8_t)(d >> 8) );
	xmit_spi_slow( (uint8_t)(d  & 0x0ff) );
	deselect_led();
}

/*-----------------------------------------------------------------------*/
/* Work Area                                                             */
/*-----------------------------------------------------------------------*/
volatile uint16_t counter;

volatile uint8_t buzz_pos;
volatile uint8_t buzz_cnt;

volatile BYTE FifoRi, FifoWi, FifoCt;	/* FIFO controls */

BYTE Buff[256];		/* Wave output FIFO */

FATFS Fs;			/* File system object */
DIR Dir;			/* Directory object */
FILINFO Fno;		/* File information */

WORD rb;			/* Return value. Put this here to avoid avr-gcc's bug */

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
	char *bp;

	bp = (char*)Buff;
	while ( (*bp++ = *fn++) ) ;

	res = pf_open((char*)Buff);		/* Open sound file */
	if (res == FR_OK) {
		sz = load_header();			/* Check file format and ready to play */
		if (sz < 1024) {
			return 255;	/* Cannot play this file */
		}

		FifoCt = 0; FifoRi = 0; FifoWi = 0;	/* Reset audio FIFO */

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

	return res;
}

static
uint8_t play_standby(void)
{
	uint8_t noplay = -1;

	/* Powerdown {PRADC} */
	PRR = _BV(PRADC);

	if (pf_mount(&Fs) != FR_OK
	    || pf_opendir(&Dir, "") != FR_OK) {
		return -1;
	}

	/* Start PWM Output */
	PLLCSR = 0b00000010;
	delay_us(110);
	loop_until_bit_is_set(PLLCSR, PLOCK);
	PLLCSR = 0b00000110;	/* Select PLL clock for TC1.ck */

	OCR1B  = 128;
	TCCR1A = 0b00100001;	/* Enable OC1B as PWM */
	TCCR1B = 0b00000001;	/* Start TC1 */

	delay_ms(1);

	// CE = L
	PINB = _BV(6);

	delay_ms(30);

	do {
		if (pf_readdir(&Dir, 0) != FR_OK
		    || pf_readdir(&Dir, &Fno) != FR_OK
		    || !Fno.fname[0]) {
			break;
		}

		if ( !(Fno.fattrib && (AM_DIR|AM_HID))
		    && !strcmp(Fno.fname, "STDBY.WAV") ) {
			if (play(Fno.fname) == FR_OK) {
				noplay = 0;
			}
		}
	} while (noplay);

	// CE = H
	PINB = _BV(6);

	delay_ms(30);

	PLLCSR = TCCR1A = TCCR1B = 0;

	return noplay;
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
			if (prev == 37) {
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

	TIMSK = 0;

	delay_ms(500);

	init_spi();

	for (;;) {
		idle();

		do {
			play_standby();

			delay_ms(1000);

		} while ( run() );
	}
}
