/*----------------------------------------------------------------------------/
/  8-pin SD audio player R0.05d                    (C)ChaN, 2011              /
/    Modofied to ATtiny861A                        (C)Mick, 2016              /
/-----------------------------------------------------------------------------/
/ This project, program codes and circuit diagrams, is opened under license
/ policy of following trems.
/
/  Copyright (C) 2010, ChaN, all right reserved.
/
/ * This project is a free software and there is NO WARRANTY.
/ * No restriction on use. You can use, modify and redistribute it for
/   personal, non-profit or commercial use UNDER YOUR RESPONSIBILITY.
/ * Redistributions of source code must retain the above copyright notice.
/
/----------------------------------------------------------------------------*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <string.h>
#include "pff.h"

// FUSES = {0xE1, 0xDD, 0xFF};	/* Fuse bytes for mono: Low, High and Extended */

#define FCC(c1,c2,c3,c4)	(((DWORD)c4<<24)+((DWORD)c3<<16)+((WORD)c2<<8)+(BYTE)c1)	/* FourCC */

void delay_us (WORD);	/* Defined in asmfunc.S */

/*---------------------------------------------------------*/
/* Work Area                                               */
/*---------------------------------------------------------*/

volatile BYTE FifoRi, FifoWi, FifoCt;	/* FIFO controls */

BYTE Buff[256];		/* Wave output FIFO */

FATFS Fs;			/* File system object */
DIR Dir;			/* Directory object */
FILINFO Fno;		/* File information */

WORD rb;			/* Return value. Put this here to avoid avr-gcc's bug */

static
void my_lseek(DWORD sz)
{
	while (sz) {
		pf_read(Buff, (sz > 256) ? 256 : sz, &rb);
		sz -= rb;
	}
}
/*---------------------------------------------------------*/
static
DWORD load_header (void)	/* 0:Invalid format, 1:I/O error, >=1024:Number of samples */
{
	DWORD sz, f;
	BYTE b, al = 0;

	if (pf_read(Buff, 12, &rb)) return 1;	/* Load file header (12 bytes) */

	if (rb != 12 || LD_DWORD(Buff+8) != FCC('W','A','V','E')) return 0;

	for (;;) {
		wdt_reset();
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
//			pf_lseek(Fs.fptr + sz);			/* Skip this chunk */
			my_lseek(sz);
			break;

		default :						/* Unknown chunk */
			return 0;
		}
	}

	return 0;
}


static
void ramp (
	int dir		/* 0:Ramp-down, 1:Ramp-up */
)
{
	BYTE v, d, n;

	if (dir) {
		v = 0; d = 1;
	} else {
		v = 128; d = (BYTE)-1;
	}

	n = 128;
	do {
		v += d;
		OCR1B = v;
		delay_us(100);
	} while (--n);
}

static
FRESULT play (
	const char *dir,	/* Directory */
	const char *fn		/* File */
)
{
	DWORD sz;
	FRESULT res;
	WORD btr;
	char *bp;

	wdt_reset();

//	xsprintf((char*)Buff, PSTR("%s/%s"), dir, fn);
	bp = (char*)Buff;
	while ( (*bp++ = *dir++) ) ;
	*(bp-1) = '/';
	while ( (*bp++ = *fn++) ) ;

	res = pf_open((char*)Buff);		/* Open sound file */
	if (res == FR_OK) {
		sz = load_header();			/* Check file format and ready to play */
		if (sz < 1024) {
			return 255;	/* Cannot play this file */
		}

		FifoCt = 0; FifoRi = 0; FifoWi = 0;	/* Reset audio FIFO */

		if (!TCCR1A) {				/* Enable audio out if not enabled */
			PLLCSR = 0b00000110;	/* Select PLL clock for TC1.ck */
			TCCR1A = MODE ? 0b10100011 : 0b00100001;	/* Enable OC1B as PWM */
			TCCR1B = 0b00000001;	/* Start TC1 */

			TCCR0A = 0b00000001;
			TCCR0B = 0b00000010;

			TIMSK = _BV(OCIE0A);
			ramp(1);
		}

		pf_read(0, 512 - (Fs.fptr % 512), &rb);	/* Snip sector unaligned part */
		sz -= rb;

		do {	/* Data transfer loop */
			wdt_reset();

			btr = (sz > 1024) ? 1024 : (WORD)sz;/* A chunk of audio data */
			res = pf_read(0, btr, &rb);	/* Forward the data into audio FIFO */
			if (rb != 1024) break;		/* Break on error or end of data */

			sz -= rb;					/* Decrease data counter */
		} while (1); // ((PINB & 1) || ++sw != 1);
	}

	while (FifoCt) ;			/* Wait for audio FIFO empty */
	OCR1B = 128;				/* Return output to center level */

	return res;
}


static
void delay500 (void)
{
	wdt_reset();

	TCCR0B = 0; TCCR0A = 0;	/* Stop TC0 */

	if (TCCR1A) {	/* Stop TC1 if enabled */
		ramp(0);
		TCCR1A = 0; TCCR1B = 0;
	}

	WDTCR = _BV(WDE) | _BV(WDIE) | 0b101;	/* Set WDT to interrupt mode in timeout of 0.5s */
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);	/* Enter power down mode */
	sleep_mode();

	wdt_reset();
	WDTCR = _BV(WDE) | 0b110;				/* Set WDT to reset mode in timeout of 1s */
}

EMPTY_INTERRUPT(WDT_vect);

/*-----------------------------------------------------------------------*/
/* Main                                                                  */

int main (void)
{
	FRESULT res;
	char *dir;
//	BYTE org_osc = OSCCAL;

	MCUSR = 0;
	WDTCR = _BV(WDE) | 0b110;	/* Enable WDT reset in timeout of 1s */

	PORTA = 0b11111001;			/* PA0: DI-In-Pullup, PA1: DO Out-low */
	DDRA  = 0b11111110;			/* PA2: USCK: OUT-low PA4: SS Out-high*/

	PORTB = 0b11110111;
	DDRB  = 0b11111101;			/* PB1:In-PullUp, PB3: PWM: Out */

	sei();

	for (;;) {
		if (pf_mount(&Fs) == FR_OK) {	/* Initialize FS */
			wdt_reset();
//			Buff[0] = 0;
//			if (!pf_open("osccal")) pf_read(Buff, 1, &rb);	/* Adjust frequency */
//			OSCCAL = org_osc + Buff[0];

//			res = pf_opendir(&Dir, dir = "wav");	/* Open sound file directory */
//			if (res == FR_NO_PATH) {
			res = pf_opendir(&Dir, dir = "");	/* Open root directory */
//			}

			while (res == FR_OK) {				/* Repeat in the dir */
				res = pf_readdir(&Dir, 0);			/* Rewind dir */
				while (res == FR_OK) {				/* Play all wav files in the dir */
					wdt_reset();
					res = pf_readdir(&Dir, &Fno);		/* Get a dir entry */
					if (res || !Fno.fname[0]) {
						break;	/* Break on error or end of dir */
					}
					if (!(Fno.fattrib & (AM_DIR|AM_HID)) && strstr(Fno.fname, ".WAV")) {
						res = play(dir, Fno.fname);		/* Play file */
					}
				}
			}
		}
		delay500();			/* Delay 500ms in low power sleep mode */
	}
}

