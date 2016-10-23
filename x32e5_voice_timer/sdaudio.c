#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include "sdaudio.h"
#include "ff.h"

#pragma GCC diagnostic ignored "-Wstrict-aliasing"

#define FCC(c1,c2,c3,c4)	(((DWORD)c4<<24)+((DWORD)c3<<16)+((WORD)c2<<8)+(BYTE)c1)	/* FourCC */
#define	LD_WORD(ptr)		(WORD)(*(WORD*)(BYTE*)(ptr))
#define	LD_DWORD(ptr)		(DWORD)(*(DWORD*)(BYTE*)(ptr))

BYTE buffer[2048];

volatile UINT fifo_ct;
volatile UINT fifo_ri;

volatile uint16_t freq;
volatile uint8_t channel;
volatile uint8_t resolution;

void timer_proc(void);

void startplay(void)
{
	DACA.CTRLB   = DAC_CHSEL_SINGLE_gc;
	DACA.CTRLC   = DAC_REFSEL_AVCC_gc | DAC_LEFTADJ_bm;
	DACA.CTRLA   = DAC_CH0EN_bm | DAC_ENABLE_bm;
	DACA.CH0DATA = 0x8000;

	PORTA.OUTCLR = PIN3_bm;

	TCD5.CTRLB    = TC45_BYTEM_NORMAL_gc | TC45_CIRCEN_DISABLE_gc | TC45_WGMODE_NORMAL_gc;
	TCD5.CNT      = 0;
	TCD5.PER      = freq - 1;
	TCD5.INTCTRLA = TC45_OVFINTLVL_HI_gc;
	TCD5.INTFLAGS = TC5_OVFIF_bm;
	TCD5.CTRLA    = TC45_CLKSEL_DIV1_gc;
}

void stopplay(void)
{
	PORTA.OUTSET = PIN3_bm;

	TCD5.INTCTRLA = 0;
	TCD5.CTRLA = TC45_CLKSEL_OFF_gc;

	DACA.CTRLA = 0;
	DACA.CTRLB = 0;
	DACA.CTRLC = 0;
}

static
DWORD loadheader (FIL* fp)	/* 0:Invalid format, 1:I/O error, >=1024:Number of samples */
{
	DWORD sz, f;
	BYTE b;
	UINT rb;

	if (f_read(fp, buffer, 12, &rb) != FR_OK) return 1;	/* Load file header (12 bytes) */

	if (rb != 12 || LD_DWORD(&buffer[8]) != FCC('W','A','V','E')) return 0;

	for (;;) {
		f_read(fp, buffer, 8, &rb);					/* Get Chunk ID and size */
		if (rb != 8) return 0;
		sz = LD_DWORD(&buffer[4]);					/* Chunk size */

		switch (LD_DWORD(&buffer[0])) {				/* Switch by chunk ID */
		case FCC('f','m','t',' ') :					/* 'fmt ' chunk */
			if (sz & 1) sz++;						/* Align chunk size */
			if (sz > 100 || sz < 16) return 0;		/* Check chunk size */
			f_read(fp, buffer, sz, &rb);			/* Get content */
			if (rb != sz) return 0;
			if (buffer[0] != 1) return 0;			/* Check coding type (LPCM) */
			b = buffer[2];
			if (b != 1 && b != 2) return 0;			/* Mono Only */
			channel = b;
			b = buffer[14];
			if (b != 8 && b != 16) return 0;		/* resolution 8bit only */
			resolution = b;

			f = LD_DWORD(&buffer[4]);				/* Check sampling freqency (8k-48k) */
			freq = F_CPU / f;
			break;

		case FCC('d','a','t','a') :				/* 'data' chunk */
			if (sz < 1024) return 0;				/* Check size */
			return sz;								/* Start to play */

		case FCC('D','I','S','P') :				/* 'DISP' chunk */
		case FCC('L','I','S','T') :				/* 'LIST' chunk */
		case FCC('f','a','c','t') :				/* 'fact' chunk */
			if (sz & 1) sz++;						/* Align chunk size */
			f_lseek(fp, fp->fptr + sz);				/* Skip this chunk */
			break;

		default :								/* Unknown chunk */
			return 0;
		}
	}

	return 0;
}

void playfile(FIL* fp)
{
	DWORD size;
	UINT rsize;
	UINT rb;
	UINT wi;

	size = loadheader(fp);

	wi = 0;

	UINT pad = 512 - (fp->fptr % 512);
	if (pad == 0) pad = 512;
	fifo_ri = 2048 - pad;

	f_read(fp, &buffer[fifo_ri], pad, &rb);
	if (pad != rb) return;
	fifo_ct = rb;
	size -= rb;

	rsize = 1024;

	f_read(fp, &buffer[0], rsize, &rb);
	if (rb != rsize) return;

	size -= rb;
	wi += rb;
	fifo_ct += rb;

	startplay();

	set_sleep_mode(SLEEP_MODE_IDLE);
	while (size || fifo_ct >= 4) {
		timer_proc();
		sleep_mode();

		if (size && fifo_ct <= 1024) {
			rsize = (size >= 1024) ? 1024 : size;
			f_read(fp, &buffer[wi], rsize, &rb);
			if (rb != rsize) break;
			size -= rb;
			wi = (wi + rb) & (2048 - 1);
			cli();
			fifo_ct += rb;
			sei();
		}
	}

	stopplay();
	f_close(fp);

	return;
}
