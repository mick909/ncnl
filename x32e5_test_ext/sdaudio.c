#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include "sdaudio.h"
#include "ff.h"

#define PLL_SCALE (1)

#pragma GCC diagnostic ignored "-Wstrict-aliasing"

#define FCC(c1,c2,c3,c4)	(((DWORD)c4<<24)+((DWORD)c3<<16)+((WORD)c2<<8)+(BYTE)c1)	/* FourCC */
#define	LD_WORD(ptr)		(WORD)(*(WORD*)(BYTE*)(ptr))
#define	LD_DWORD(ptr)		(DWORD)(*(DWORD*)(BYTE*)(ptr))

#define BUFFER_SIZE (2048)

BYTE buffer[BUFFER_SIZE];

volatile UINT fifo_ct;
volatile UINT fifo_ri;

volatile uint16_t freq;
volatile uint8_t channel;
volatile uint8_t resolution;

void startplayisr(void)
{
	DACA.CTRLB = DAC_CHSEL_SINGLE_gc;
	DACA.CTRLC = DAC_REFSEL_AVCC_gc | DAC_LEFTADJ_bm;
	DACA.CTRLA = DAC_CH0EN_bm | DAC_ENABLE_bm;
	DACA.CH0DATA = 128;

	TCD5.CTRLB = TC45_BYTEM_NORMAL_gc | TC45_CIRCEN_DISABLE_gc | TC45_WGMODE_NORMAL_gc;
	TCD5.CNT = 0;
	TCD5.PER = freq * PLL_SCALE - 1;
	TCD5.INTCTRLA = TC45_OVFINTLVL_HI_gc;
	TCD5.INTFLAGS = TC5_OVFIF_bm;
	TCD5.CTRLA = TC45_CLKSEL_DIV1_gc;
}

void stopplayisr(void)
{
	DACA.CTRLA = 0;
	DACA.CTRLB = 0;
	DACA.CTRLC = 0;

	TCD5.INTCTRLA = 0;
	TCD5.CTRLA = TC45_CLKSEL_OFF_gc;
}

static
DWORD loadheader (FIL* fp)	/* 0:Invalid format, 1:I/O error, >=1024:Number of samples */
{
	DWORD sz, f;
	BYTE b;
	UINT rb;

	if (f_read(fp, buffer, 12, &rb) != FR_OK) return 1;	/* Load file header (12 bytes) */

	if (rb != 12 || LD_DWORD(buffer+8) != FCC('W','A','V','E')) return 0;

	for (;;) {
		f_read(fp, buffer, 8, &rb);				/* Get Chunk ID and size */
		if (rb != 8) return 0;
		sz = LD_DWORD(&buffer[4]);					/* Chunk size */

		switch (LD_DWORD(&buffer[0])) {			/* Switch by chunk ID */
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

uint8_t playfile(FIL* fp)
{
	DWORD size;
	UINT rsize;
	UINT rb;
	UINT wi;

	size = loadheader(fp);

	wi = 0;

	UINT pad = 512 - (fp->fptr % 512);
	if (pad) {
		fifo_ri = BUFFER_SIZE - pad;

		f_read(fp, &buffer[fifo_ri], pad, &rb);
		if (pad != rb) return 1;
		fifo_ct = rb;
		size -= rb;
	} else {
		fifo_ct = 0;
		fifo_ri = 0;
	}

	rsize = BUFFER_SIZE / 2;
	f_read(fp, &buffer[0], rsize, &rb);
	if (rb != rsize) return 2;

	size -= rb;
	wi += rb;
	fifo_ct += rb;

	startplayisr();

	set_sleep_mode(SLEEP_MODE_IDLE);
	while (size || fifo_ct >= 4) {
		sleep_mode();

		if (size && fifo_ct <= (BUFFER_SIZE/2)) {
			rsize = (size >= (BUFFER_SIZE/2)) ? (BUFFER_SIZE/2) : size;
			f_read(fp, &buffer[wi], rsize, &rb);
			if (rb != rsize) break;
			size -= rb;
			wi = (wi + rb) & (BUFFER_SIZE-1);
			cli();
			fifo_ct += rb;
			sei();
		}
	}

	stopplayisr();
	f_close(fp);

	return 9;
}
