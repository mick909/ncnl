#include <avr/io.h>
#include <avr/interrupt.h>

#include "sdaudio.h"
#include "ff.h"

#define FCC(c1,c2,c3,c4)	(((DWORD)c4<<24)+((DWORD)c3<<16)+((WORD)c2<<8)+(BYTE)c1)	/* FourCC */

uint8_t buffer[512];

volatile uint8_t playb;
volatile uint8_t playidx;
volatile UINT playcnt;

volatile uint16_t freq;
volatile uint8_t channel;
volatile uint8_t resolution;

volatile uint8_t readb;
volatile UINT readcnt;

volatile uint8_t cnt;

void startplayisr(void)
{
	DACA.CTRLB = DAC_CHSEL_SINGLE_gc;
	DACA.CTRLC = DAC_REFSEL_AVCC_gc | DAC_LEFTADJ_bm;
	DACA.CTRLA = DAC_CH0EN_bm | DAC_ENABLE_bm;
	DACA.CH0DATA = 128;

	TCD5.CTRLB = TC45_BYTEM_NORMAL_gc | TC45_CIRCEN_DISABLE_gc | TC45_WGMODE_NORMAL_gc;
	TCD5.CNT = 0;
	TCD5.PER = freq *2 - 1;
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

static uint8_t get_byte(void)
{
	uint8_t i, d;
	uint8_t pb;
	UINT n;

	i = playidx;
	n = playcnt;
	pb = playb;

	if (!n) {
		return 128;
	}

	d = buffer[((uint16_t)pb << 8) + i++];

	if (!--n) {
		if (pb == readb) {
			playb = 1-pb;
			i =0;
			n = readcnt;
		}
	}

	playidx = i;
	playcnt = n;

	return d;
}

static uint16_t get_word(void)
{
	uint8_t i, dh, dl;
	uint8_t pb;
	UINT n;

	i = playidx;
	n = playcnt;
	pb = playb;

	if (n < 2) {
		return 0x8000;
	}

	dl = buffer[((uint16_t)pb << 8) + i++];
	dh = buffer[((uint16_t)pb << 8) + i++] + 0x80;
	n -= 2;
	if (!n) {
		if (pb == readb) {
			playb = 1-pb;
			i =0;
			n = readcnt;
		}
	}

	playidx = i;
	playcnt = n;

	return ((uint16_t)dh<<8) + (uint16_t)dl;
}

ISR(TCD5_OVF_vect)
{
	uint8_t dh;
	uint16_t d;
	TCD5.INTFLAGS = TC5_OVFIF_bm;

	if (resolution == 8) {
		dh = get_byte();
		if (channel == 2) {
			dh = (dh >> 1) + (get_byte() >> 1);
		}
		DACA.CH0DATAL = 0;
		DACA.CH0DATAH = dh;
	} else {
		d = get_word();
		if (channel == 2) {
			d = (d >> 1) + (get_word() >> 1);
		}
		DACA.CH0DATA = d;
	}
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
	FRESULT res;
	DWORD size;
	UINT rsize;
	UINT rb;

	size = loadheader(fp);
	if (size < 1024) return 8;

	res = f_read(fp, buffer, 256 - (fp->fptr % 256) , &rb);
	size -= rb;
	playcnt = rb;
	if (res != FR_OK) return 2;

	res = f_read(fp, buffer+256, 256, &rb);
	size -= rb;
	readcnt = rb;
	if (res != FR_OK || rb != 256) return 3;

	playb = readb = 0;
	playidx = 0;

	startplayisr();

	for (;;) {
		uint8_t p, r;
		do {
			cli();
			p = playb;
			r = readb;
			sei();
		} while (p == r);

		rsize = (size > 256) ? 256 : (UINT)size;
		res  = f_read(fp, buffer + ((uint16_t)readb << 8), rsize, &rb);
		if (res != FR_OK) break;

		size -= rb;

		cli();
		readb = playb;
		readcnt = rb;
		sei();

		if (rb != 256) break;
	}

	while (playcnt) {};

	stopplayisr();

	return 9;
}
