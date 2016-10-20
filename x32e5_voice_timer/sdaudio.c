#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include "sdaudio.h"
#include "ff.h"

#pragma GCC diagnostic ignored "-Wstrict-aliasing"

#define FCC(c1,c2,c3,c4)	(((DWORD)c4<<24)+((DWORD)c3<<16)+((WORD)c2<<8)+(BYTE)c1)	/* FourCC */
#define	LD_WORD(ptr)		(WORD)(*(WORD*)(BYTE*)(ptr))
#define	LD_DWORD(ptr)		(DWORD)(*(DWORD*)(BYTE*)(ptr))

BYTE buffer[2][1024];
volatile uint16_t buffer_ct[2];
volatile uint8_t play_buffer;

volatile uint16_t freq;
volatile uint8_t channel;
volatile uint8_t resolution;

void timer_proc(void);

void startplayisr(void)
{
	DACA.CTRLB = DAC_CHSEL_SINGLE_gc;
	DACA.CTRLC = DAC_REFSEL_AVCC_gc | DAC_LEFTADJ_bm;
	DACA.CTRLA = DAC_CH0EN_bm | DAC_ENABLE_bm;
	DACA.CH0DATA = 128;

	PORTA.OUTCLR = PIN3_bm;

	TCD5.CTRLB = TC45_BYTEM_NORMAL_gc | TC45_CIRCEN_DISABLE_gc | TC45_WGMODE_NORMAL_gc;
	TCD5.CNT = 0;
	TCD5.PER = freq - 1;
	TCD5.INTCTRLA = TC45_OVFINTLVL_HI_gc;
	TCD5.INTFLAGS = TC5_OVFIF_bm;
	TCD5.CTRLA = TC45_CLKSEL_DIV1_gc;
}

void stopplayisr(void)
{
	PORTA.OUTSET = PIN3_bm;

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

	if (f_read(fp, buffer[0], 12, &rb) != FR_OK) return 1;	/* Load file header (12 bytes) */

	if (rb != 12 || LD_DWORD(&buffer[0][8]) != FCC('W','A','V','E')) return 0;

	for (;;) {
		f_read(fp, buffer[0], 8, &rb);				/* Get Chunk ID and size */
		if (rb != 8) return 0;
		sz = LD_DWORD(&buffer[0][4]);					/* Chunk size */

		switch (LD_DWORD(&buffer[0][0])) {			/* Switch by chunk ID */
		case FCC('f','m','t',' ') :					/* 'fmt ' chunk */
			if (sz & 1) sz++;						/* Align chunk size */
			if (sz > 100 || sz < 16) return 0;		/* Check chunk size */
			f_read(fp, buffer[0], sz, &rb);			/* Get content */
			if (rb != sz) return 0;
			if (buffer[0][0] != 1) return 0;			/* Check coding type (LPCM) */
			b = buffer[0][2];
			if (b != 1 && b != 2) return 0;			/* Mono Only */
			channel = b;
			b = buffer[0][14];
			if (b != 8 && b != 16) return 0;		/* resolution 8bit only */
			resolution = b;
			f = LD_DWORD(&buffer[0][4]);				/* Check sampling freqency (8k-48k) */
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

uint16_t make_dac_data(uint8_t* buff, uint16_t size)
{
	uint8_t unit = 1;
	uint16_t ct = 0;

	if (resolution == 16) unit = 2;
	if (channel == 2) unit *= 2;

	while (size > unit) {
		if (channel == 1) {
			if (resolution == 8) {
				uint16_t d = buff[size-1];
				d <<= 4;
				buff[size*2-2] = d;
				--size;
			}
			else {
				uint16_t d = *(uint16_t*)buff + 0x8000;
				d >>= 4;
				*(uint16_t*)buff = d;
				buff += 2;
				size -= 2;
			}
		}
		else {
			if (resolution == 8) {
				uint16_t d = *buff;
				d += *(buff+1);
				d <<= 3;
				*(uint16_t*)buff = d;
				buff += 2;
				size -= 2;
			}
			else {
				uint16_t d = (*(uint16_t*)buff + 0x8000) >> 1;
				d += (*(uint16_t*)(buff+2) + 0x8000) >> 1;
				*(uint16_t*)(buff+ct) = d >> 3;
				buff += unit;
				size -= unit;
			}
		}
		ct += 2;
	}

	return ct;
}

void playfile(FIL* fp)
{
	DWORD size;
	UINT rsize;
	UINT rb;

	size = loadheader(fp);

	UINT pad = 512 - (fp->fptr % 512);
	if (pad == 0) pad = 512;
	f_read(fp, &buffer[0][0], pad, &rb);
	if (pad != rb) return;
	buffer_ct[0] = make_dac_data(buffer[0], rb);
	size -= rb;

	if (resolution == 8 && channel == 1) rsize = 512;
	else rsize = 1024;

	f_read(fp, &buffer[1][0], rsize, &rb);
	if (rb != rsize) return;
	buffer_ct[1] = make_dac_data(buffer[1], rb);
	size -= rb;

	play_buffer = 0;

	startplayisr();

	set_sleep_mode(SLEEP_MODE_IDLE);
	while (size || buffer_ct[play_buffer] >= 4) {
		sleep_mode();
		timer_proc();

		if (size && buffer_ct[play_buffer] == 0) {
			cli();
			play_buffer = 1 - play_buffer;
			sei();

			rsize = (size >= rsize) ? rsize : size;
			f_read(fp, &buffer[1-play_buffer][0], rsize, &rb);
			if (rb != rsize) break;
			buffer_ct[1-play_buffer] = make_dac_data(buffer[1-play_buffer], rb);
			size -= rb;
		}
	}

	stopplayisr();
	f_close(fp);

	return;
}
