#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include "sdaudio.h"
#include "ff.h"

#pragma GCC diagnostic ignored "-Wstrict-aliasing"

void set_led_digit(uint16_t v);

#define FCC(c1,c2,c3,c4)	(((DWORD)c4<<24)+((DWORD)c3<<16)+((WORD)c2<<8)+(BYTE)c1)	/* FourCC */
#define	LD_WORD(ptr)		(WORD)(*(WORD*)(BYTE*)(ptr))
#define	LD_DWORD(ptr)		(DWORD)(*(DWORD*)(BYTE*)(ptr))

volatile uint16_t trace_num;

BYTE buffer[2][1024];
volatile uint16_t buffer_ct[2];
volatile uint8_t play_buffer;
volatile uint16_t* playp;
volatile uint16_t playcnt;

uint16_t (* volatile transfer)(BYTE* p, uint16_t size);

volatile uint16_t freq;
volatile uint8_t channel;
volatile uint8_t resolution;

void timer_proc(void);

void startplay(void)
{
	DACA.CTRLB = DAC_CHSEL_SINGLE_gc;
	DACA.CTRLC = DAC_REFSEL_AVCC_gc | DAC_LEFTADJ_bm;
	DACA.CTRLA = DAC_CH0EN_bm | DAC_ENABLE_bm;

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
	TCD5.INTFLAGS = TC5_OVFIF_bm;
	TCD5.CTRLA = TC45_CLKSEL_OFF_gc;

	DACA.CTRLA = 0;
	DACA.CTRLB = 0;
	DACA.CTRLC = 0;
}

uint16_t transfer_8bit_mono(BYTE* buff, uint16_t size)
{
	BYTE* dp = buff + size * 2;
	buff += size;

	while (buff != dp) {
		*--dp = *--buff;
		*--dp = 0;
		timer_proc();
	}

	return size * 2;
}

uint16_t transfer_8bit_stereo(BYTE* buff, uint16_t size)
{
	uint16_t d;
	uint16_t* dp = (uint16_t*)(buff);

	for (uint16_t cnt=size; cnt > 0; cnt -= 2) {
		d = *buff++;
		d += *buff++;
		d <<= 7;
		*dp++ = d;
		timer_proc();
	}

	return size;
}

uint16_t transfer_16bit_mono(BYTE* buff, uint16_t size)
{
	uint16_t d;
	uint16_t* dp = (uint16_t*)(buff);

	for (uint16_t cnt=size; cnt > 0; cnt -= 2) {
		d = *dp;
		d = (d + 0x8000) & 0xfff0;
		*dp++ = d;
		timer_proc();
	}

	return size;
}

uint16_t transfer_16bit_stereo(BYTE* buff, uint16_t size)
{
	uint16_t d;
	uint16_t* sp = (uint16_t*)(buff);
	uint16_t* dp = (uint16_t*)(buff);

	for (uint16_t cnt=size; cnt >= 4; cnt -= 4) {
		d = (*sp++ + 0x8000) >> 1;
		d += (*sp++ + 0x8000) >> 1;
		*dp++ = d & 0xfff0;
		timer_proc();
	}

	return size / 2;
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

			if (channel == 1) {
				if (resolution == 8) {
					transfer = transfer_8bit_mono;
				} else {
					transfer = transfer_16bit_mono;
				}
			} else {
				if (resolution == 8) {
					transfer = transfer_8bit_stereo;
				} else {
					transfer = transfer_16bit_stereo;
				}
			}

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

ISR(TCD5_OVF_vect) {
	TCD5.INTFLAGS = TC5_OVFIF_bm;

	uint16_t cnt = playcnt;

	if (cnt) {
		DACA.CH0DATA = *playp++;
		playcnt = cnt - 2;
	}
}

void playfile(FIL* fp)
{
	DWORD size;
	UINT rsize;
	UINT rb;

	size = loadheader(fp);

	trace_num = 0;

	UINT pad = 512 - (fp->fptr % 512);
	if (pad == 0) pad = 512;
	f_read(fp, &buffer[0][0], pad, &rb);
	if (pad != rb) return;
	buffer_ct[0] = (*transfer)(buffer[0], rb);
	size -= rb;

	if (resolution == 8 && channel == 1) rsize = 512;
	else rsize = 1024;

	f_read(fp, &buffer[1][0], rsize, &rb);
	if (rb != rsize) return;
	buffer_ct[1] = (*transfer)(buffer[1], rb);
	size -= rb;

	play_buffer = 0;
	playcnt = buffer_ct[0];
	playp = (uint16_t*)(buffer[0]);
	startplay();

	set_sleep_mode(SLEEP_MODE_IDLE);
	while (size || playcnt) {
		do {
			sleep_mode();
			timer_proc();
		} while ( playcnt );

		uint16_t cur_buffer = play_buffer;
		uint16_t next_buffer = 1 - play_buffer;

		cli();
		playp = (uint16_t*)(buffer[next_buffer]);
		playcnt = buffer_ct[next_buffer];
		sei();

		if (size) {
			rsize = (size >= rsize) ? rsize : size;
			f_read(fp, &buffer[cur_buffer][0], rsize, &rb);
			if (rb != rsize) break;
			buffer_ct[cur_buffer] = (*transfer)(buffer[cur_buffer], rb);
			size -= rb;
		} else {
			buffer_ct[cur_buffer] = 0;
		}
		play_buffer = next_buffer;
	}

	set_led_digit(9999);
	stopplay();
	f_close(fp);

	return;
}
