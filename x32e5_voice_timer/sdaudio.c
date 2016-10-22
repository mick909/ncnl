#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include "sdaudio.h"
#include "ff.h"

#define EDMACH	CH2

#pragma GCC diagnostic ignored "-Wstrict-aliasing"

#define FCC(c1,c2,c3,c4)	(((DWORD)c4<<24)+((DWORD)c3<<16)+((WORD)c2<<8)+(BYTE)c1)	/* FourCC */
#define	LD_WORD(ptr)		(WORD)(*(WORD*)(BYTE*)(ptr))
#define	LD_DWORD(ptr)		(DWORD)(*(DWORD*)(BYTE*)(ptr))

BYTE buffer[2][1024];
volatile uint16_t buffer_ct[2];
volatile uint8_t play_buffer;
volatile uint16_t* playp;
volatile uint16_t playcnt;

volatile uint16_t dummy;

uint16_t (* volatile transfer)(BYTE* p, uint16_t size);

volatile uint16_t freq;
volatile uint8_t channel;
volatile uint8_t resolution;

void timer_proc(void);

void startplay(void)
{
	DACA.EVCTRL = DAC_EVSEL_2_gc;
	DACA.CTRLB  = DAC_CHSEL_SINGLE_gc | DAC_CH0TRIG_bm;
	DACA.CTRLC  = DAC_REFSEL_AVCC_gc | DAC_LEFTADJ_bm;
	DACA.CTRLA  = DAC_CH0EN_bm | DAC_ENABLE_bm;

	PORTA.OUTCLR = PIN3_bm;

	EDMA.EDMACH.ADDRCTRL = EDMA_CH_RELOAD_NONE_gc  | EDMA_CH_DIR_INC_gc;
	EDMA.EDMACH.TRFCNTL  = (playcnt / 2) & 0xff;
	EDMA.EDMACH.ADDR     = ((uint16_t)playp);
	EDMA.EDMACH.TRIGSRC  = EDMA_CH_TRIGSRC_DACA_CH0_gc;

	EDMA.EDMACH.CTRLB = EDMA_CH_TRNIF_bm | 0x03;
	EDMA.EDMACH.CTRLA = EDMA_CH_ENABLE_bm | EDMA_CH_REPEAT_bm | EDMA_CH_SINGLE_bm | EDMA_CH_BURSTLEN_bm;

	TCD5.CTRLB    = TC45_BYTEM_NORMAL_gc | TC45_CIRCEN_DISABLE_gc | TC45_WGMODE_NORMAL_gc;
	TCD5.CNT      = 0;
	TCD5.PER      = freq - 1;
	TCD5.CTRLA    = TC45_CLKSEL_DIV1_gc;
	EVSYS.CH2MUX = EVSYS_CHMUX_TCD5_OVF_gc;
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

ISR(EDMA_CH2_vect)
{
	uint16_t next_buffer = 1 - play_buffer;
	uint16_t next_cnt = buffer_ct[next_buffer];
	uint16_t playp = (uint16_t)(buffer[next_buffer]);

	if (next_cnt) {
		EDMA.EDMACH.ADDRCTRL = EDMA_CH_RELOAD_NONE_gc | EDMA_CH_DIR_INC_gc;
		EDMA.EDMACH.TRFCNT   = (next_cnt / 2) & 0xff;
		EDMA.EDMACH.ADDR     =  playp;

		EDMA.EDMACH.TRIGSRC  = EDMA_CH_TRIGSRC_DACA_CH0_gc;

		EDMA.EDMACH.CTRLA = EDMA_CH_ENABLE_bm | EDMA_CH_REPEAT_bm | EDMA_CH_SINGLE_bm | EDMA_CH_BURSTLEN_bm;
		EDMA.EDMACH.CTRLB = EDMA_CH_TRNIF_bm | 0x03;
	} else {
		EDMA.EDMACH.CTRLB = EDMA_CH_TRNIF_bm;
	}
	playcnt = 0;
}

void playfile(FIL* fp)
{
	DWORD size;
	UINT rsize;
	UINT rb;

	size = loadheader(fp);

	UINT pad = 256 - (fp->fptr % 256);
	if (pad == 0) pad = 256;
	f_read(fp, &buffer[0][0], pad, &rb);
	if (pad != rb) return;
	buffer_ct[0] = (*transfer)(buffer[0], rb);
	size -= rb;

	if (resolution == 8 && channel == 1) rsize = 256;
	else rsize = 512;

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
//			timer_proc();
//			sleep_mode();
		} while (playcnt);

		uint16_t cur_buffer = play_buffer;
		uint16_t next_buffer = 1 - play_buffer;
		uint16_t next_cnt = buffer_ct[next_buffer];

		cli();
		playcnt = next_cnt;
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

	stopplay();
	f_close(fp);

	return;
}
