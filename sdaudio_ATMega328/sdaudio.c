#include <avr/io.h>
#include <avr/interrupt.h>

#include "sdaudio.h"
#include "ff.h"
#include "usart.h"

#define FCC(c1,c2,c3,c4)	(((DWORD)c4<<24)+((DWORD)c3<<16)+((WORD)c2<<8)+(BYTE)c1)	/* FourCC */

uint8_t buffer[512];

volatile uint8_t playb;
volatile uint8_t playidx;
volatile UINT playcnt;

volatile uint8_t readb;
volatile UINT readcnt;

volatile uint8_t cnt;

/* Tpwm = 250/8M = 31.25us (fpwm = 32kHz)            */
/* Tint = 1/8k = 125us                               */
/* N = 4                                             */
/* fint < fcutoff < fpwm = 16kHz                     */
/* CR LPF : R = 100 ohm, C = 0.1uF                   */
void initpwm(void)
{
	// OC2B : PD3 -- Out, lo
	PORTD &= 0b11110111;
	DDRD  |= 0b00001000;

	TCCR2A = 0;
	TCCR2B = 0;
	OCR2A  = 250-1;
	OCR2B  = 125;
	TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
	TCCR2B = _BV(WGM22) | _BV(CS20);
	TIMSK2 = 0;
}

void startplayisr(void)
{
	TIMSK2 = _BV(OCIE2A);
}

static void stopplayisr(void)
{
	TIMSK2 = 0;
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
		usart_write_string("uf");
		return 125;
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

	if (d < 3) d = 3;
	d -= 3;
	if (d > 249) d = 249;
	return d;
}

ISR(TIMER2_COMPA_vect)
{
	uint8_t c = cnt;

	if ( (++c) >= 4) {
		OCR2B = get_byte();
		c = 0;
	}
	cnt = c;
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
			if (b != 1) return 0;					/* Mono Only */
			b = buffer[14];
			if (b != 8) return 0;					/* resolution 8bit only */
			f = LD_DWORD(&buffer[4]);				/* Check sampling freqency (8k-48k) */
			if (f != 8000) return 4;				/* 8kHz only */
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
	FRESULT res;
	DWORD size;
	UINT rsize;
	UINT rb;

	size = loadheader(fp);
	if (size < 1024) return;

	usart_write_string("loadheader : OK\r\n");

	res = f_read(fp, buffer, 256 - (fp->fptr % 256) , &rb);
	size -= rb;
	playcnt = rb;
	if (res != FR_OK) return;

	res = f_read(fp, buffer+256, 256, &rb);
	size -= rb;
	readcnt = rb;
	if (res != FR_OK || rb != 256) return;

	playb = readb = 0;
	playidx = 0;

	usart_write_string("playfile : start\r\n");

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

	while (playcnt) ;

	stopplayisr();

	usart_write_string("playfile : play end\r\n");

	/* Center Level */
	OCR2B = 125;
}