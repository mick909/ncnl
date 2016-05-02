#include <avr/io.h>
#include "usart.h"
#include "ff.h"
#include "diskio.h"

#define XRESET	(_BV(PD2))	/* O: Reset */
#define XCS		(_BV(PD3))	/* O: Chip Select for SPI */
#define XDCS	(_BV(PD4))	/* O: XDCS */
#define DREQ	(_BV(PD5))	/* I: Data Request */

static BYTE buf[256];

void delay_ms(uint16_t ms);

static
BYTE xchg_spi( BYTE dat )
{
	SPDR = dat;
	loop_until_bit_is_set(SPSR, SPIF);
	return SPDR;
}


void prepare_decoder()
{
	// SCK  : PB5 -- Out, lo
	// MISO : PB4 -- In , hi-z
	// MOSI : PB3 -- Out, hi
	// SS   : PB2 -- Out, hi
	PORTB = (PORTB & 0b11011111) | 0b00001100;	/* Configure SCK/MOSI/CS as output */
	DDRB  = (DDRB  & 0b11101111) | 0b00101100;

	PORTD = (PORTD & 0b11011011) | 0b00011000;
	DDRD  = (DDRB  & 0b11011111) | 0b00011100;

	SPCR = 0x50;			/* Enable SPI function in mode 0, Fosc/4 */
	SPSR = 0x00;			/* SPI 1x mode */

	usart_write_string("prepare vs1011e, now wakeup...");
	/* Reset */
	/* XRESET<-H, and wait 2ms, and wait until DREQ->H */
	delay_ms(10);
	PORTD |= XRESET;
	delay_ms(3);
	do {} while (! (PIND & DREQ) );

	usart_write_string("wakeup done, set SDI New mode...");
	/* SDI New Mode */
	PORTD &= ~XCS;		/* Chip Select */
	xchg_spi(0x02);		/* Write */
	xchg_spi(0x00);		/* Address = MODE */
	xchg_spi(0x08);		/* SM_SDINEW = 1 */
	xchg_spi(0x20);		/* SM_TESTS  = 1 */
	PORTD |= XCS;
	delay_ms(1);
	do {} while (! (PIND & DREQ) );

	usart_write_string("set clock doubler...");
	/* Clock doubler */
	PORTD &= ~XCS;		/* Chip Select */
	xchg_spi(0x02);		/* Write */
	xchg_spi(0x03);		/* Address = CLOCKF */
	xchg_spi(0x98);		/* clock source : 12.288MHz */
	xchg_spi(0x00);
	PORTD |= XCS;
	delay_ms(1);
	do {} while (! (PIND & DREQ) );

	usart_write_string("set volume...");
	/* Volume */
	PORTD &= ~XCS;		/* Chip Select */
	xchg_spi(0x02);		/* Write */
	xchg_spi(0x0B);		/* Address = VOL */
	xchg_spi(0x20);		/* L ch */
	xchg_spi(0xFF);		/* R ch */
	PORTD |= XCS;
	delay_ms(1);
	do {} while (! (PIND & DREQ) );

	usart_write_string("prepare done\r\n");

#if 0
	BYTE dh, dl;

	PORTD &= ~XCS;
	xchg_spi(0x03);
	xchg_spi(0x01);
	dh = xchg_spi(0xff);
	dl = xchg_spi(0xff);
	PORTD |= XCS;
	delay_ms(1);
	do {} while (! (PIND & DREQ) );

	usart_write( "0123456789ABCDEF"[ (dh >> 4) ] );
	usart_write( "0123456789ABCDEF"[ (dh & 0x0f) ] );
	usart_write( ' ' );
	usart_write( "0123456789ABCDEF"[ (dl >> 4) ] );
	usart_write( "0123456789ABCDEF"[ (dl & 0x0f) ] );
	usart_write_cr();

	PORTD &= ~XCS;
	xchg_spi(0x03);
	xchg_spi(0x00);
	dh = xchg_spi(0xff);
	dl = xchg_spi(0xff);
	PORTD |= XCS;
	delay_ms(1);
	do {} while (! (PIND & DREQ) );

	usart_write( "0123456789ABCDEF"[ (dh >> 4) ] );
	usart_write( "0123456789ABCDEF"[ (dh & 0x0f) ] );
	usart_write( ' ' );
	usart_write( "0123456789ABCDEF"[ (dl >> 4) ] );
	usart_write( "0123456789ABCDEF"[ (dl & 0x0f) ] );
	usart_write_cr();

	PORTD &= ~XCS;
	xchg_spi(0x03);
	xchg_spi(0x03);
	dh = xchg_spi(0xff);
	dl = xchg_spi(0xff);
	PORTD |= XCS;
	delay_ms(1);
	do {} while (! (PIND & DREQ) );

	usart_write( "0123456789ABCDEF"[ (dh >> 4) ] );
	usart_write( "0123456789ABCDEF"[ (dh & 0x0f) ] );
	usart_write( ' ' );
	usart_write( "0123456789ABCDEF"[ (dl >> 4) ] );
	usart_write( "0123456789ABCDEF"[ (dl & 0x0f) ] );
	usart_write_cr();

	PORTD &= ~XCS;
	xchg_spi(0x03);
	xchg_spi(0x0B);
	dh = xchg_spi(0xff);
	dl = xchg_spi(0xff);
	PORTD |= XCS;
	delay_ms(1);
	do {} while (! (PIND & DREQ) );

	usart_write( "0123456789ABCDEF"[ (dh >> 4) ] );
	usart_write( "0123456789ABCDEF"[ (dh & 0x0f) ] );
	usart_write( ' ' );
	usart_write( "0123456789ABCDEF"[ (dl >> 4) ] );
	usart_write( "0123456789ABCDEF"[ (dl & 0x0f) ] );
	usart_write_cr();
#endif
}

void test()
{
	prepare_decoder();

	do {} while (! (PIND & DREQ) );

	PORTD &= ~XDCS;

	xchg_spi(0x53);
	xchg_spi(0xef);
	xchg_spi(0x6e);
	xchg_spi(0x7d);
	xchg_spi(0x00);
	xchg_spi(0x00);
	xchg_spi(0x00);
	xchg_spi(0x00);

	PORTD |= XDCS;
}

FRESULT play(FIL *fp)
{
	UINT bytesread;
	FRESULT res;
	prepare_decoder();

	do {
		WORD idx;
		res = f_read(fp, buf, 256, &bytesread);

		if ( res ) {
			usart_write_string("f_read failed :");
			usart_write( (res / 10) + '0' );
			usart_write( (res % 10) + '0' );
			usart_write_cr();
			return res;
		}

		if ( bytesread == 0 ) break;

//		usart_write_string("set 256 bytes...");
		PORTD &= ~XDCS;
		idx = 0;
		do {
			do {} while (! (PIND & DREQ));
			xchg_spi(buf[idx++]);
		} while (--bytesread);
		PORTD |= XDCS;
//		usart_write_string("done\r\n");
	} while (1);

	usart_write_string("play done, reset vs1011e\r\n");
	PORTD &= ~XRESET;
	return FR_OK;
}