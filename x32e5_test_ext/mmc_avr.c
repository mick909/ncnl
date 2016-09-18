/*-----------------------------------------------------------------------*/
/* MMCv3/SDv1/SDv2 (in SPI mode) control module                          */
/*-----------------------------------------------------------------------*/
/*
/  Copyright (C) 2014, ChaN, all right reserved.
/
/ * This software is a free software and there is NO WARRANTY.
/ * No restriction on use. You can use, modify and redistribute it for
/   personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
/ * Redistributions of source code must retain the above copyright notice.
/
/-------------------------------------------------------------------------*/

#include <avr/io.h>
#include "diskio.h"


/* Port controls  (Platform dependent) */
#define CS_LOW()	PORTD.OUTCLR = PIN4_bm;		/* CS=low */
#define	CS_HIGH()	PORTD.OUTSET = PIN4_bm;		/* CS=high */

#define FCLK_SLOW()	USARTD0.BAUDCTRLA = 63;
#define FCLK_HIGH() USARTD0.BAUDCTRLA = 0;

/*--------------------------------------------------------------------------

   Module Private Functions

---------------------------------------------------------------------------*/

/* Definitions for MMC/SDC command */
#define CMD0	(0)			/* GO_IDLE_STATE */
#define CMD1	(1)			/* SEND_OP_COND (MMC) */
#define	ACMD41	(0x80+41)	/* SEND_OP_COND (SDC) */
#define CMD8	(8)			/* SEND_IF_COND */
#define CMD9	(9)			/* SEND_CSD */
#define CMD10	(10)		/* SEND_CID */
#define CMD12	(12)		/* STOP_TRANSMISSION */
#define ACMD13	(0x80+13)	/* SD_STATUS (SDC) */
#define CMD16	(16)		/* SET_BLOCKLEN */
#define CMD17	(17)		/* READ_SINGLE_BLOCK */
#define CMD18	(18)		/* READ_MULTIPLE_BLOCK */
#define CMD23	(23)		/* SET_BLOCK_COUNT (MMC) */
#define	ACMD23	(0x80+23)	/* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24	(24)		/* WRITE_BLOCK */
#define CMD25	(25)		/* WRITE_MULTIPLE_BLOCK */
#define CMD32	(32)		/* ERASE_ER_BLK_START */
#define CMD33	(33)		/* ERASE_ER_BLK_END */
#define CMD38	(38)		/* ERASE */
#define CMD55	(55)		/* APP_CMD */
#define CMD58	(58)		/* READ_OCR */


static volatile
DSTATUS Stat = STA_NOINIT;	/* Disk status */

volatile
BYTE Timer1, Timer2;	/* 100Hz decrement timer */

static
BYTE CardType;			/* Card type flags */

/*-----------------------------------------------------------------------*/
/* Power Control  (Platform dependent)                                   */
/*-----------------------------------------------------------------------*/
/* When the target system does not support socket power control, there   */
/* is nothing to do in these functions and chk_power always returns 1.   */

static
void power_on (void)
{
	/* SS   = PD4 = Out,hi   */
	/* SCK  = PD5 = Out,lo   */
	/* MISO = PD6 = In, hi-z */
	/* MOSI = PD7 = Out,hi   */

	PORTD.DIRSET   = PIN4_bm;
	PORTD.OUTSET   = PIN4_bm;

	PORTD.DIRSET = PIN5_bm | PIN7_bm;
	PORTD.OUTCLR = PIN5_bm;
	PORTD.OUTSET = PIN7_bm;

	PORTD.DIRCLR = PIN6_bm;

	PORTD.REMAP = PORT_USART0_bm;

	/* Setup USART D0 as MSPI mode */
	USARTD0.CTRLC = USART_CMODE_MSPI_gc;	/* SPI Mode 0, MSB first */
	USARTD0.CTRLB = USART_TXEN_bm | USART_RXEN_bm;
	USARTD0.BAUDCTRLA = 63;

	/* Setup DMA for USART SPI */
	EDMA.CTRL = EDMA_ENABLE_bm | EDMA_CHMODE_STD02_gc
			   | EDMA_DBUFMODE_DISABLE_gc | EDMA_PRIMODE_CH0123_gc;
}

static
void power_off (void)
{
	USARTD0.BAUDCTRLA = 0;
	USARTD0.CTRLB     = 0;

	PORTD.OUTSET = PIN4_bm;

	EDMA.CTRL = 0;

	Stat |= STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Transmit/Receive data from/to MMC via SPI  (Platform dependent)       */
/*-----------------------------------------------------------------------*/

/* Exchange a byte */
static
BYTE xchg_spi (		/* Returns received data */
	BYTE dat		/* Data to be sent */
)
{
	while ( !(USARTD0.STATUS & USART_DREIF_bm));
	USARTD0.DATA = dat;
	while ( !(USARTD0.STATUS & USART_RXCIF_bm));
	return USARTD0.DATA;
}

#if 0
/* Send a data block fast */
static
void xmit_spi_multi (
	const BYTE *p,	/* Data block to be sent */
	UINT cnt		/* Size of data block (must be multiple of 2) */
)
{
	do {
		while ( !(USARTD0.STATUS & USART_DREIF_bm));
		USARTD0.DATA = *p++;
		while ( !(USARTD0.STATUS & USART_RXCIF_bm));

		while ( !(USARTD0.STATUS & USART_DREIF_bm));
		USARTD0.DATA = *p++;
		while ( !(USARTD0.STATUS & USART_RXCIF_bm));
	} while (cnt -= 2);
}
#endif

/* Receive a data block fast */
static
void rcvr_spi_multi (
	BYTE *p,	/* Data buffer */
	UINT cnt	/* Size of data block */
)
{
	volatile uint8_t dmy = 0xff;

	/* DMA Chennel0 -> Transfer from USARTD0.Data to buffer */
	EDMA.CH0.ADDRCTRL     = EDMA_CH_RELOAD_NONE_gc | EDMA_CH_DIR_FIXED_gc;
	EDMA.CH0.DESTADDRCTRL = EDMA_CH_RELOAD_NONE_gc | EDMA_CH_DIR_INC_gc;
	EDMA.CH0.TRIGSRC  = EDMA_CH_TRIGSRC_USARTD0_RXC_gc;

	EDMA.CH0.ADDRL =  (uint16_t)(&USARTD0.DATA)       & 0xff;
	EDMA.CH0.ADDRH = ((uint16_t)(&USARTD0.DATA) >> 8) & 0xff;

	EDMA.CH0.DESTADDRL =  (uint16_t)p       & 0xff;
	EDMA.CH0.DESTADDRH = ((uint16_t)p >> 8) & 0xff;

	EDMA.CH0.TRFCNTL =  cnt       & 0xff;
	EDMA.CH0.TRFCNTH = (cnt >> 8) & 0xff;

	/* DMA Chennel2 -> Transfer dummy 0xff to USARTD0.Data */
	EDMA.CH2.ADDRCTRL     = EDMA_CH_RELOAD_NONE_gc | EDMA_CH_DIR_FIXED_gc;
	EDMA.CH2.DESTADDRCTRL = EDMA_CH_RELOAD_NONE_gc | EDMA_CH_DIR_FIXED_gc;
	EDMA.CH2.TRIGSRC  = EDMA_CH_TRIGSRC_USARTD0_DRE_gc;

	EDMA.CH2.ADDRL =  (uint16_t)(&dmy)       & 0xff;
	EDMA.CH2.ADDRH = ((uint16_t)(&dmy) >> 8) & 0xff;

	EDMA.CH2.DESTADDRL =  (uint16_t)(&USARTD0.DATA)       & 0xff;
	EDMA.CH2.DESTADDRH = ((uint16_t)(&USARTD0.DATA) >> 8) & 0xff;

	EDMA.CH2.TRFCNTL =  cnt       & 0xff;
	EDMA.CH2.TRFCNTH = (cnt >> 8) & 0xff;

	EDMA.CH2.CTRLA = EDMA_CH_ENABLE_bm | EDMA_CH_SINGLE_bm;
	EDMA.CH0.CTRLA = EDMA_CH_ENABLE_bm | EDMA_CH_SINGLE_bm;

	while ( !(EDMA.INTFLAGS & EDMA_CH0TRNFIF_bm) ) {}

	EDMA.INTFLAGS |= EDMA_CH0TRNFIF_bm | EDMA_CH2TRNFIF_bm;
}



/*-----------------------------------------------------------------------*/
/* Wait for card ready                                                   */
/*-----------------------------------------------------------------------*/

static
int wait_ready (	/* 1:Ready, 0:Timeout */
	UINT wt			/* Timeout [ms] */
)
{
	BYTE d;

	Timer2 = wt / 10;
	do
		d = xchg_spi(0xFF);
	while (d != 0xFF && Timer2);

	return (d == 0xFF) ? 1 : 0;
}



/*-----------------------------------------------------------------------*/
/* Deselect the card and release SPI bus                                 */
/*-----------------------------------------------------------------------*/

static
void deselect (void)
{
	CS_HIGH();		/* Set CS# high */
	xchg_spi(0xFF);	/* Dummy clock (force DO hi-z for multiple slave SPI) */
}



/*-----------------------------------------------------------------------*/
/* Select the card and wait for ready                                    */
/*-----------------------------------------------------------------------*/

static
int select (void)	/* 1:Successful, 0:Timeout */
{
	CS_LOW();		/* Set CS# low */
	xchg_spi(0xFF);	/* Dummy clock (force DO enabled) */
	if (wait_ready(500)) return 1;	/* Wait for card ready */

	deselect();
	return 0;	/* Timeout */
}



/*-----------------------------------------------------------------------*/
/* Receive a data packet from MMC                                        */
/*-----------------------------------------------------------------------*/

static
int rcvr_datablock (
	BYTE *buff,			/* Data buffer to store received data */
	UINT btr			/* Byte count (must be multiple of 4) */
)
{
	BYTE token;

	Timer1 = 20;
	do {							/* Wait for data packet in timeout of 200ms */
		token = xchg_spi(0xFF);
	} while ((token == 0xFF) && Timer1);
	if (token != 0xFE) return 0;	/* If not valid data token, retutn with error */

	rcvr_spi_multi(buff, btr);		/* Receive the data block into buffer */
	xchg_spi(0xFF);					/* Discard CRC */
	xchg_spi(0xFF);

	return 1;						/* Return with success */
}


/*-----------------------------------------------------------------------*/
/* Send a command packet to MMC                                          */
/*-----------------------------------------------------------------------*/

static
BYTE send_cmd (		/* Returns R1 resp (bit7==1:Send failed) */
	BYTE cmd,		/* Command index */
	DWORD arg		/* Argument */
)
{
	BYTE n, res;


	if (cmd & 0x80) {	/* ACMD<n> is the command sequense of CMD55-CMD<n> */
		cmd &= 0x7F;
		res = send_cmd(CMD55, 0);
		if (res > 1) return res;
	}

	/* Select the card and wait for ready except to stop multiple block read */
	if (cmd != CMD12) {
		deselect();
		if (!select()) return 0xFF;
	}

	/* Send command packet */
	xchg_spi(0x40 | cmd);				/* Start + Command index */
	xchg_spi((BYTE)(arg >> 24));		/* Argument[31..24] */
	xchg_spi((BYTE)(arg >> 16));		/* Argument[23..16] */
	xchg_spi((BYTE)(arg >> 8));			/* Argument[15..8] */
	xchg_spi((BYTE)arg);				/* Argument[7..0] */
	n = 0x01;							/* Dummy CRC + Stop */
	if (cmd == CMD0) n = 0x95;			/* Valid CRC for CMD0(0) + Stop */
	if (cmd == CMD8) n = 0x87;			/* Valid CRC for CMD8(0x1AA) Stop */
	xchg_spi(n);

	/* Receive command response */
	if (cmd == CMD12) xchg_spi(0xFF);		/* Skip a stuff byte when stop reading */
	n = 10;								/* Wait for a valid response in timeout of 10 attempts */
	do
		res = xchg_spi(0xFF);
	while ((res & 0x80) && --n);

	return res;			/* Return with the response value */
}



/*--------------------------------------------------------------------------

   Public Functions

---------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
/* Initialize Disk Drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv		/* Physical drive nmuber (0) */
)
{
	BYTE n, cmd, ty, ocr[4];

	if (pdrv) return STA_NOINIT;		/* Supports only single drive */
	power_off();						/* Turn off the socket power to reset the card */
	if (Stat & STA_NODISK) return Stat;	/* No card in the socket */
	power_on();							/* Turn on the socket power */
	FCLK_SLOW();

	for (n = 10; n; n--) xchg_spi(0xFF);	/* 80 dummy clocks */

	ty = 0;
	if (send_cmd(CMD0, 0) == 1) {			/* Enter Idle state */
		Timer1 = 100;						/* Initialization timeout of 1000 msec */
		if (send_cmd(CMD8, 0x1AA) == 1) {	/* SDv2? */
			for (n = 0; n < 4; n++) ocr[n] = xchg_spi(0xFF);		/* Get trailing return value of R7 resp */
			if (ocr[2] == 0x01 && ocr[3] == 0xAA) {				/* The card can work at vdd range of 2.7-3.6V */
				while (Timer1 && send_cmd(ACMD41, 1UL << 30));	/* Wait for leaving idle state (ACMD41 with HCS bit) */
				if (Timer1 && send_cmd(CMD58, 0) == 0) {		/* Check CCS bit in the OCR */
					for (n = 0; n < 4; n++) ocr[n] = xchg_spi(0xFF);
					ty = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;	/* SDv2 */
				}
			}
		} else {							/* SDv1 or MMCv3 */
			if (send_cmd(ACMD41, 0) <= 1) 	{
				ty = CT_SD1; cmd = ACMD41;	/* SDv1 */
			} else {
				ty = CT_MMC; cmd = CMD1;	/* MMCv3 */
			}
			while (Timer1 && send_cmd(cmd, 0));			/* Wait for leaving idle state */
			if (!Timer1 || send_cmd(CMD16, 512) != 0)	/* Set R/W block length to 512 */
				ty = 0;
		}
	}
	CardType = ty;
	deselect();

	if (ty) {			/* Initialization succeded */
		Stat &= ~STA_NOINIT;		/* Clear STA_NOINIT */

		FCLK_HIGH();
	} else {			/* Initialization failed */
		power_off();
	}

	return Stat;
}



/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber (0) */
)
{
	if (pdrv) return STA_NOINIT;	/* Supports only single drive */
	return Stat;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,			/* Physical drive nmuber (0) */
	BYTE *buff,			/* Pointer to the data buffer to store read data */
	DWORD sector,		/* Start sector number (LBA) */
	UINT count			/* Sector count (1..128) */
)
{
	BYTE cmd;


	if (pdrv || !count) return RES_PARERR;
	if (Stat & STA_NOINIT) return RES_NOTRDY;

	if (!(CardType & CT_BLOCK)) sector *= 512;	/* Convert to byte address if needed */

	cmd = count > 1 ? CMD18 : CMD17;			/*  READ_MULTIPLE_BLOCK : READ_SINGLE_BLOCK */
	if (send_cmd(cmd, sector) == 0) {
		do {
			if (!rcvr_datablock(buff, 512)) break;
			buff += 512;
		} while (--count);
		if (cmd == CMD18) send_cmd(CMD12, 0);	/* STOP_TRANSMISSION */
	}
	deselect();

	return count ? RES_ERROR : RES_OK;
}
