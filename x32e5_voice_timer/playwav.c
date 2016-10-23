#include <avr/io.h>
#include <string.h>

#include "ff.h"
#include "diskio.h"
#include "sdaudio.h"

FRESULT scan_files(void)
{
	FRESULT res;
	FILINFO fno;
	FIL fp;
	DIR dir;
	int j;
	char *fn;
	char path[64];

	strcpy(path, "0:");
	res = f_opendir(&dir, path);
	if (res == FR_OK) {

		for (;;) {
			res = f_readdir(&dir, &fno);
			fn = fno.fname;

			if (res || !fn[0]) { break; }
			if (fn[0] == '.' || fn[0] == '_') { continue; }

			j = strlen(fn);
			if (j < 4) continue;
			if (fn[j-4] != '.' || fn[j-3] != 'W' || fn[j-2] != 'A' || fn[j-1] != 'V') continue;

			strcpy(&path[2], fn);

			res = f_open(&fp, path, FA_OPEN_EXISTING | FA_READ);
			if (res == FR_OK) {
				playfile(&fp);
				break;
			}
		}

	} else {
	}
	return res;
}

void play_wav(void)
{
	FATFS fs;
	volatile uint8_t dmy = 0xff;

	/* SS   = PC0 = Out,hi   */
	/* SCK  = PC1 = Out,lo   */
	/* MISO = PC2 = In, hi-z */
	/* MOSI = PC3 = Out,hi   */

	PORTC.DIRSET   = PIN0_bm;
	PORTC.OUTSET   = PIN0_bm;

	PORTC.DIRSET = PIN1_bm | PIN3_bm;
	PORTC.OUTCLR = PIN1_bm;
	PORTC.OUTSET = PIN3_bm;

	PORTC.DIRCLR = PIN2_bm;

	/* Setup USART C0 as MSPI mode */
	USARTC0.CTRLC = USART_CMODE_MSPI_gc;	/* SPI Mode 0, MSB first */
	USARTC0.CTRLB = USART_TXEN_bm | USART_RXEN_bm;
	USARTC0.BAUDCTRLA = 63;

	/* Setup DMA for USART SPI */
	EDMA.CTRL = EDMA_ENABLE_bm | EDMA_CHMODE_STD02_gc
			   | EDMA_DBUFMODE_DISABLE_gc | EDMA_PRIMODE_CH0123_gc;

	/* DMA Chennel0 -> Transfer from USARTC0.Data to buffer */
	EDMA.CH0.ADDRCTRL     = EDMA_CH_RELOAD_NONE_gc | EDMA_CH_DIR_FIXED_gc;
	EDMA.CH0.ADDR         = (uint16_t)(&USARTC0.DATA);
	EDMA.CH0.DESTADDRCTRL = EDMA_CH_RELOAD_NONE_gc | EDMA_CH_DIR_INC_gc;
	EDMA.CH0.TRIGSRC      = EDMA_CH_TRIGSRC_USARTC0_RXC_gc;

	/* DMA Chennel2 -> Transfer dummy 0xff to USARTC0.Data */
	EDMA.CH2.ADDRCTRL     = EDMA_CH_RELOAD_NONE_gc | EDMA_CH_DIR_FIXED_gc;
	EDMA.CH2.ADDR         = (uint16_t)(&dmy);
	EDMA.CH2.DESTADDRCTRL = EDMA_CH_RELOAD_NONE_gc | EDMA_CH_DIR_FIXED_gc;
	EDMA.CH2.DESTADDR     = (uint16_t)(&USARTC0.DATA);
	EDMA.CH2.TRIGSRC      = EDMA_CH_TRIGSRC_USARTC0_DRE_gc;

	f_mount(&fs, "", 0);
	scan_files();

	EDMA.CH0.CTRLA = 0;
	EDMA.CH2.CTRLA = 0;
	EDMA.CTRL = 0;

	USARTC0.CTRLB     = 0;

	PORTC.OUTSET = PIN0_bm;
}

