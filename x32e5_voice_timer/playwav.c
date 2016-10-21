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

	/* Setup DMA for USART SPI */
	EDMA.CTRL = EDMA_ENABLE_bm | EDMA_CHMODE_STD0_gc
			   | EDMA_DBUFMODE_DISABLE_gc | EDMA_PRIMODE_CH0123_gc;

	f_mount(&fs, "", 0);
	scan_files();

	EDMA.CTRL = 0;
}

