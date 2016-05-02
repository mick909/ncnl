/* FAT file system test for FatFS */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>

#include "ff.h"
#include "diskio.h"
#include "usart.h"

/* for ATMega328P only */
/* low  : 1 0 1 1  0 1 1 1   :  CKOUT en, ext-Full Swing crystal, 8-16MHz, slowly rising power */
/* high : 1 1 0 1  1 1 1 1   :  BOOTRST disable (BOOTSZ = 256 words)                 */
/* ext  : 0 0 0 0  0 1 1 1   :  BOD disable                                          */
FUSES = {0xe2, 0xdb, 0x07};

void play(FIL *fp);

static
void init(void)
{
  DDRB  = 0b00000000;
  PORTB = 0b00000000;

  DDRC  = 0b00000000;
  PORTC = 0b00000000;

  DDRD  = 0b00011100;
  PORTD = 0b00011000;

  usart_init();

  sei();
}

FRESULT scan_files(char* path)
{
  FRESULT res;
  FILINFO fno;
  FIL fp;
  DIR dir;
  int i, j;
  char *fn;

  res = f_opendir(&dir, path);
  if (res == FR_OK) {

    i = strlen(path);

    for (;;) {
      res = f_readdir(&dir, &fno);
      fn = fno.fname;

      if (res || !fn[0]) break;
      if (fn[0] == '.' || fn[0] == '_') continue;

      j = strlen(fn);
      if (j < 4) continue;
      if (fn[j-4] != '.' || fn[j-3] != 'M' || fn[j-2] != 'P' || fn[j-1] != '3') continue;

      sprintf(&path[i], "/%s", fn);
      res = f_open(&fp, path, FA_OPEN_EXISTING | FA_READ);
      if (res == FR_OK) {

        usart_write_string("play : ");
        usart_write_string(path);
        usart_write_cr();

        play(&fp);

        usart_write_string("play : done\r\n");

      }
      path[i] = 0;

/*
      if (fno.fattrib & AM_DIR) {
        usart_write_string("directory :");
        usart_write_string(fn);
        usart_write_cr();
        if (strlen(path) + strlen(fn) > 60) {
          usart_write_string("too long path. abort\r\n");
          continue;
        } else {
          sprintf(&path[i], "/%s", fn);
          res = scan_files(path);
          if (res) break;
          path[i] = 0;
        }
      } else {
        usart_write_string("file :");
        usart_write_string(fn);
        usart_write_cr();
      }
*/

    }

/*
    usart_write_string("leave :");
    usart_write_string(path);
    usart_write_cr();
    usart_write_cr();
*/
  } else {
    usart_write_string("opendir failed :");
    usart_write_string(path);
    usart_write_cr();
  }

  return res;
}

void test(void);

int main(void)
{
  FATFS fs;
  FRESULT res;
  char path[64];

  init();

  usart_write_string("Start SD-Card Test \r\n");

  res = f_mount(&fs, "", 0);
  if (res != FR_OK) {
    usart_write_string("f_mout failed :");
    usart_write( (res / 10) + '0' );
    usart_write( (res % 10) + '0' );
    usart_write_cr();
    for (;;) ;
  }

//  test();
//  do {} while (1);

  strcpy(path, "0:");
  scan_files(path);

  usart_write_string("End\r\n");

  for (;;) ;
}