/*
 ATXMEGA32E5
 12.8MHz External Clock

 TCC4
  Normal Operation
  CLKSEL = DIV256 (clkPER / 256)
  PER = 10ms (12800000 / 256 / 500)
  TCC4_OVF -> Ev-Ch : 0
    CH0MUX = TCC4_OVF
    INTCTRLA = OVFINTLVL_LO -> get count(TCC5) to display

LED Update Interrupt at
  CCBUFA =  25 -> DIG1
  CCBUFB = 150 -> DIG2
  CCBUFC = 275 -> DIG3
  CCBUFD = 400 -> DIG4
    INTCTRLB = CCDINTLVL_LO | CCCINTLVL_LO | CCBINTLVL_LO | CCAINTLVL_LO

TCC5
  Normal Operation
  CLKSEL = EVCH0
  PER = 60000 (600.00sec)
    INTCTRLA = OVFINTLVL_LO -> overflow (60000 -> 10000)

  CCA capture -> counts 10ms
    CTRLB =
    CTRLD = EVACT_OFF | EVSEL_CH1
    CTRLE = CCAMODE_CAPT
    INTCTRLB = CCAINTLVL_MED
 */

#include <avr/io.h>
#include <avr/cpufunc.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>

#include "ff.h"
#include "diskio.h"
#include "sdaudio.h"

extern
uint8_t Timer1, Timer2;	/* 100Hz decrement timer */

volatile uint16_t count = 0;
volatile uint8_t digits[4] = {0xff, 0xff, 0xff, 0xff};

/* 16KHz Sampling, 400Hz sine wave */
/*
const uint8_t sin_400Hz_16KHz[] = {
	128, 148, 167, 186, 203, 218, 231, 241,
	249, 253, 255, 253, 249, 241, 231, 218,
	203, 186, 167, 148, 128, 108,  89,  70,
	 53,  38,  25,  15,   7,   3,   1,   3,
	  7,  15,  25,  38,  53,  70,  89, 108
};

#define SIN_400Hz_16KHz_NUM (40)
*/

/*         0 1 2 3 4 5 6 7 8 9
          ---------------------
  Q7 : DP  0 0 0 0 0 0 0 0 0 0
  Q6 : g   0 0 1 1 1 1 1 0 1 1
  Q5 : f   1 0 0 0 1 1 1 0 1 1
  Q4 : e   1 0 1 0 0 0 1 0 1 0
  Q3 : d   1 0 1 1 0 1 1 0 1 1
  Q2 : c   1 1 0 1 1 1 1 1 1 1
  Q1 : b   1 1 1 1 1 0 0 1 1 1
  Q0 : a   1 0 1 1 0 1 1 1 1 1
*/

volatile const uint8_t digit[] = { 0xc0, 0xf9, 0xa4, 0xb0, 0x99, 0x92, 0x82, 0xf8, 0x80, 0x90 };

void main_clocksource_select(uint8_t clkCtrl);
void clock_prescaler_select(uint8_t psConfig);

void setup_XOSC(void)
{
	/* Select XTAL1 for external oscillator. */
	OSC.XOSCCTRL = OSC_FRQRANGE_12TO16_gc | OSC_XOSCSEL_EXTCLK_gc;

	/* Enable External Clock Sounrce */
	OSC.CTRL |= OSC_XOSCEN_bm;

	/* wait until stable. */
	do { } while (!( OSC.STATUS & OSC_XOSCRDY_bm ));

	OSC.PLLCTRL = OSC_PLLSRC_XOSC_gc + 2;

	OSC.CTRL |= OSC_PLLEN_bm;
	do { } while (!( OSC.STATUS & OSC_PLLRDY_bm ));

	/* set the External Clock as the main clock source. */
	main_clocksource_select(CLK_SCLKSEL_PLL_gc);
}

ISR(TCC5_OVF_vect)
{
	TCC5.INTFLAGS = TC5_OVFIF_bm;
	TCC5.CNT = TCC5.CNT + 10000;
}

ISR(TCC4_OVF_vect)
{
	TCC4.INTFLAGS = TC4_OVFIF_bm;
	count = TCC5.CNT;

	sei();

	digits[0] = digit[count % 10];
	count /= 10;
	digits[1] = digit[count % 10];
	count /= 10;
	digits[2] = digit[count % 10] & 0x7f;
	count /= 10;
	if (count == 0) {
		digits[3] = 0xff;
	} else {
		digits[3] = digit[count % 10];
	}

	uint8_t n;

	n = Timer1;				/* 100Hz decrement timer */
	if (n) Timer1 = --n;
	n = Timer2;
	if (n) Timer2 = --n;
}

ISR(TCC4_CCA_vect)
{
	sei();
	PORTA.OUTCLR = PIN7_bm | PIN6_bm | PIN5_bm | PIN4_bm;
	PORTC.OUT = digits[0];
	PORTA.OUTSET = PIN4_bm;
}
ISR(TCC4_CCB_vect)
{
	sei();
	PORTA.OUTCLR = PIN7_bm | PIN6_bm | PIN5_bm | PIN4_bm;
	PORTC.OUT = digits[1];
	PORTA.OUTSET = PIN5_bm;
}
ISR(TCC4_CCC_vect)
{
	sei();
	PORTA.OUTCLR = PIN7_bm | PIN6_bm | PIN5_bm | PIN4_bm;
	PORTC.OUT = digits[2];
	PORTA.OUTSET = PIN6_bm;
}
ISR(TCC4_CCD_vect)
{
	sei();
	PORTA.OUTCLR = PIN7_bm | PIN6_bm | PIN5_bm | PIN4_bm;
	PORTC.OUT = digits[3];
	PORTA.OUTSET = PIN7_bm;
}

void setupTCC4_10ms(void)
{
	count = 0;

	EVSYS.CH0MUX = EVSYS_CHMUX_TCC4_OVF_gc;

	/* Normal Operation */
	/* EV CH0 */
	/* TOP = 60000 */
	TCC5.CTRLB = TC45_BYTEM_NORMAL_gc | TC45_CIRCEN_DISABLE_gc | TC45_WGMODE_NORMAL_gc;
	TCC5.CNT = 0;
	TCC5.PER = 60000 - 1;
	TCC5.INTCTRLA = TC45_OVFINTLVL_LO_gc;
	TCC5.INTFLAGS = TC5_OVFIF_bm;
	TCC5.CTRLA = TC45_CLKSEL_EVCH0_gc;

	/* Normal Operation */
	/* 12.8MHz div256 / 500 = 100Hz */
	/* Interrupt at 25, 150, 275, 400 (Level = Lo) */
	TCC4.CTRLB = TC45_BYTEM_NORMAL_gc | TC45_CIRCEN_DISABLE_gc | TC45_WGMODE_NORMAL_gc;
	TCC4.CNT = 0;
	TCC4.PER = 500*2 - 1;		/* 12.8MHz div256 / 500 = 100hz */
	TCC4.CCA =  25*2 - 1;
	TCC4.CCB = 150*2 - 1;
	TCC4.CCC = 275*2 - 1;
	TCC4.CCD = 400*2 - 1;
	TCC4.INTCTRLA = TC45_OVFINTLVL_MED_gc;
	TCC4.INTCTRLB = TC45_CCDINTLVL_LO_gc | TC45_CCCINTLVL_LO_gc | TC45_CCBINTLVL_LO_gc | TC45_CCAINTLVL_LO_gc;
	TCC4.INTFLAGS = TC4_CCDIF_bm | TC4_CCCIF_bm | TC4_CCBIF_bm | TC4_CCAIF_bm | TC4_OVFIF_bm;
	TCC4.CTRLA = TC45_CLKSEL_DIV256_gc;
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

			if (res || !fn[0]) { break; }
			if (fn[0] == '.' || fn[0] == '_') { continue; }

			j = strlen(fn);
			if (j < 4) continue;
			if (fn[j-4] != '.' || fn[j-3] != 'W' || fn[j-2] != 'A' || fn[j-1] != 'V') continue;

			sprintf(&path[i], "/%s", fn);
			res = f_open(&fp, path, FA_OPEN_EXISTING | FA_READ);
			if (res == FR_OK) {
				playfile(&fp);
			}
			path[i] = 0;
		}
	} else {
	}
	return res;
}

int main(void)
{
	FATFS fs;
	char path[64];

	setup_XOSC();
	OSC.CTRL &= ~OSC_RC2MEN_bm;

	PORTC.DIR = 0xff;
	PORTC.OUT = 0xff;
	PORTA.DIRSET = PIN7_bm | PIN6_bm | PIN5_bm | PIN4_bm;
	PORTA.OUTCLR = PIN7_bm | PIN6_bm | PIN5_bm | PIN4_bm;

	PORTA.DIRSET = PIN2_bm;
	PORTA.OUTCLR = PIN2_bm;

	setupTCC4_10ms();

//	output_wave(sin_400Hz_16KHz, SIN_400Hz_16KHz_NUM);

	/* Enable interrpts. */
	PMIC.CTRL |= PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	sei();

	f_mount(&fs, "", 0);

	strcpy(path, "0:");
	scan_files(path);

	do {} while (1);
}