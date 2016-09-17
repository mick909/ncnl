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

volatile uint16_t count = 0;
volatile uint8_t digits[4] = {0x3f, 0x28, 0x28, 0x28};

/* 16KHz Sampling, 400Hz sine wave */
const uint8_t sin_400Hz_16KHz[] = {
	128, 148, 167, 186, 203, 218, 231, 241,
	249, 253, 255, 253, 249, 241, 231, 218,
	203, 186, 167, 148, 128, 108,  89,  70,
	 53,  38,  25,  15,   7,   3,   1,   3,
	  7,  15,  25,  38,  53,  70,  89, 108
};

#define SIN_400Hz_16KHz_NUM (40)

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

	/* set the External Clock as the main clock source. */
	main_clocksource_select(CLK_SCLKSEL_XOSC_gc);
}

volatile const uint8_t* wave_buff;
volatile uint8_t wave_len;
volatile uint8_t wave_pt;

ISR(TCD5_OVF_vect)
{
	TCD5.INTFLAGS = TC5_OVFIF_bm;

//	PORTA.OUTTGL = PIN2_bm;

	DACA.CH0DATAL = 0;
	DACA.CH0DATAH = *(wave_buff + wave_pt);
	if (++wave_pt >= wave_len) wave_pt = 0;
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
	TCC4.PER = 500 - 1;		/* 12.8MHz div256 / 500 = 100hz */
	TCC4.CCA = 25 - 1;
	TCC4.CCB = 150 - 1;
	TCC4.CCC = 275 - 1;
	TCC4.CCD = 400 - 1;
	TCC4.INTCTRLA = TC45_OVFINTLVL_MED_gc;
	TCC4.INTCTRLB = TC45_CCDINTLVL_LO_gc | TC45_CCCINTLVL_LO_gc | TC45_CCBINTLVL_LO_gc | TC45_CCAINTLVL_LO_gc;
	TCC4.INTFLAGS = TC4_CCDIF_bm | TC4_CCCIF_bm | TC4_CCBIF_bm | TC4_CCAIF_bm | TC4_OVFIF_bm;
	TCC4.CTRLA = TC45_CLKSEL_DIV256_gc;
}

void output_wave(const uint8_t* const wave, const uint8_t len)
{
	wave_buff = wave;
	wave_len = len;
	wave_pt = 0;

	DACA.CTRLB = DAC_CHSEL_SINGLE_gc;
	DACA.CTRLC = DAC_REFSEL_AVCC_gc | DAC_LEFTADJ_bm;
	DACA.CTRLA = DAC_CH0EN_bm | DAC_ENABLE_bm;
	DACA.CH0DATA = 128 << 4;

	/* 12.8MHz / 800 = 16,000Hz */
	TCD5.CTRLB = TC45_BYTEM_NORMAL_gc | TC45_CIRCEN_DISABLE_gc | TC45_WGMODE_NORMAL_gc;
	TCD5.CNT = 0;
	TCD5.PER = 800-1;
	TCD5.INTCTRLA = TC45_OVFINTLVL_HI_gc;
	TCD5.INTFLAGS = TC5_OVFIF_bm;
	TCD5.CTRLA = TC45_CLKSEL_DIV1_gc;
}

int main(void)
{
	setup_XOSC();
	OSC.CTRL &= ~OSC_RC2MEN_bm;

	PORTC.DIR = 0xff;
	PORTC.OUT = 0xff;
	PORTA.DIRSET = PIN7_bm | PIN6_bm | PIN5_bm | PIN4_bm;
	PORTA.OUTCLR = PIN7_bm | PIN6_bm | PIN5_bm | PIN4_bm;

	PORTA.DIRSET = PIN2_bm;
	PORTA.OUTCLR = PIN2_bm;

	setupTCC4_10ms();

	output_wave(sin_400Hz_16KHz, SIN_400Hz_16KHz_NUM);

	/* Enable interrpts. */
	PMIC.CTRL |= PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	sei();

	do {} while (1);
}