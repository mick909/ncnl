/*
  ATMega368P + ATTiny13a + HT82V739
    8MHz external xtal clock
    3.3V
    BOD Disable
    WDT Disalbe

  PB0 : LED-b    : Out-Low
  PB1 : LED-CA4  : Out-Low
  PB2 : LED-g    : Out-Low
  PB3 : LED-c    ; Out-Low
  PB4 : LED-dp   : Out-Low
  PB5 : LED-d    : Out-Low
  PB6 : xtal
  PB7 : xtal

  PC0 : LED-e    : Out-Low
  PC1 : ADC1(AIN): In-HiZ
  PC2 : S1       : In-PU  (PCINT10)
  PC3 : S2       : In-Pu  (PCINT11)
  PC4 : S3       : In-Pu
  PC5 : AMP En   : Out-High
  PC6 : reset

  PD0 : RxD      : In-HiZ
  PD1 : TxD      : Out-Low
  PD2 : LED-f    : Out-Low
  PD3 : LED-CA1  ; Out-Low
  PD4 : LED-a    : Out-Low
  PD5 : BuzzPWM  : Out-Low (OC0B)
  PD6 : LED-CA2  : Out-Low
  PD7 : LED-CA3  : Out-Low
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

void delay_ms (uint16_t);
void delay_us (uint16_t);

/*-----------------------------------------------------------------------*/
/* Work Area                                                             */
/*-----------------------------------------------------------------------*/
volatile uint8_t count5;

volatile uint16_t counter;
volatile uint16_t counts[10] = {0};

volatile uint8_t count_num;

/* Segmented digit's font data : PORTB, PORTC, PORTD */
volatile uint8_t seg_data[3*4] = {0};

const uint8_t seg_font[3*10] = {
/*   --D.CG-B     -------E     ---A-F--        */
  ~0b00101001, ~0b00000001, ~0b00010100,  /* 0 */
  ~0b00001001, ~0b00000000, ~0b00000000,  /* 1 */
  ~0b00100101, ~0b00000001, ~0b00010000,  /* 2 */
  ~0b00101101, ~0b00000000, ~0b00010000,  /* 3 */
  ~0b00001101, ~0b00000000, ~0b00000100,  /* 4 */
  ~0b00101100, ~0b00000000, ~0b00010100,  /* 5 */
  ~0b00101100, ~0b00000001, ~0b00010100,  /* 6 */
  ~0b00001001, ~0b00000000, ~0b00010100,  /* 7 */
  ~0b00101101, ~0b00000001, ~0b00010100,  /* 8 */
  ~0b00101101, ~0b00000000, ~0b00010100   /* 9 */
};

volatile uint8_t row = 0;
volatile uint8_t *sdrp = seg_data;

volatile uint16_t buzz_time;
volatile uint8_t buzz_pos;
volatile uint8_t buzz_cnt;
volatile uint8_t buzz_shut;

volatile uint8_t ac_blank;

/*-----------------------------------------------------------------------*/
/* Xorshift pseudo random generator                                      */
/*-----------------------------------------------------------------------*/
volatile uint32_t y = 2463534242;

uint32_t xorshift(void)
{
  y ^= (y<<13); y ^= (y >> 17); y ^= (y << 5);
  return y;
}

/*-----------------------------------------------------------------------*/
/* Reset Watchdog on boot                                                */
/*-----------------------------------------------------------------------*/
void get_mcusr(void) __attribute__((naked)) __attribute__((section(".init3")));
void get_mcusr(void)
{
  MCUSR = 0;
  wdt_disable();
}

/*-----------------------------------------------------------------------*/
/* Interrupts                                                            */
/*-----------------------------------------------------------------------*/

EMPTY_INTERRUPT(WDT_vect);

ISR(PCINT1_vect) {
  PCICR = 0;
  PCMSK1 = 0;
}

ISR(TIMER2_COMPA_vect) {
  if (--count5 == 0) {
    count5 = 5;
    counter += 1;
  }

  if (buzz_shut) {
    PORTC |= _BV(5);
    if (--buzz_shut == 0) {
      TCCR0A = TCCR0B = 0;
      TIMSK0 = 0;
      PRR = _BV(PRTWI) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRSPI)
        | _BV(PRUSART0) | _BV(PRADC);
    }
  }

  PORTB &= ~0b00000010;
  PORTD &= ~0b11001000;

  PORTB |= 0b00111101;
  PORTC |= 0b00000001;
  PORTD |= 0b00010100;

  PORTB &= *sdrp++;
  PORTC &= *sdrp++;
  PORTD &= *sdrp++;

  switch (row++) {
    case 0:
      PORTB |= 0b00000010;
      break;
    case 1:
      PORTD |= 0b10000000;
      break;
    case 2:
      PORTD |= 0b01000000;
//      if (!dot) {
//        PORTB |= 0b00010000;
//      }
      break;
    case 3:
      PORTD |= 0b00001000;

      row = 0;
      sdrp = seg_data;
      break;
  }
}

/*-----------------------------------------------------------------------*/
/* Display Func                                                          */
/*-----------------------------------------------------------------------*/
void set_time(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4)
{
  volatile uint8_t* dp = seg_data;

  volatile const uint8_t* fp = seg_font + (d0 * 3);
  *dp++ = *fp++;
  *dp++ = *fp++;
  *dp++ = *fp;

  fp = seg_font + (d1 * 3);
  *dp++ = *fp++;
  *dp++ = *fp++;
  *dp++ = *fp;

  fp = seg_font + (d2 * 3);
  *dp++ = *fp++;
  *dp++ = *fp++;
  *dp++ = *fp;

  if (d3 + d4) {
    fp = seg_font + (d3 * 3);
    *dp++ = *fp++;
    *dp++ = *fp++;
    *dp++ = *fp;
  } else {
    *dp++ = 0xff;
    *dp++ = 0xff;
    *dp   = 0xff;
  }

  seg_data[6] &= ~0b00010000;
}

static
void set_count(uint16_t c)
{
  uint8_t d0, d1, d2, d3, d4;

  d4 = c / 10000;
  c -= d4 * 10000;

  d3 = c / 1000;
  c -= d3 * 1000;

  d2 = c / 100;
  c -= d2 * 100;

  d1 = c / 10;
  d0 = c - d1 * 10;

  set_time(d0, d1, d2, d3, d4);
}

/*-----------------------------------------------------------------------*/
/* SLEEP Mode                                                            */
/*-----------------------------------------------------------------------*/
static
uint8_t sleep(void)
{
  TIMSK2 = 0;
  TCCR2A = TCCR2B = 0;

  PRR = _BV(PRTWI) | _BV(PRTIM2) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRSPI)
        | _BV(PRUSART0) | _BV(PRADC);

  /* CA2's dp on */
  PORTB &= ~0b00000010;
  PORTD &= ~0b11001000;
  PORTD |=  0b01000000;

  PORTB |= 0b00111101;
  PORTC |= 0b00000001;
  PORTD |= 0b00010100;
  PORTB &=~0b00010000;

  /* PCINT enable */
  PCICR  = _BV(PCIE1);
  PCMSK1 = _BV(PCINT10) | _BV(PCINT11);

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  do {
    cli();
    wdt_reset();
    WDTCSR = _BV(WDCE) | _BV(WDE);
    WDTCSR = _BV(WDIE) | 0b110;    /* 1s */
    sei();

    sleep_mode();

    xorshift();

    /* blink dot per 0.5ms */
    PINB = 0b00010000;
  } while ( (PINC & _BV(2)) && (PINC & _BV(3)) );

  cli();
  wdt_reset();
  WDTCSR = _BV(WDCE) | _BV(WDE);
  WDTCSR = 0;
  sei();

  /* if s1 pushed exit idle loop, else continue idle */
  return (PINC & _BV(2));
}

/*-----------------------------------------------------------------------*/
/* IDLE Mode                                                             */
/*-----------------------------------------------------------------------*/
static
uint8_t idle(void)
{
  uint16_t prev;
  uint8_t count_pos;
  uint8_t s2_status = 0xff;

  PRR = _BV(PRTWI) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRSPI)
        | _BV(PRUSART0) | _BV(PRADC);

  /* TC2 : 2ms Interval Interrupt */
  OCR2A  = 250-1;   /* (F_CPU / 8 / 1000 * 2) */
  TCCR2A = 0b010;
  TCCR2B = 0b010;
  TIMSK2 = _BV(OCIE2A);

  set_sleep_mode(SLEEP_MODE_EXT_STANDBY);

  cli();
  counter = 0;
  sei();
  prev = 0;
  count_pos = (count_num) ? 1 : 0;
  set_count(counts[count_pos]);

  /* PCINT enable (S1 only) */
  PCICR  = _BV(PCIE1);
  PCMSK1 = _BV(PCINT10);

  do {
    uint16_t tmp;

    sleep_mode();
    cli();
    tmp = counter;
    sei();

    /* check 10ms interval */
    if (tmp != prev) {
      prev = tmp;

      xorshift();

      s2_status <<= 1;
      if ( PINC & _BV(3) ) ++s2_status;
      if (s2_status == 1) {
        /* first H->L trigger */
        prev = 0;
        cli();
        counter = 0;
        sei();
      }

      /* 3min -> enter deep sleep */
      if (prev >= 3000) {
        return 1;
      }
    }
  } while ( PINC & _BV(2) );

  return 0;
}

/*-----------------------------------------------------------------------*/
/* DELAY Mode                                                            */
/*-----------------------------------------------------------------------*/
static
void delay(void)
{
  uint8_t count;

  TIMSK2 = 0;
  TCCR2A = TCCR2B = 0;

  PRR = _BV(PRTWI) | _BV(PRTIM2) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRSPI)
        | _BV(PRUSART0) | _BV(PRADC);

  /* ---- */
  PORTB |=  0b00111111;
  PORTC |=  0b00000001;
  PORTD |=  0b11011100;
  PORTB &= ~0b00000100;

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  /* 64ms * 48 = 3.072sec */
  /* 64ms * 24 = 1.536sec */
  count = 48 + (uint8_t)(xorshift() % 25);

  do {
    cli();
    wdt_reset();
    WDTCSR = _BV(WDCE) | _BV(WDE);
    WDTCSR = _BV(WDIE) | 0b010;    /* 64ms */
    sei();

    sleep_mode();

    xorshift();
  } while ( --count > 0);

  cli();
  wdt_reset();
  WDTCSR = _BV(WDCE) | _BV(WDE);
  WDTCSR = 0;
  sei();
}

/*-----------------------------------------------------------------------*/
/* RUN Mode                                                              */
/*-----------------------------------------------------------------------*/
static
uint8_t run(void)
{
  uint8_t start_sw = 0xff;
  uint8_t disp_sw = 0xff;

  uint16_t prev = 0;

  TIMSK2 = 0;

  set_time(0,0,0,0,0);

  /* 8MHz Clock */
  cli();
  CLKPR = _BV(CLKPCE);
  CLKPR = 0b0000;

  counter = 0;
  buzz_cnt = buzz_pos = buzz_shut = ac_blank = count_num = 0;
  buzz_time = 47875 * 0.3;
  sei();

  PRR = _BV(PRTWI) | _BV(PRTIM1) | _BV(PRSPI)
        | _BV(PRUSART0) | _BV(PRADC);

  /* PWM Center */
  OCR0B = 128;
  TCCR0A = _BV(COM0B1) | _BV(WGM01) | _BV(WGM00);
  TCCR0B = _BV(CS00);

  delay_ms(1);

  PORTC &= ~(_BV(5));

  delay_ms(30);

  /* TC2 : 2ms Interval Interrupt */
  OCR2A  = 250-1;   /* (F_CPU / 8 / 1000 * 2) */
  TCCR2A = 0b010;
  TCCR2B = 0;

  /* Reset TC2 Counter (With Prescaler) */
  TCNT2 = 0;
  GTCCR = _BV(PSRASY);

  /* Start Buzzer & Timer */
  TIMSK0 = _BV(TOIE0);
  TCCR2B = 0b100;
  TIMSK2 = _BV(OCIE2A);

  set_sleep_mode(SLEEP_MODE_IDLE);

  do {
    uint16_t tmp;
    uint8_t num;

    sleep_mode();
    cli();
    tmp = counter;
    num = count_num;
    sei();

    if (prev != tmp) {
      if (num == 0) {
        set_count(tmp);
      } else if (prev < counts[num]) {
        set_count(counts[num]);
      }
      prev = tmp;

      if (num == 9) break;

      if (tmp > 200) {
        start_sw <<= 1;
        if (!(PINC & _BV(2))) ++start_sw;
        disp_sw <<= 1;
        if (!(PINC & _BV(3))) ++ disp_sw;
      }

      if (tmp > 18000) return !sleep();
    }
  } while ( start_sw != 1 && disp_sw != 1);

  TIMSK2 = 0;
  TCCR2A = TCCR2B = 0;

  TIMSK0 = 0;
  TCCR0A = TCCR0B = 0;

  PORTC |= _BV(5);
  PORTD &= ~(_BV(5));

  /* 1MHz Clock */
  cli();
  CLKPR = _BV(CLKPCE);
  CLKPR = 0b0011;
  sei();

  return start_sw == 1;
}

/*-----------------------------------------------------------------------*/
/* Main                                                                  */
/*-----------------------------------------------------------------------*/
int main (void)
{
  PORTB = 0b00000000;
  DDRB  = 0b00111111;

  PORTC = 0b00111100;
  DDRC  = 0b00100001;

  PORTD = 0b00000000;
  DDRD  = 0b11111110;

  set_time(0,0,0,0,0);

  PRR = _BV(PRTWI) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRSPI)
        | _BV(PRUSART0) | _BV(PRADC);

  count5 = 5;
  counter = 0;

  /* TC2 : 2ms Interval Interrupt */
  OCR2A  = 250-1;   /* (F_CPU / 8 / 1000 * 2) */
  TCCR2A = 0b010;
  TCCR2B = 0b010;
  TIMSK2 = _BV(OCIE2A);

  sei();

  for (;;) {

    do {
    } while (idle() && sleep());

    do {
      if (PINC & _BV(4)) delay();
    } while (run());
  }
}
