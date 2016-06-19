/*
  ATMega368P
    8MHz external xtal clock
    3.3V
    BOD Disable
    WDT Disalbe
    with 50x50 pcb

    LFUSE = F7
    HFUSE = DF
    EFUSE = FF

  PB0 : LED-d    : Out-Low
  PB1 : NC       : Out-Low
  PB2 : LED-dp   : Out-Low
  PB3 : LED-c    ; Out-Low
  PB4 : LED-g    : Out-Low
  PB5 : LED-a    : Out-Low
  PB6 : xtal
  PB7 : xtal

  PC0 : LED-CA1  : Out-Low
  PC1 : LED-CA2  : Out-Low
  PC2 : LED-f    : Out-Low
  PC3 : LED-CA3  : Out-Low
  PC4 : LED-CA4  : Out-Low
  PC5 : LED-b    : Out-Low
  PC6 : reset

  PD0 : S1       : In-PU  (PCINT16)
  PD1 : S2       : In-Pu  (PCINT17)
  PD2 : T1       : In-Pu
  PD3 : T2       : In-Pu
  PD4 : S3       : In-Pu
  PD5 : Buzz     : Out-Low (OC0B)
  PD6 : LED-e    : Out-Low
  PD7 : SENSOR(AIN1)
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

void set_display (uint16_t);
void enable_ac (void);

/*-----------------------------------------------------------------------*/
/* Work Area                                                             */
/*-----------------------------------------------------------------------*/
volatile uint8_t count5;

volatile uint16_t counter;
volatile uint16_t counts[10] = {0};

volatile uint8_t count_num;

/* Segmented digit's font data : PORTC, PORTB, PORTD */
volatile uint8_t seg_data[3*4] = {0};

const uint8_t seg_font[3*10] = {
/*   --b--f--     --agcP-d     -e------        */
  ~0b00100100, ~0b00101001, ~0b01000000,  /* 0 */
  ~0b00100000, ~0b00001000, ~0b00000000,  /* 1 */
  ~0b00100000, ~0b00110001, ~0b01000000,  /* 2 */
  ~0b00100000, ~0b00111001, ~0b00000000,  /* 3 */
  ~0b00100100, ~0b00011000, ~0b00000000,  /* 4 */
  ~0b00000100, ~0b00111001, ~0b00000000,  /* 5 */
  ~0b00000100, ~0b00111001, ~0b01000000,  /* 6 */
  ~0b00100100, ~0b00101000, ~0b00000000,  /* 7 */
  ~0b00100100, ~0b00111001, ~0b01000000,  /* 8 */
  ~0b00100100, ~0b00111001, ~0b00000000,  /* 9 */
};

volatile uint8_t dot = 1;
volatile uint8_t row = 0;
volatile uint8_t *sdrp = seg_data;

volatile uint8_t ac_blank;

const uint8_t tone[4] = {125-1, 94-1, 62-1, 31-1};
const uint8_t duty[4] = {62-1, 16-1, 16-1, 16-1};

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

volatile uint8_t pind_tmp;

ISR(PCINT2_vect)
{
  pind_tmp = PIND;
  PCICR = 0;
  PCMSK2 = 0;
}

/*-----------------------------------------------------------------------*/
/* SLEEP Mode                                                            */
/*-----------------------------------------------------------------------*/
static
uint8_t sleep(void)
{
  TIMSK2 = 0;
  TCCR2A = TCCR2B = 0;

  /* 1/32 Clock */
  cli();
  CLKPR = _BV(CLKPCE);
  CLKPR = 0b0101;
  sei();

  PRR = _BV(PRTWI) | _BV(PRTIM2) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRSPI)
        | _BV(PRUSART0) | _BV(PRADC);

  /* CA2's dp on */
  PORTC &= ~0b00011011;

  PORTB |= 0b00111101;
  PORTC |= 0b00100100;
  PORTD |= 0b01000000;
  PORTB &=~0b00000100;

  PORTC |= 0b00000010;

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  /* PCINT enable */
  PCICR  = _BV(PCIE2);
  PCMSK2 = _BV(PCINT16) | _BV(PCINT17);

  do {
    cli();
    wdt_reset();
    WDTCSR = _BV(WDCE) | _BV(WDE);
    WDTCSR = _BV(WDIE) | 0b110;    /* 1s */
    sei();

    sleep_mode();

    if (PCICR == 0) break;

    xorshift();

    /* blink dot per 0.5ms */
    PINB = 0b00000100;
  } while (1);

  cli();
  wdt_reset();
  WDTCSR = _BV(WDCE) | _BV(WDE);
  WDTCSR = 0;
  sei();

  /* if s1 pushed exit idle loop, else continue idle */
  return (pind_tmp & _BV(0));
}

/*-----------------------------------------------------------------------*/
/* IDLE Mode                                                             */
/*-----------------------------------------------------------------------*/
static
uint8_t idle(void)
{
  uint16_t prev;
  uint8_t count_pos;
  uint8_t s1_status = 0xff;
  uint8_t s2_status = 0xff;

  TIMSK2 = 0;
  TCCR2A = TCCR2B = 0;

  /* 1/32 Clock */
  cli();
  CLKPR = _BV(CLKPCE);
  CLKPR = 0b0101;

  counter = 0;
  prev = 0;
  sei();

  PRR = _BV(PRTWI) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRSPI)
        | _BV(PRUSART0) | _BV(PRADC);

  count_pos = (count_num) ? 1 : 0;
  set_display(counts[count_pos]);
  dot = 1;

  /* TC2 : 4ms Interval Interrupt */
  OCR2A  = 125-1;   /* (F_CPU / 32 / 8 / 1000 * 4) */
  TCCR2A = 0b010;
  TCCR2B = 0b010;
  TIMSK2 = _BV(OCIE2A);

  set_sleep_mode(SLEEP_MODE_EXT_STANDBY);

  do {
    uint16_t tmp;

    sleep_mode();
    cli();
    tmp = counter;
    sei();

    /* check 20ms interval */
    if (tmp != prev) {
      prev = tmp;

      xorshift();

      s1_status <<= 1;
      if ( !(PIND & _BV(0)) ) ++s1_status;

      s2_status <<= 1;
      if ( !(PIND & _BV(1)) ) ++s2_status;

      if (s2_status == 1) {
        /* first H->L trigger */
        cli();
        prev = 0;
        counter = 0;
        sei();

        if (count_num) {
          if (++count_pos > count_num) count_pos = 1;
          set_display(counts[count_pos]);
        }
      }

      /* 3min -> enter deep sleep */
      if (prev >= 18000 / 2) {
        return 1;
      }
    }
  } while ( s1_status != 1 );

  return 0;
}

/*-----------------------------------------------------------------------*/
/* DELAY Mode                                                            */
/*-----------------------------------------------------------------------*/
static
void delay(void)
{
  uint16_t count;
  uint16_t tmp;

  TIMSK2 = 0;
  TCCR2A = TCCR2B = 0;

  /* 1/128 Clock */
  cli();
  CLKPR = _BV(CLKPCE);
  CLKPR = 0b0111;
  counter = 0;
  sei();

  PRR = _BV(PRTWI) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRSPI)
        | _BV(PRUSART0) | _BV(PRADC);

  /* ---- */
  seg_data[1] = seg_data[4] = seg_data[7] = seg_data[10] = ~0b00010000;
  seg_data[0] = seg_data[3] = seg_data[6] = seg_data[ 9] = 0xff;
  seg_data[2] = seg_data[5] = seg_data[8] = seg_data[11] = 0xff;
  dot = 0;

  /* TC2 : 4ms Interval Interrupt */
  OCR2A  = 250-1;   /* (F_CPU / 128 / 1 / 1000 * 4) */
  TCCR2A = 0b010;
  TCCR2B = 0b001;
  TIMSK2 = _BV(OCIE2A);

  set_sleep_mode(SLEEP_MODE_EXT_STANDBY);

  count = (150
           + (uint16_t)(xorshift() & 0x3f)    /* 0 - 63 */
           + (uint16_t)(xorshift() & 0x0f));  /* 0 - 15 */

  do {
    sleep_mode();
    cli();
    tmp = counter;
    sei();
    xorshift();
  } while ( tmp < count );
}

/*-----------------------------------------------------------------------*/
/* RUN Mode                                                              */
/*-----------------------------------------------------------------------*/
static
uint8_t run(void)
{
  uint8_t start_sw = 0xff;
  uint8_t disp_sw = 0xff;
  uint8_t blink = 50;

  uint16_t prev = 0;

  TIMSK2 = 0;

  set_display(0);
  dot = 1;

  /* 8MHz Clock */
  cli();
  CLKPR = _BV(CLKPCE);
  CLKPR = 0b0000;
  sei();

  PRR = _BV(PRTWI) | _BV(PRTIM1) | _BV(PRSPI) | _BV(PRUSART0) | _BV(PRADC);

  /* TC2 : 2ms Interval Interrupt */
  OCR2A  = 250-1;   /* (F_CPU / 64 / 1000 * 2) */
  TCCR2A = 0b010;
  TCCR2B = 0;

  /* Reset TC2 Counter (With Prescaler) */
  TCNT2 = 0;
  GTCCR = _BV(PSRASY);

  TCCR0A = _BV(COM0B1) | _BV(WGM01) | _BV(WGM00);
  TCCR0B = 0;
  OCR0A = tone[(PIND >> 2) & 0x03];
  OCR0B = duty[(PIND >> 2) & 0x03];
  TCNT0 = 0;

  cli();
  count5 = 5;
  counter = 0;
  ac_blank = count_num = 0;
  TCCR0B = _BV(WGM02) | 0b011;
  sei();

  TCCR2B = 0b100;
  TIMSK2 = _BV(OCIE2A);

  /* setup Analog Comparator */
  enable_ac();

  set_sleep_mode(SLEEP_MODE_IDLE);

  do {
    uint16_t tmp;
    uint8_t num;

    sleep_mode();
    cli();
    tmp = counter;
/*
    if (tmp == 20000) {
      counter = tmp = 10000;
    }
*/
    num = count_num;
    sei();

    if (prev != tmp) {
      if (tmp == 30) {
        TCCR0A = TCCR0B = 0;
        PRR = _BV(PRTWI) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRSPI)
              | _BV(PRUSART0) | _BV(PRADC);
        PORTD &= ~(_BV(5));
      }

      if (num == 0) {
        set_display(tmp);
      } else if (num == 1 && prev <= counts[num]) {
        set_display(counts[num]);
      }
      prev = tmp;

      if (num == 9) {
        ac_blank = 0;
        dot = 1;
        break;
      }

      if (--blink == 0) {
        dot = 1 - dot;
        blink = 50;
      }

      if (tmp > 200) {
        start_sw <<= 1;
        if (!(PIND & _BV(0))) ++start_sw;
        disp_sw <<= 1;
        if (!(PIND & _BV(1))) ++ disp_sw;
      }

      if (tmp > 18000) return !sleep();
    }
  } while ( start_sw != 1 && disp_sw != 1);

  TIMSK2 = 0;
  TCCR2A = TCCR2B = 0;

  TCCR0A = TCCR0B = 0;
  PRR = _BV(PRTWI) | _BV(PRTIM2) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRSPI)
        | _BV(PRUSART0) | _BV(PRADC);
  PORTD &= ~(_BV(5));

  return start_sw == 1;
}

/*-----------------------------------------------------------------------*/
/* Main                                                                  */
/*-----------------------------------------------------------------------*/
int main (void)
{
  PORTB = 0b00000000;
  DDRB  = 0b00111111;

  PORTC = 0b01000000;
  DDRC  = 0b00111111;

  PORTD = 0b00011111;
  DDRD  = 0b01100000;

  DIDR0  = 0b00000010;

  set_display(0);
  dot = 1;

  PRR = _BV(PRTWI) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRSPI)
        | _BV(PRUSART0) | _BV(PRADC);

  count5 = 5;
  counter = 0;

  sei();

  for (;;) {

    do {
    } while (idle() && sleep());

    do {
      if (PIND & _BV(4)) delay();
    } while (run());
  }
}
