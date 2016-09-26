/*
  ATMega368P
    8MHz external xtal clock
    3.3V
    BOD Disable
    WDT Disalbe

    LFUSE = F7
    HFUSE = DF
    EFUSE = FF

  PB0 : NC       : In-PU
  PB1 : NC       : In-PU
  PB2 : BUZZ     : OC1B
  PB3 : CA3      ; Out-High (MOSI)
  PB4 : CA2      : Out-High (MISO)
  PB5 : CA1      ; Out-High (SCK)
  PB6 : xtal
  PB7 : xtal

  PC0 : H-B      : In-PU
  PC1 : H-A      : In-PU (PCINT9)
  PC2 : L-B      : In-PU
  PC3 : L-A      : In-PU (PCINT11)
  PC4 : I/D      : In-PU
  PC5 : Start    : In-PU (PCINT13)
  PC6 : reset

  PD0 : b        : Out-Low
  PD1 : dp       : Out-Low
  PD2 : c        : Out-Low
  PD3 : d        : Out-Low
  PD4 : e        : Out-Low
  PD5 : g        : Out-Low
  PD6 : f        : Out-Low
  PD7 : a        : Out-Low
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

void set_display (uint16_t);

/*-----------------------------------------------------------------------*/
/* Work Area                                                             */
/*-----------------------------------------------------------------------*/
volatile uint16_t par_count;

volatile uint16_t counter;

volatile uint8_t dot = 1;
volatile uint8_t row = 0;
volatile uint8_t seg_data[4] = {0x00, 0x00, 0x00, 0x00};
volatile uint8_t *sdrp = seg_data + 1;

volatile uint8_t count5;

void update_qdec();

volatile uint8_t    qdec0 = 0;
volatile uint8_t    qdec1 = 0;
volatile uint8_t    dec99 = 0;
volatile uint8_t  dec9900 = 0;

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

/*-----------------------------------------------------------------------*/
/* LED-Drive                                                             */
/*-----------------------------------------------------------------------*/

/*        0 1 2 3 4 5 6 7 8 9
        ---------------------
  7 : a   1 0 1 1 0 1 1 1 1 1
  6 : f   1 0 0 0 1 1 1 0 1 1
  5 : g   0 0 1 1 1 1 1 0 1 1
  4 : e   1 0 1 0 0 0 1 0 1 0
  3 : d   1 0 1 1 0 1 1 0 1 1
  2 : c   1 1 0 1 1 1 1 1 1 1
  1 : dp  0 0 0 0 0 0 0 0 0 0
  0 : b   1 1 1 1 1 0 0 1 1 1
*/

volatile const uint8_t seg_font[] = { 0xdd, 0x05, 0xb9, 0xad, 0x65, 0xec, 0xfc, 0x85, 0xfd, 0xed };


/*-----------------------------------------------------------------------*/
/* IDLE Mode                                                             */
/*-----------------------------------------------------------------------*/
static
uint8_t idle(void)
{
  uint8_t s1_status = 0xff;
  uint16_t prev;

  /* 1/8 Clock */
  cli();
  CLKPR = _BV(CLKPCE);
  CLKPR = 0b0011;

  PRR = _BV(PRTWI) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRSPI) | _BV(PRUSART0) | _BV(PRADC);

  counter = prev = 0;

  update_qdec();
  update_qdec();
  dec99 = dec9900 = 0;
  par_count = 0;
  set_display(par_count);
  dot = 1;

  /* TC2 : 2ms Interval Interrupt */
  OCR2A  = 250-1;   /* (F_CPU / 8 / 8 / 250= 500Hz (2ms)) */
  TCCR2A = 0b010;
  TCCR2B = 0b010;
  TIMSK2 = _BV(OCIE2A);

  PCMSK1 = _BV(PCINT9) | _BV(PCINT11);
  PCIFR = _BV(PCIF1);
  PCICR = _BV(PCIE1);

  sei();

  set_sleep_mode(SLEEP_MODE_EXT_STANDBY);

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

//      s1_status <<= 1;
//      if ( !(PINC & _BV(5)) ) ++s1_status;

      /* 10min -> enter deep sleep */
      if (prev >= 60000) {
        prev = 0;
        /* return 1; */
      }
    }
  } while ( 1 );

  TIMSK2 = 0;
  TCCR2A = TCCR2B = 0;

  PCICR = 0;
  PCMSK1 = 0;
  PCIFR = _BV(PCIF1);

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

  /* 1/128 Clock */
  cli();
  CLKPR = _BV(CLKPCE);
  CLKPR = 0b0111;
  counter = 0;
  sei();

  PRR = _BV(PRTWI) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRUSART0) | _BV(PRADC);

  seg_data[1] = seg_data[2] = seg_data[3] = 0x20;
  dot = 0;

  /* TC0 : 4ms Interval Interrupt */
  OCR0A  = 250-1;   /* (F_CPU / 128 / 1 / 1000 * 4) */
  TCCR0A = 0b010;
  TCCR0B = 0b001;
  TIMSK0 = _BV(OCIE0A);

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

  PRR = _BV(PRTWI) | _BV(PRTIM1) | _BV(PRUSART0) | _BV(PRADC);

  /* TC0 : 2ms Interval Interrupt */
  OCR0A  = 250-1;   /* (F_CPU / 64 / 1000 * 2) */
  TCCR0A = 0b010;
  TCCR0B = 0;

  /* Reset TC0 Counter (With Prescaler) */
  TCNT0 = 0;
  GTCCR = _BV(PSRSYNC);

  TCCR1A = _BV(COM1B1) | _BV(WGM01) | _BV(WGM00);
  TCCR1B = 0;
  OCR1A =
  OCR1B =
  TCNT1 = 0;

  cli();
  count5 = 5;
  counter = 0;
  TCCR1B = _BV(WGM02) | 0b011;
  sei();

  TCCR0B = 0b100;
  TIMSK0 = _BV(OCIE0A);

  set_sleep_mode(SLEEP_MODE_IDLE);

  do {
    uint16_t tmp;

    sleep_mode();
    cli();
    tmp = counter;
    sei();

    if (prev != tmp) {
      if (tmp == 30) {
        TCCR1A = TCCR1B = 0;
        PRR = _BV(PRTWI) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRUSART0) | _BV(PRADC);
        PORTB &= ~(_BV(2));
      }

      set_display(tmp);
      prev = tmp;

      if (--blink == 0) {
        dot = 1 - dot;
        blink = 50;
      }

      if (tmp > 200) {
        start_sw <<= 1;
        if (!(PINC & _BV(5))) ++start_sw;
      }

//      if (tmp > 60000) return !sleep();
    }
  } while ( start_sw != 1 );


  PRR = _BV(PRTWI) | _BV(PRTIM2) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRUSART0) | _BV(PRADC);
  PORTB &= ~(_BV(2));

  return start_sw == 1;
}

/*-----------------------------------------------------------------------*/
/* Main                                                                  */
/*-----------------------------------------------------------------------*/
int main (void)
{
  PORTB = 0b00111011;
  DDRB  = 0b00111100;

  PORTC = 0b00111111;
  DDRC  = 0b00000000;

  PORTD = 0b00000000;
  DDRD  = 0b11111111;

  set_display(0);
  dot = 1;

  PRR = _BV(PRTWI) | _BV(PRTIM2) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRSPI) | _BV(PRUSART0) | _BV(PRADC);

  count5 = 5;
  counter = 0;
  par_count = 0;

  for (;;) {
    idle();
/*
    do {
    } while (idle() && sleep());

    do {
      if (PINC & _BV(4)) delay();
    } while (run());
*/
  }
}
