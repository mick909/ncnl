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

  PB0 : NC       : Out-Low
  PB1 : NC       : Out-Low
  PB2 : NC       : Out-Low
  PB3 : MOSI     ; Out-Low
  PB4 : MISO     : Out-High
  PB5 : SCK      ; Out-Low
  PB6 : xtal
  PB7 : xtal

  PC0 : L-A      : In-PU
  PC1 : L-B      : In-PU
  PC2 : H-A      : In-PU
  PC3 : H-B      : In-PU
  PC4 : NC       : Out-Low
  PC5 : NC       : Out-Low
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
volatile uint16_t par_count;

volatile uint16_t counter;
volatile uint16_t counts[10] = {0};

volatile uint8_t count_num;

volatile uint8_t ac_blank;

volatile uint8_t dot = 1;
volatile uint8_t row = 0;
volatile uint8_t seg_data[4] = {0xff, 0xff, 0xff, 0xff};

volatile uint8_t count5;

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
/* LED-Drive                                                             */
/*-----------------------------------------------------------------------*/

/*         0 1 2 3 4 5 6 7 8 9  A b C d E F
          ---------------------
  Q7 : e   0 1 0 1 1 1 0 1 0 1  0 0 0 0 0 0
  Q6 : d   0 1 0 0 1 0 0 1 0 0  1 0 0 0 0 1
  Q5 : DP  1 1 1 1 1 1 1 1 1 1  1 1 1 1 1 1
  Q4 : c   0 0 1 0 0 0 0 0 0 0  0 0 1 0 1 1
  Q3 : g   1 1 0 0 0 0 0 1 0 0  0 0 0 0 0 0
  Q2 : a   0 1 0 0 1 0 0 0 0 0  0 1 0 1 0 0
  Q1 : f   0 1 1 1 0 0 0 1 0 0  0 0 0 1 0 0
  Q0 : b   0 0 0 0 0 1 1 0 0 0  0 1 1 0 1 1
*/

volatile const uint8_t seg_font[] = { 0x28, 0xee, 0x32, 0xa2, 0xe4, 0xa1, 0x21, 0xea, 0x20, 0xa0, 0x60, 0x25, 0x31, 0x26, 0x31, 0x71 };

void init_spi_led(void)
{
/* Clear 74HC595 */
  SPCR  = 0b01010000; /* SPIE=0, SPE=1, MSTR=1, CPOL=SPHA=0, SPR=osc/2 */
  SPSR  = 0b00000001; /* SPI2X=1 */

  SPDR  = 0xff;
  do {} while ( !(SPSR & 0b10000000) );
  SPCR = 0;     /* SPI Disable */

/* Clear 74AC164 */
  PORTB &= ~0b00001000;
  PINB  = 0b00010000; /* MISO output L */
  PINB  = 0b00010000; /* MISO output H -> load */
  PINB  = 0b00010000; /* MISO output L */
  PINB  = 0b00010000; /* MISO output H -> load  */
  PINB  = 0b00010000; /* MISO output L */
  PINB  = 0b00010000; /* MISO output H -> load  */
  PINB  = 0b00010000; /* MISO output L */
  PINB  = 0b00010000; /* MISO output H -> load  */
  PINB  = 0b00010000; /* MISO output L */
}

void output_spi_led(uint8_t d, uint8_t f)
{
  SPCR  = 0b01010000; /* SPIE=0, SPE=1, MSTR=1, CPOL=SPHA=0, SPR=osc/2 */
  SPSR  = 0b00000001; /* SPI2X=1 */

  SPDR  = d;
  do {} while ( !(SPSR & 0b10000000) );
  SPCR = 0;     /* SPI Disable */

  PORTB &= ~0b00001000;     /* setup MOSI as 74AC164's data */
  if (f) PORTB |= 0b00001000;

  PINB  = 0b00010000; /* MISO output H */
  PINB  = 0b00010000; /* MISO output L */
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

  PRR = _BV(PRTWI) | _BV(PRTIM2) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRUSART0) | _BV(PRADC);

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
  uint8_t r1_status = PINC & 0x03;
  uint8_t r2_status = (PINC >> 2) & 0x03;
  uint8_t rcnt = 2;

  TIMSK2 = 0;
  TCCR2A = TCCR2B = 0;

  /* 1/16 Clock */
  cli();
  CLKPR = _BV(CLKPCE);
  CLKPR = 0b0100;

  counter = 0;
  prev = 0;
  sei();

  PRR = _BV(PRTWI) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRUSART0) | _BV(PRADC);

  count_pos = (count_num) ? 1 : 0;
  set_display(counts[count_pos]);
  dot = 1;

  /* TC2 : 2ms Interval Interrupt */
  OCR2A  = 125-1;   /* (F_CPU / 16 / 8 / 125= 500Hz (2ms)) */
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

    // check every 2ms
    {
      r1_status = (r1_status << 2) + (PINC & 0x03);
      if ( (r1_status & 0x0f) == 0x08 ) {
        par_count = par_count + ((par_count + 10) % 100) - (par_count % 100);
        set_display(par_count);
      } else if ( (r1_status & 0x0f) == 0x02 ) {
        par_count = par_count + ((par_count + 90) % 100) - (par_count % 100);
        set_display(par_count);
      }

      r2_status = (r2_status << 2) + ((PINC >> 2) & 0x03);
      if ( (r2_status & 0x0f) == 0x08 ) {
        par_count = par_count + ((par_count + 100) % 10000) - (par_count % 10000);
        set_display(par_count);
      } else if ( (r2_status & 0x0f) == 0x02 ) {
        par_count = par_count + ((par_count + 9900) % 10000) - (par_count % 10000);
        set_display(par_count);
      }

    }

    /* check 10ms interval */
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
      if (prev >= 18000) {
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

  PRR = _BV(PRTWI) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRUSART0) | _BV(PRADC);

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

  PRR = _BV(PRTWI) | _BV(PRTIM1) | _BV(PRUSART0) | _BV(PRADC);

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
        PRR = _BV(PRTWI) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRUSART0) | _BV(PRADC);
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
  PRR = _BV(PRTWI) | _BV(PRTIM2) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRUSART0) | _BV(PRADC);
  PORTD &= ~(_BV(5));

  return start_sw == 1;
}

/*-----------------------------------------------------------------------*/
/* Main                                                                  */
/*-----------------------------------------------------------------------*/
int main (void)
{
  PORTB = 0b00010000;
  DDRB  = 0b00111111;

  PORTC = 0b00001111;
  DDRC  = 0b00110000;

  PORTD = 0b00011111;
  DDRD  = 0b01100000;

  init_spi_led();

  set_display(0);
  dot = 1;

  PRR = _BV(PRTWI) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRUSART0) | _BV(PRADC);

  count5 = 5;
  counter = 0;
  par_count = 0;

  count_num = 0;

  sei();

  for (;;) {

    do {
    } while (idle() && sleep());

    do {
      if (PIND & _BV(4)) delay();
    } while (run());
  }
}
