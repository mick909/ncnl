/* FUSE H = FF */
/* FUSE L = 69 , SUT=10, CKSEL=01 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

EMPTY_INTERRUPT(INT0_vect);
EMPTY_INTERRUPT(ADC_vect);
EMPTY_INTERRUPT(WDT_vect);

/*-----------------------------------------------------------------------*/
/* Xorshift pseudo random generator                                      */
/*-----------------------------------------------------------------------*/
volatile uint32_t y = 2463534242;

uint32_t xorshift(void)
{
  y ^= (y<<13); y ^= (y >> 17); y ^= (y << 5);
  return y;
}

static void wait_switch(void)
{
    /* Wait INT0 Low */
    GIMSK = _BV(INT0);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sei();
    sleep_mode();
    cli();
    GIMSK = 0;
}

static void wait_fix_time(void)
{
    uint8_t adcin;
    /* ADC */
    PRR   = _BV(PRTIM0);
    ADMUX = _BV(ADLAR) | _BV(MUX0);
    ADCSRA = _BV(ADEN) | _BV(ADIE);

    set_sleep_mode(SLEEP_MODE_ADC);
    sei();
    sleep_mode();
    cli();

    adcin = ADCH;
    PRR   = _BV(PRTIM0) | _BV(PRADC);

    /* 0 : 0sec - 255 : 8sec */
    do {
        wdt_reset();
        WDTCR = _BV(WDCE) | _BV(WDTIE) | 0b001;   // WDT 32ms

        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        sei();
        sleep_mode();
        cli();

        xorshift();

        wdt_reset();
        WDTCR = _BV(WDCE);
    } while (adcin--);

    /* +2 sec */
    wdt_reset();
    WDTCR = _BV(WDCE) | _BV(WDTIE) | 0b111;   // WDT 4sec

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sei();
    sleep_mode();
    cli();

    wdt_reset();
    WDTCR = _BV(WDCE);
}

static void wait_additional_random(void)
{
    uint8_t cnt;

    cnt = (uint8_t)(xorshift() & 0x0f) + (uint8_t)(xorshift() & 0x1f);
    /* +0 to 3 sec */
    do {
        wdt_reset();
        WDTCR = _BV(WDCE) | _BV(WDTIE) | 0b010;   // WDT 64ms

        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        sei();
        sleep_mode();
        cli();

        xorshift();

        wdt_reset();
        WDTCR = _BV(WDCE);
    } while (cnt--);
}

static void wait_500ms(void)
{
    /* +2 sec */
    wdt_reset();
    WDTCR = _BV(WDCE) | _BV(WDTIE) | 0b101;   // WDT 500ms

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sei();
    sleep_mode();
    cli();

    wdt_reset();
    WDTCR = _BV(WDCE);
}

int main(void)
{
    PORTB = 0b00001010;
    /* PB0 : ADC Input V */
    /* PB1 : Switch(INT0)*/
    /* PB2 : ADC1        */
    /* PB3 : LED(INV)    */
    /* PB4 : Base Out    */
    DDRB  = 0b00011001;

    PRR   = _BV(PRTIM0) | _BV(PRADC);
    ACSR  = _BV(ACD);

    DIDR0 = _BV(ADC1D);

    do {
        wait_switch();

        /* LED ON, ADC in = VCC */
        PORTB &= ~_BV(3);
        PORTB |= _BV(0);

        wait_fix_time();

        wait_additional_random();

        /* LED OFF, ADC in = VCC, Base On */
        PORTB |= _BV(3) | _BV(4);
        PORTB &= ~_BV(0);

        wait_500ms();

        /* Base Off */
        PORTB &= ~_BV(4);
    } while(1);
}