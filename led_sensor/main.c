/* FUSE H = FF */
/* FUSE L = 6b , SUT=10, CKSEL=11 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

ISR(ANA_COMP_vect)
{
    ACSR &= ~_BV(ACIE);
    ACSR = _BV(ACD) | _BV(ACIS1);
}

EMPTY_INTERRUPT(WDT_vect);

int main(void)
{
    PORTB = 0b00111101;
    DDRB  = 0b00000001;
    DIDR0 = 0b00000010;

    while(1)
    {
        ACSR &= ~_BV(ACIE);
        ACSR = _BV(ACD) | _BV(ACIS1);

        PORTB = 0b00111100;

        PRR = _BV(PRADC) | _BV(PRTIM0);

        // start Timer
        wdt_reset();
        WDTCR = _BV(WDCE) | _BV(WDTIE) | _BV(WDP3) | 0b000;   // WDT 4sec

        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        sei();
        sleep_mode();
        cli();

        wdt_reset();
        WDTCR = _BV(WDCE);

        PORTB = 0b00111101;

        PRR = _BV(PRTIM0);

        // sensor on
        ACSR &= ~_BV(ACIE);
        ACSR = _BV(ACD) | _BV(ACIS1) | _BV(ACBG);
        PRR = _BV(PRTIM0);
        ACSR = _BV(ACI) | _BV(ACIS1) | _BV(ACBG);
        ACSR = _BV(ACIE) | _BV(ACIS1) | _BV(ACBG);

        set_sleep_mode(SLEEP_MODE_IDLE);
        sei();
        sleep_mode();
        cli();
    }
    return 0;
}