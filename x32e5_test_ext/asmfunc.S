.nolist
#include <avr/io.h>
.list

;---------------------------------------------------------------------------;
; void main_clocksource_select(uint8_t clkCtrl);

.global main_clocksource_select
.func main_clocksource_select
main_clocksource_select:
	ldi		r25, 0xD8
	out		_SFR_IO_ADDR(CPU_CCP), r25
	sts		CLK_CTRL, r24
	ret
.endfunc

;---------------------------------------------------------------------------;
; void clock_prescaler_select(uint8_t psConfig);

.global clock_prescaler_select
.func clock_prescaler_select
clock_prescaler_select:
	ldi		r25, 0xD8
	out		_SFR_IO_ADDR(CPU_CCP), r25
	sts		CLK_PSCTRL, r24
	ret
.endfunc
