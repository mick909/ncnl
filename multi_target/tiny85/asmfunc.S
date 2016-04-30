;---------------------------------------------------------------------------;
;
;---------------------------------------------------------------------------;

.nolist
#include <avr/io.h>	// Include device specific definitions.
.list

.global delay_ms
.func delay_ms
delay_ms:
	sbiw	r24, 1
	brcs	9f
	ldi		ZL, lo8(F_CPU / 4000)
	ldi		ZH, hi8(F_CPU / 4000)
1:
	sbiw	ZL, 1
	brne	1b
	rjmp	delay_ms
9:	ret
.endfunc

.global delay_06us
.func delay_06us
delay_06us:
	ret				; rcall + ret = 7clock
.endfunc

.global delay_13us
.func delay_13us
delay_13us:
	nop				; rcall + ret = 7clock + nop * 5
	nop
	nop
	nop
	nop
	ret
.endfunc
