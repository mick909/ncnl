.nolist
#include <avr/io.h>
.list

.macro STORE addr, reg
	.if	\addr > 0x3F
		sts	\addr, \reg
	.else
		out	_SFR_IO_ADDR(\addr), \reg
	.endif
.endm

.macro LOAD reg, addr
	.if	\addr > 0x3F
		lds	\reg, \addr
	.else
		in	\reg, _SFR_IO_ADDR(\addr)
	.endif
.endm

;---------------------------------------------------------------------------;
; void main_clocksource_select(uint8_t clkCtrl);

.global main_clocksource_select
.func main_clocksource_select
main_clocksource_select:
	ldi		r25, 0xD8
	STORE	CPU_CCP, r25
	STORE	CLK_CTRL, r24
	ret
.endfunc

;---------------------------------------------------------------------------;
; ISR(TCC4_CCx_vect)
;   update LED dynamic drive per 2.5 ms x 4 digit

.global TCC4_CCA_vect
.func TCC4_CCA_vect
TCC4_CCA_vect:
	push	r24
	LOAD	r24, SREG
	push	r24

	lds		r24, tcc4_intflags
	ori		r24, TC4_CCAIF_bm
	sts		tcc4_intflags, r24

	pop		r24
	STORE	SREG, r24
	pop		r24
	reti
.endfunc

.global TCC4_CCB_vect
.func TCC4_CCB_vect
TCC4_CCB_vect:
	push	r24
	LOAD	r24, SREG
	push	r24

	lds		r24, tcc4_intflags
	ori		r24, TC4_CCBIF_bm
	sts		tcc4_intflags, r24

	pop		r24
	STORE	SREG, r24
	pop		r24
	reti
.endfunc

.global TCC4_CCC_vect
.func TCC4_CCC_vect
TCC4_CCC_vect:
	push	r24
	LOAD	r24, SREG
	push	r24

	lds		r24, tcc4_intflags
	ori		r24, TC4_CCCIF_bm
	sts		tcc4_intflags, r24

	pop		r24
	STORE	SREG, r24
	pop		r24
	reti
.endfunc

.global TCC4_CCD_vect
.func TCC4_CCD_vect
TCC4_CCD_vect:
	push	r24
	LOAD	r24, SREG
	push	r24

	lds		r24, tcc4_intflags
	ori		r24, TC4_CCDIF_bm
	sts		tcc4_intflags, r24

	pop		r24
	STORE	SREG, r24
	pop		r24
	reti
.endfunc

;---------------------------------------------------------------------------;
; void set_led_digit(uint16_t v)
;   update digit[] fount data by v
;
; break: r22, r24, r25, XL, XH, SREG

.global set_led_digit
.func set_led_digit
set_led_digit:
	; DIG1
	clr		XL
	ldi		r22, 16
1:
	lsl		r24
	rol		r25
	rol		XL
	cpi		XL, 10
	brcs	2f
	subi	XL, 10
	inc		r24
2:
	dec		r22
	brne	1b

	clr		XH
	subi	XL, lo8(-(digit))
	sbci	XH, hi8(-(digit))
	ld		r22, X
	sts		digits, r22

	; DIG2
	clr		XL
	ldi		r22, 16
1:
	lsl		r24
	rol		r25
	rol		XL
	cpi		XL, 10
	brcs	2f
	subi	XL, 10
	inc		r24
2:
	dec		r22
	brne	1b

	clr		XH
	subi	XL, lo8(-(digit))
	sbci	XH, hi8(-(digit))
	ld		r22, X
	sts		digits+1, r22

	; DIG3
	clr		XL
	ldi		r22, 16
1:
	lsl		r24
	rol		r25
	rol		XL
	cpi		XL, 10
	brcs	2f
	subi	XL, 10
	inc		r24
2:
	dec		r22
	brne	1b

	clr		XH
	subi	XL, lo8(-(digit))
	sbci	XH, hi8(-(digit))
	ld		r22, X
	andi	r22, 0xdf
	sts		digits+2, r22

	; DIG4
	clr		XL
	cp		r24, XL
	cpc		r25, XL
	breq	9f

	ldi		r22, 16
1:
	lsl		r24
	rol		r25
	rol		XL
	cpi		XL, 10
	brcs	2f
	subi	XL, 10
	inc		r24
2:
	dec		r22
	brne	1b

	clr		XH
	subi	XL, lo8(-(digit))
	sbci	XH, hi8(-(digit))
	ld		r22, X
	sts		digits+3, r22

	ret

9:
	ldi		r22, 0xff
	sts		digits+3, r22

	ret
.endfunc
