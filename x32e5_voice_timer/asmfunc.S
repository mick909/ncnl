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
; ISR(TCD5_OVF_vect)
;   Output Audio Sampling data to DAC0

.global TCD5_OVF_vect
.func TCD5_OVF_vect
TCD5_OVF_vect:
	push	r24
	LOAD	r24, SREG
	push	r24

	push	r23
	push	r25
	push	r26
	push	r27
	push	ZL
	push	ZH

	ldi		r24, TC5_OVFIF_bm;
	STORE	TCD5_INTFLAGS, r24

	lds		r24, fifo_ct
	lds		r25, fifo_ct+1

	lds		r26, fifo_ri
	lds		r27, fifo_ri+1

	ldi		ZL, lo8(buffer)
	ldi		ZH, hi8(buffer+1)

	add		ZL, r26
	adc		ZH, r27

	lds		r23, channel
	cpi		r23, 1
	brne	stereo

	lds		r23, resolution
	cpi		r23, 8
	brne	mono_b16

;Mono 8bit
	sbiw	r24, 1
	brcc	1f
	rjmp		9f
1:
	adiw	r26, 1
	andi	r27, hi8(2048-1)

	sts		fifo_ct+1, r25
	sts		fifo_ct,   r24
	sts		fifo_ri+1, r27
	sts		fifo_ri,   r26

	ld		r25, Z
	ldi		r24, 0
	STORE	DACA_CH0DATA+1, r25
	STORE	DACA_CH0DATA,   r24
	rjmp	9f

mono_b16:
	sbiw	r24, 2
	brcc	2f
	rjmp	9f
2:
	adiw	r26, 2
	andi	r27, hi8(2048-1)

	sts		fifo_ct+1, r25
	sts		fifo_ct,   r24
	sts		fifo_ri+1, r27
	sts		fifo_ri,   r26

	ld		r24, Z+
	andi	r24, 0xf0
	ld		r25, Z
	subi	r25, 0x80
	STORE	DACA_CH0DATA+1, r25
	STORE	DACA_CH0DATA,   r24
	rjmp	9f

stereo:
	lds		r23, resolution
	cpi		r23, 8

	brne	3f

	sbiw	r24, 2
	brcs	9f
	adiw	r26, 2
	andi	r27, hi8(2048-1)

	sts		fifo_ct+1, r25
	sts		fifo_ct,   r24
	sts		fifo_ri+1, r27
	sts		fifo_ri,   r26

	ld		r25, Z+
	ldi		r24, 0
	lsr		r25
	ror		r24

	ld		r27, Z
	ldi		r26, 0
	lsr		r27
	ror		r26

	add		r24, r26
	adc		r25, r27

	STORE	DACA_CH0DATA+1, r25
	STORE	DACA_CH0DATA,   r24
	rjmp	9f

3:
	sbiw	r24, 4
	brcs	9f
	adiw	r26, 4
	andi	r27, hi8(2048-1)

	sts		fifo_ct+1, r25
	sts		fifo_ct,   r24
	sts		fifo_ri+1, r27
	sts		fifo_ri,   r26

	ld		r24, Z+
	ld		r25, Z+
	subi	r25, 0x80
	lsr		r25
	ror		r24

	; if fifo_ri == 2 then Z = &(buffer[0])
	cpi		r27, 0
	brne	4f
	cpi		r26, 2
	brne	4f
	ldi		ZL, lo8(buffer)
	ldi		ZH, hi8(buffer+1)
4:

	ld		r26, Z+
	ld		r27, Z+
	subi	r27, 0x80
	lsr		r27
	ror		r26

	add		r24, r26
	adc		r25, r27
	andi	r24, 0xf0

	STORE	DACA_CH0DATA+1, r25
	STORE	DACA_CH0DATA,   r24

9:
	pop		ZH
	pop		ZL
	pop		r27
	pop		r26
	pop		r25
	pop		r23

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
