.nolist
#include <avr/io.h>
.list

;---------------------------------------------------------------------------;
; delay_ms
;
; void delay_ms (uint16_t ms);

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

.global delay_us
.func delay_us
delay_us:
	ldi		r23, 1		; 1
1:	dec		r23			; 1
	nop					; 1
	brne	1b			; 1
	sbiw	r24, 1		; 2
	brne	delay_us	; 2
	ret
.endfunc


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


;---------------------------------------------------------------------------;
; ISR(TCC4_OVF_vect)
;   10ms interval (counter is updated at TCC5)

.global TCC4_OVF_vect
.func TCC4_OVF_vect
TCC4_OVF_vect:
	push	r24
	in		r24, _SFR_IO_ADDR(SREG)
	push	r24
	push	r25

	push	r22
	push	XL
	push	XH

	ldi		r24, TC4_OVFIF_bm
	sts		TCC4_INTFLAGS, r24

	lds		r24, Timer1
	subi	r24, 1
	brcs	1f
	sts		Timer1, r24
1:
	lds		r24, Timer2
	subi	r24, 1
	brcs	2f
	sts		Timer2, r24
2:

	lds		r24, TCC5_CNT
	lds		r25, TCC5_CNT+1
	sts		count+1, r25
	sts		count, r24

	; DIG1
	clr		XL
	ldi		r22, 16
2:
	lsl		r24
	rol		r25
	rol		XL
	cpi		XL, 10
	brcs	3f
	subi	XL, 10
	inc		r24
3:
	dec		r22
	brne	2b

	clr		XH
	subi	XL, lo8(-(digit))
	sbci	XH, hi8(-(digit))
	ld		r22, X
	sts		digits, r22

	; DIG2
	clr		XL
	ldi		r22, 16
2:
	lsl		r24
	rol		r25
	rol		XL
	cpi		XL, 10
	brcs	3f
	subi	XL, 10
	inc		r24
3:
	dec		r22
	brne	2b

	clr		XH
	subi	XL, lo8(-(digit))
	sbci	XH, hi8(-(digit))
	ld		r22, X
	sts		digits+1, r22

	; DIG3
	clr		XL
	ldi		r22, 16
2:
	lsl		r24
	rol		r25
	rol		XL
	cpi		XL, 10
	brcs	3f
	subi	XL, 10
	inc		r24
3:
	dec		r22
	brne	2b

	clr		XH
	subi	XL, lo8(-(digit))
	sbci	XH, hi8(-(digit))
	ld		r22, X
	andi	r22, 0x7f
	sts		digits+2, r22

	; DIG4
	clr		XL
	cp		r24, XL
	cpc		r25, XL
	brne	1f
	ldi		r22, 0xff
	sts		digits+3, r22
	rjmp	9f

1:
	ldi		r22, 16
2:
	lsl		r24
	rol		r25
	rol		XL
	cpi		XL, 10
	brcs	3f
	subi	XL, 10
	inc		r24
3:
	dec		r22
	brne	2b

	clr		XH
	subi	XL, lo8(-(digit))
	sbci	XH, hi8(-(digit))
	ld		r22, X
	sts		digits+3, r22

9:
	pop		XH
	pop		XL
	pop		r22

	pop		r25
	pop		r24
	out		_SFR_IO_ADDR(SREG), r24
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
	in		r24, _SFR_IO_ADDR(SREG)
	push	r24

	push	r23
	push	r25
	push	r26
	push	r27
	push	ZL
	push	ZH

	ldi		r24, TC5_OVFIF_bm;
	sts		TCD5_INTFLAGS, r24

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
	sts		DACA_CH0DATA+1, r25
	sts		DACA_CH0DATA,   r24
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
	sts		DACA_CH0DATA+1, r25
	sts		DACA_CH0DATA,   r24
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

	sts		DACA_CH0DATA+1, r25
	sts		DACA_CH0DATA,   r24
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

	ld		r26, Z+
	ld		r27, Z+
	subi	r27, 0x80
	lsr		r27
	ror		r26

	add		r24, r26
	adc		r25, r27
	andi	r24, 0xf0

	sts		DACA_CH0DATA+1, r25
	sts		DACA_CH0DATA,   r24

9:
	pop		ZH
	pop		ZL
	pop		r27
	pop		r26
	pop		r25
	pop		r23

	pop		r24
	out		_SFR_IO_ADDR(SREG), r24
	pop		r24
	reti
.endfunc

;---------------------------------------------------------------------------;
; ISR(TCC5_OVF_vect)
;   10ms counter wraparound at 600.00 sec

.global TCC5_OVF_vect
.func TCC5_OVF_vect
TCC5_OVF_vect:
	push	r24
	in		r24, _SFR_IO_ADDR(SREG)
	push	r24
	push	r25

	ldi		r24, TC5_OVFIF_bm
	sts		TCC5_INTFLAGS, r24

	lds		r24, TCC5_CNT
	lds		r25, TCC5_CNT+1

	subi	r24, lo8(-(10000))
	sbci	r25, hi8(-(10000))

	sts		TCC5_CNT+1, r25
	sts		TCC5_CNT, r24

	pop		r25
	pop		r24
	out		_SFR_IO_ADDR(SREG), r24
	pop		r24
	reti
.endfunc


;---------------------------------------------------------------------------;
; ISR(TCC4_CCx_vect)
;   update LED dynamic drive per 2.5 ms x 4 digit

.global TCC4_CCA_vect
.func TCC4_CCA_vect
TCC4_CCA_vect:
	push	r24
	in		r24, _SFR_IO_ADDR(SREG)
	push	r24

	ldi		r24, TC4_CCDIF_bm | TC4_CCCIF_bm | TC4_CCBIF_bm | TC4_CCAIF_bm;
	sts		TCC4_INTFLAGS, r24

	ldi		r24, PIN7_bm | PIN6_bm | PIN5_bm | PIN4_bm
	sts		PORTA_OUTCLR, r24
	lds		r24, digits
	sts		PORTC_OUT, r24
	ldi		r24, PIN4_bm
	sts		PORTA_OUTSET, r24

	pop		r24
	out		_SFR_IO_ADDR(SREG), r24
	pop		r24
	reti
.endfunc

.global TCC4_CCB_vect
.func TCC4_CCB_vect
TCC4_CCB_vect:
	push	r24
	in		r24, _SFR_IO_ADDR(SREG)
	push	r24

	ldi		r24, TC4_CCDIF_bm | TC4_CCCIF_bm | TC4_CCBIF_bm | TC4_CCAIF_bm;
	sts		TCC4_INTFLAGS, r24

	ldi		r24, PIN7_bm | PIN6_bm | PIN5_bm | PIN4_bm
	sts		PORTA_OUTCLR, r24
	lds		r24, digits+1
	sts		PORTC_OUT, r24
	ldi		r24, PIN5_bm
	sts		PORTA_OUTSET, r24

	pop		r24
	out		_SFR_IO_ADDR(SREG), r24
	pop		r24
	reti
.endfunc

.global TCC4_CCC_vect
.func TCC4_CCC_vect
TCC4_CCC_vect:
	push	r24
	in		r24, _SFR_IO_ADDR(SREG)
	push	r24

	ldi		r24, TC4_CCDIF_bm | TC4_CCCIF_bm | TC4_CCBIF_bm | TC4_CCAIF_bm;
	sts		TCC4_INTFLAGS, r24

	ldi		r24, PIN7_bm | PIN6_bm | PIN5_bm | PIN4_bm
	sts		PORTA_OUTCLR, r24
	lds		r24, digits+2
	sts		PORTC_OUT, r24
	ldi		r24, PIN6_bm
	sts		PORTA_OUTSET, r24

	pop		r24
	out		_SFR_IO_ADDR(SREG), r24
	pop		r24
	reti
.endfunc

.global TCC4_CCD_vect
.func TCC4_CCD_vect
TCC4_CCD_vect:
	push	r24
	in		r24, _SFR_IO_ADDR(SREG)
	push	r24

	ldi		r24, TC4_CCDIF_bm | TC4_CCCIF_bm | TC4_CCBIF_bm | TC4_CCAIF_bm;
	sts		TCC4_INTFLAGS, r24

	ldi		r24, PIN7_bm | PIN6_bm | PIN5_bm | PIN4_bm
	sts		PORTA_OUTCLR, r24
	lds		r24, digits+3
	sts		PORTC_OUT, r24
	ldi		r24, PIN7_bm
	sts		PORTA_OUTSET, r24

	pop		r24
	out		_SFR_IO_ADDR(SREG), r24
	pop		r24
	reti
.endfunc
