;---------------------------------------------------------------------------;
;
;---------------------------------------------------------------------------;

.nolist
#include <avr/io.h>	// Include device specific definitions.
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
	ldi	r23, 1			; 1
1:	dec	r23				; 1
	nop					; 1
	brne	1b			; 1
	sbiw	r24, 1		; 2
	brne	delay_us	; 2
	ret
.endfunc


;---------------------------------------------------------------------------;
; Buzzer(8MHz / 128 = 62.5kHz) interrupt process
;
; ISR(TIMER0_OVF_vect);

.global TIMER0_OVF_vect
.func TIMER0_OVF_vect
TIMER0_OVF_vect:
	push	r24
	in		r24, _SFR_IO_ADDR(SREG)
	push	r24
	push	r25
	push	ZL
	push	ZH

	sei

	ldi		r25, 42*3
	lds		ZH, buzz_time + 1
	lds		ZL, buzz_time
	sbiw	ZL, 1
	brcc	1f

	ldi		r25, 42*5
	rjmp	2f
1:
	sts		buzz_time+1, ZH
	sts		buzz_time, ZL
2:
; --buzz_cnt
	lds		r24, buzz_cnt
	subi	r24, 1
	sts		buzz_cnt, r24
	brcc	9f
	ldi		r24, 2			/* 62.5kHz / 4 = 15.625kHz */
	sts		buzz_cnt, r24

	lds		r24, buzz_pos
	mov		ZL, r24
	clr		ZH
	subi	ZL, lo8(-(buzz_wave))
	sbci	ZH, hi8(-(buzz_wave))

	subi	r24, -1
	cp		r24, r25
	brcs	3f

	ldi		r24, 42*2
	cpi		r25, 42*3
	breq	3f

	ldi		r24, 0
	sts		TIMSK0, r24		; TIMSK0 = 0
	ldi		r24, 5
	sts		buzz_shut, r24
	ldi		r24, 64
	rjmp	8f

3:	sts		buzz_pos, r24

	lpm		r24, Z
8:
	out		_SFR_IO_ADDR(OCR0B), r24
9:
	pop		ZH
	pop		ZL
	pop		r25
	pop		r24
	out		_SFR_IO_ADDR(SREG), r24
	pop		r24
	reti
.endfunc

buzz_wave:
; 1/4 compress
	.byte	 72, 78, 72, 55, 48, 55, 72, 80, 72, 56
	.byte	 50, 60, 72, 75, 65, 55, 56, 67, 72, 68
	.byte	 60, 58, 61, 67, 70, 66, 61, 59, 62, 70
	.byte	 68, 60, 56, 67, 71, 66, 60, 66, 69, 60
	.byte	 53, 57

; 3/4 compress
	.byte	 89,106, 89, 37, 16, 38, 89,112, 87, 41
	.byte	 22, 51, 88, 97, 66, 38, 41, 72, 88, 76
	.byte	 53, 45, 56, 72, 81, 69, 55, 50, 59, 81
	.byte	 76, 53, 40, 72, 84, 69, 52, 70, 79, 53
	.byte	 32, 44

; Normal
	.byte	 97,120, 97, 28,  0, 29, 98,128, 94, 33
	.byte	  8, 47, 96,108, 67, 29, 34, 75, 96, 80
	.byte	 50, 38, 53, 75, 87, 71, 52, 46, 57, 87
	.byte	 80, 49, 32, 75, 91, 70, 47, 72, 84, 50
	.byte	 21, 37

; 1/2 compress
	.byte	 81, 92, 81, 46, 32, 46, 81, 96, 79, 49
	.byte	 36, 55, 80, 86, 66, 46, 49, 69, 80, 72
	.byte	 57, 51, 58, 70, 75, 67, 58, 55, 61, 75
	.byte	 72, 56, 48, 70, 78, 67, 56, 68, 74, 57
	.byte	 43, 50

; 1/4 compress
	.byte	 72, 78, 72, 55, 48, 55, 72, 80, 72, 56
	.byte	 50, 60, 72, 75, 65, 55, 56, 67, 72, 68
	.byte	 60, 58, 61, 67, 70, 66, 61, 59, 62, 70
	.byte	 68, 60, 56, 67, 71, 66, 60, 66, 69, 60
	.byte	 53, 57

