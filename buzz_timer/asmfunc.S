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
; Buzzer(46.875kHz) interrupt process
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
	ldi		r24, 1
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
	ldi		r24, 128
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
	.byte	145, 156, 145, 110,  96, 111, 145, 160, 143, 113
	.byte	100, 120, 144, 150, 130, 111, 113, 133, 144, 136
	.byte	121, 115, 123, 134, 139, 131, 122, 119, 125, 139
	.byte	136, 121, 112, 134, 142, 131, 120, 132, 138, 121
	.byte	107, 115

; 3/4 compress
	.byte	178, 211, 178,  74,  33,  76, 178, 223, 173,  82
	.byte	 45, 103, 176, 194, 133,  76,  83, 144, 176, 151
	.byte	197,  90, 112, 145, 162, 137, 110, 101, 118, 162
	.byte	151, 106,  80, 145, 169, 137, 103, 139, 158, 107
	.byte	 64,  88

; Normal
	.byte	194, 239, 194,  56,   1,  58, 195, 255, 188,  67
	.byte	 17,  94, 192, 216, 134,  58,  68, 149, 192, 159
	.byte	100,  77, 106, 150, 173, 141, 104,  92, 115, 173
	.byte	159,  98,  64, 150, 182, 140,  95, 143, 168, 100
	.byte	 43,  74

; 1/2 compress
	.byte	161, 184, 161,  92,  65,  93, 162, 192, 158,  98
	.byte	 73, 111, 169, 172, 131,  93,  98, 139, 160, 144
	.byte	114, 103, 117, 139, 151, 135, 116, 110, 122, 151
	.byte	144, 113,  96, 139, 155, 134, 112, 136, 148, 114
	.byte	 86, 101

; 1/4 compress
	.byte	145, 156, 145, 110,  96, 111, 145, 160, 143, 113
	.byte	100, 120, 144, 150, 130, 111, 113, 133, 144, 136
	.byte	121, 115, 123, 134, 139, 131, 122, 119, 125, 139
	.byte	136, 121, 112, 134, 142, 131, 120, 132, 138, 121
	.byte	107, 115
