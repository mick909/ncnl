;---------------------------------------------------------------------------;
;
;---------------------------------------------------------------------------;

.nolist
#include <avr/io.h>	// Include device specific definitions.
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
; Set 7Seg Display Font
;
; void set_display(uint16_t num)

.global set_display
.func set_display
set_display:
	ldi		ZL, lo8(seg_data)
	ldi		ZH, hi8(seg_data)

	ldi		r18, 3
1:
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
	subi	XL, lo8(-(seg_font))
	sbci	XH, hi8(-(seg_font))
	ld		r22, X
	st		Z+, r22

	subi	r18, 1
	brcs	4f
	brne	1b

	cp		r24, r18
	cpc		r25, r18
	brne	1b
	ldi		r22, 0x00
	st		Z, r22
4:
	lds		r22, (seg_data+2)
	sbr		r22, 0b00000010
	sts		(seg_data+2), r22

	ret
.endfunc


;---------------------------------------------------------------------------;
; 2ms/4ms Timer interrupt process
;
; ISR(TIMER0_COMPA_vect);

.global TIMER2_COMPA_vect
.func TIMER2_COMPA_vect
TIMER2_COMPA_vect:
.endfunc

.global TIMER0_COMPA_vect
.func TIMER0_COMPA_vect
TIMER0_COMPA_vect:
	push	r24
	in		r24, _SFR_IO_ADDR(SREG)
	push	r24
	push	r25
	push	ZL
	push	ZH

;  if (--count5 == 0) {
;    count5 = 5;
;    counter += 1;
;  }
	lds		r24, count5
	subi	r24, 1
	brne	1f

	lds		r24, counter
	lds		r25, counter+1
	subi	r24, -1
	sbci	r25, -1
	sts		counter, r24
	sts		counter+1, r25

	ldi		r24, 5
1:
	sts		count5, r24

	sei

	; Clear all digits (output H to common cathode)
	in		r24, _SFR_IO_ADDR(PORTB)
	ori		r24, 0b00111000
	out		_SFR_IO_ADDR(PORTB), r24

	; Set segment font data (output to each anode)
	lds		ZL, sdrp
	lds		ZH, sdrp+1

	ld		r24, Z+						; r24 = sedment font data
	out		_SFR_IO_ADDR(PORTD), r24

	; select one digit (output L to common cathode)
	lds		r24, row
	subi	r24, 1
	brcc	2f

	; row = 0 -> LSD
	cbi		_SFR_IO_ADDR(PORTB), 5
	ldi		r24, 2
	rjmp	8f

2:
	brne	3f

	; row = 1 -> MSD
	cbi		_SFR_IO_ADDR(PORTB), 3
	ldi		ZL, lo8(seg_data+1)
	ldi		ZH, hi8(seg_data+1)
	rjmp	8f

3:
	; row = 2 -> Center digit
	cbi		_SFR_IO_ADDR(PORTB), 4

8:
	sts		row, r24
	sts		sdrp, ZL
	sts		sdrp+1, ZH

	pop		ZH
	pop		ZL
	pop		r25
	pop		r24
	out		_SFR_IO_ADDR(SREG), r24
	pop		r24
	reti
.endfunc

;---------------------------------------------------------------------------;
.global update_qdec
.func update_qdec
update_qdec:
	LOAD	ZL, PINC
	lsr		ZL
	lsr		ZL
	andi	ZL, 0x03
	lds		r24, qdec0
	lsl		r24
	lsl		r24
	or		ZL, r24
	andi	ZL, 0x0f
	sts		qdec0, ZL

	clr		ZH
	subi	ZL, lo8(-(qdec_table))
	sbci	ZH, hi8(-(qdec_table))
	lpm		r24, Z

	lds		ZL, dec99
	add		ZL, r24
	cpi		ZL, 0xff
	brne	1f
	ldi		ZL, 19
1:
	cpi		ZL, 20
	brcs	2f
	clr		ZL
2:
	sts		dec99, ZL

;-------------------------------------
	LOAD	ZL, PINC
	andi	ZL, 0x03
	lds		r24, qdec1
	lsl		r24
	lsl		r24
	or		ZL, r24
	andi	ZL, 0x0f
	sts		qdec1, ZL

	clr		ZH
	subi	ZL, lo8(-(qdec_table))
	sbci	ZH, hi8(-(qdec_table))
	lpm		r24, Z

	lds		ZL, dec9900
	add		ZL, r24
	cpi		ZL, -1
	brne	1f
	ldi		ZL, 199
1:
	cpi		ZL, 200
	brcs	2f
	clr		ZL
2:
	sts		dec9900, ZL

;-------------------------------------
	lsr		ZL
	mov		r24, ZL
	clr		r25
	clr		ZH
	add		r24, ZL
	adc		r25, ZH
	add		r24, ZL
	adc		r25, ZH		; r25:r24 = ZL*3
	lsl		r24
	rol		r25
	lsl		r24
	rol		r25
	lsl		r24
	rol		r25			; r25:r24 = (ZL*3)*8
	add		r24, ZL
	adc		r25, ZH		; r25:r24 = (ZL*3)*8 + ZL
	lsl		r24
	rol		r25
	lsl		r24
	rol		r25			; r25:r24 = ZL * 100

	lds		ZH, dec99
	mov		ZL, ZH
	lsl		ZL
	lsl		ZL
	add		ZL, ZH
	andi	ZL, 0xfe
	clr		ZH
	add		r24, ZL
	adc		r25, ZH

	ret
.endfunc

qdec_table:
	.byte	  0,  0,  1, -1
	.byte	  0,  0,  1, -1
	.byte	 -1,  1,  0,  0
	.byte	 -1,  1,  0,  0

;---------------------------------------------------------------------------;
.global PCINT1_vect
.func PCINT1_vect
PCINT1_vect:
	push	r24
	LOAD	r24, SREG
	push	r24
	push	r25

	ldi		r24, _BV(PCINT11) | _BV(PCINT13)
	STORE	PCMSK1, r24

	push	ZL
	push	ZH
	rcall	update_qdec
	sts		par_count, r24
	sts		par_count+1, r25
	rcall	set_display
	pop		ZH
	pop		ZL

	pop		r25
	pop		r24
	STORE	SREG, r24
	pop		r24
	reti
.endfunc