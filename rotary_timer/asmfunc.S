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
	ldi		r22, 0xff
	st		Z, r22
4:
//	lds		r22, (seg_data+2)
//	cbr		r22, 0b00100000
//	sts		(seg_data+2), r22

	ret

.endfunc

;---------------------------------------------------------------------------;
; Enable Analog Comparator
;
; void enable_ac (void)

.global enable_ac
.func enable_ac
enable_ac:
	/* ACBG, ACI clear, H->L trigger */
	ldi		r25, 0b01010010
	out		_SFR_IO_ADDR(ACSR), r25		/* ACIE = 0 */

	ldi		r25, 0b00000000
	sts		ADCSRB, r25					/* ACME = 0 */

	ldi		r25, 0b01011010				/* ACIE = 1 */
	out		_SFR_IO_ADDR(ACSR), r25
	ret
.endfunc

;---------------------------------------------------------------------------;
; Analog Comparator  interrupt process
;
; ISR(ANALOG_COMP_vect);

.global ANALOG_COMP_vect
.func ANALOG_COMP_vect
ANALOG_COMP_vect:
	push	r24
	in		r24, _SFR_IO_ADDR(SREG)
	push	r24
	push	r25
	push	ZL
	push	ZH

	/* disable Comparator */
	ldi		r24, 0b11000010				/* ACD = 1, ACIE = 0 */
	out		_SFR_IO_ADDR(ACSR), r24

	lds		r24, counter
	lds		r25, counter+1

	sei

	lds		ZL, count_num
	inc		ZL
	sts		count_num, ZL
	add		ZL, ZL
	clr		ZH
	subi	ZL, lo8(-(counts))
	sbci	ZH, hi8(-(counts))
	st		Z+, r24
	st		Z, r25

	ldi		r24, 5
	sts		ac_blank, r24

	pop		ZH
	pop		ZL
	pop		r25
	pop		r24
	out		_SFR_IO_ADDR(SREG), r24
	pop		r24
	reti
.endfunc


;---------------------------------------------------------------------------;
; 2ms/4ms Timer interrupt process
;
; ISR(TIMER2_COMPA_vect);

.global TIMER2_COMPA_vect
.func TIMER2_COMPA_vect
TIMER2_COMPA_vect:
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

;  if (ac_blank && --ac_blank == 0) {
;    enable_ac();
;  }
	lds		r25, ac_blank
	subi	r25, 1
	brcs	1f
	sts		ac_blank, r25
	brne	1f
	rcall	enable_ac
1:
	sts		count5, r24

	sei

	lds		ZL, row
	subi	ZL, 1
	brcc	2f
	ldi		ZL, 3
2:
	sts		row, ZL
	clr		ZH
	subi	ZL, lo8(-(seg_data))
	sbci	ZH, hi8(-(seg_data))
	ld		r24, Z

	lds		r25, row
	cpi		r25, 2
	brne	3f
	lds		r25, dot
	or		r25, r25
	breq	3f
	andi	r24, 0b11011111
3:
	ldi		ZL, 0b01010000
	ldi		ZH,	0b00000001
	out		_SFR_IO_ADDR(SPCR), ZL
	out		_SFR_IO_ADDR(SPSR), ZH

	out		_SFR_IO_ADDR(SPDR), r24

4:
	in		r24, _SFR_IO_ADDR(SPSR)
	andi	r24, 0b10000000
	breq	4b

	clr		r24
	out		_SFR_IO_ADDR(SPCR), r24

	cbi		_SFR_IO_ADDR(PORTB), 3
	lds		r24, row
	cpi		r24, 3
	brne	9f
	sbi		_SFR_IO_ADDR(PORTB), 3
9:	sbi		_SFR_IO_ADDR(PINB), 4
	sbi		_SFR_IO_ADDR(PINB), 4

	pop		ZH
	pop		ZL
	pop		r25
	pop		r24
	out		_SFR_IO_ADDR(SREG), r24
	pop		r24
	reti
.endfunc
