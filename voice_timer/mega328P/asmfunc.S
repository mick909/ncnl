.nolist
#include <avr/io.h>	// Include device specific definitions.
.list

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
	clr		r23
	ldi		r22, 16
2:
	lsl		r24
	rol		r25
	rol		r23
	cpi		r23, 10
	brcs	3f
	subi	r23, 10
	inc		r24
3:
	dec		r22
	brne	2b

	mov		XL,	r23
	add		XL, r23
	clr		XH
	subi	XL, lo8(-(seg_font))
	sbci	XH, hi8(-(seg_font))
	ld		r22, X+
	st		Z+, r22
	ld		r22, X
	st		Z+, r22

	subi	r18, 1
	brcs	5f
	brne	1b

; if num < 20000 -> set dot
	subi	r24, 20
	brcc	4f
	lds		r22, (seg_data+4)
	cbr		r22, 0b00001000
	sts		(seg_data+4), r22
	subi	r24, -20
4:
	cp		r24, r18
	brne	1b

	ldi		r22, 0xff
	st		Z+, r22
	st		Z, r22
5:
	ret

.endfunc


;---------------------------------------------------------------------------;
; Read SPI Data
;
; uint8_t read_spi(void)

.global read_spi
.func read_spi
read_spi:
	in		r24, _SFR_IO_ADDR(SPSR)
	sbrs	r24, SPIF
	rjmp	read_spi
	in		r24, _SFR_IO_ADDR(SPDR)
	ret

;	lds		r24, spi_rf
;	cpi		r24, 0
;	breq	1f
;
;	clr		r24
;	sts		spi_rf, r24
;	lds		r24, spi_rd
;	ret
;1:
;	ldi		r24, 1
;	cli
;	out		_SFR_IO_ADDR(SMCR), r24
;	sei
;	sleep
;	out		_SFR_IO_ADDR(SMCR), r1
;	rjmp	read_spi
.endfunc


;---------------------------------------------------------------------------;
; 4ms Timer interrupt process
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

	sei

;	PORTB &= ~0b00000010;
;	PORTD &= ~0b01100100;

;	PORTC |= 0b00111110;
;	PORTD |= 0b10011000;

;	PORTC &= *sdrp++;
;	PORTD &= *sdrp++;

	lds		ZL, sdrp
	lds		ZH, sdrp+1

	cbi		_SFR_IO_ADDR(PORTB), 1

	in		r24, _SFR_IO_ADDR(PORTC)
	ori		r24, 0b00111110
	ld		r25, Z+
	and		r24, r25
	out		_SFR_IO_ADDR(PORTC), r24

	in		r24, _SFR_IO_ADDR(PORTD)
	andi	r24, 0b10011011
	ori		r24, 0b10011000
	ld		r25, Z+
	and		r24, r25
	out		_SFR_IO_ADDR(PORTD), r24

	; 1 <- 2 <- 3 <- 0
	lds		r24, row
	subi	r24, 1
	brcc	2f

	; row = 0
	sbi		_SFR_IO_ADDR(PORTB), 1
	ldi		r24, 3
	rjmp	8f

2:
	brne	3f

	; row = 3
	sbi		_SFR_IO_ADDR(PORTD), 2
	ldi		ZL, lo8(seg_data)
	ldi		ZH, hi8(seg_data)
	rjmp	8f

3:
	cpi		r24, 1
	brne	4f

	; row = 2
	sbi		_SFR_IO_ADDR(PORTD), 5
	rjmp	8f

4:
	; row = 1
	sbi		_SFR_IO_ADDR(PORTD), 6

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
; Pin Change 0 interrupt process
;
; ISR(PCINT0_vect);

.global PCINT0_vect
.func PCINT0_vect
PCINT0_vect:
	push	r24
	in		r24, _SFR_IO_ADDR(SREG)
	push	r24

	sbic	_SFR_IO_ADDR(PINB), 2
	rjmp	1f

; Enable SPI
	ldi		r24, 0b10101011
	sts		PRR, r24
	ldi		r24, 0b01000000
	out		_SFR_IO_ADDR(SPCR), r24
	rjmp 2f

; Disable SPI
1:	clr		r24
	out		_SFR_IO_ADDR(SPCR), r24
	ldi		r24, 0b10101111
	sts		PRR, r24

2:
	pop		r24
	out		_SFR_IO_ADDR(SREG), r24
	pop		r24
	reti
.endfunc


;;---------------------------------------------------------------------------;
;; SPI Serial Transfer Complete interrupt process
;;
;; ISR(SPI_STC_vect);
;
;.global SPI_STC_vect
;.func SPI_STC_vect
;SPI_STC_vect:
;	push	r24
;	in		r24, _SFR_IO_ADDR(SREG)
;	push	r24
;
;	in		r24, _SFR_IO_ADDR(SPDR)
;	sts		spi_rd, r24
;	ldi		r24, 1
;	sts		spi_rf, r24
;
;	pop		r24
;	out		_SFR_IO_ADDR(SREG), r24
;	pop		r24
;	reti
;.endfunc
