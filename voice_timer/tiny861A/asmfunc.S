;---------------------------------------------------------------------------;
;
;---------------------------------------------------------------------------;

.nolist
#include <avr/io.h>	// Include device specific definitions.
.list

#define A_DO	1
#define A_MMICS	3
#define A_LEDSS	4

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

;---------------------------------------------------------------------------;
; Initialize USI
;
; void init_spi (void);

.global init_spi
.func init_spi
init_spi:
	ldi		r24, 0b00000001
	out		_SFR_IO_ADDR(USIPP), R24
	ldi		r24, 0b00001000			; Enable only SCK and DI. DO is controlled by software
	out		_SFR_IO_ADDR(USICR), r24
	sbi		_SFR_IO_ADDR(PORTA), A_DO		; MMC DI = H
	sbi		_SFR_IO_ADDR(PORTA), A_MMICS	; MMC CS = H
	sbi 	_SFR_IO_ADDR(PORTA), A_LEDSS	; LED SS = H
	ret
.endfunc

;---------------------------------------------------------------------------;
; Transmit a byte (slow)
;
; void xmit_spi_slow (BYTE);

.global xmit_spi_slow
.func xmit_spi_slow
xmit_spi_slow:
	ldi		r25, 0b000100			; PA2(SCK)
	in		r22, _SFR_IO_ADDR(PORTA)

	ldi		r23, 8
1:	bst		r24, 7				;data bit to be sent
	bld		r22, A_DO			;
	out		_SFR_IO_ADDR(PORTA), r22	;

	.rept 6
	nop
	.endr

	lsl		r24				;/
	out		_SFR_IO_ADDR(PINA), r25		;SCK = H

	.rept 6
	nop
	.endr

	out		_SFR_IO_ADDR(PINA), r25		;SCK = L

	dec		r23				;while(--r23)
	brne	1b				;/

	sbi		_SFR_IO_ADDR(PORTA), A_DO	;MMC DI = H

	ret
.endfunc

;---------------------------------------------------------------------------;
; Transmit a byte
;
; void xmit_spi (BYTE);

.global xmit_spi
.func xmit_spi
xmit_spi:
	ldi		r25, 0b000100			; PA2(SCK)
	in		r22, _SFR_IO_ADDR(PORTA)

	ldi		r23, 8
1:	bst		r24, 7				;MMC DI = data bit to be sent
	bld		r22, A_DO			;
	out		_SFR_IO_ADDR(PORTA), r22	;
	lsl		r24				;/
	out		_SFR_IO_ADDR(PINA), r25		;SCK = H
	out		_SFR_IO_ADDR(PINA), r25		;SCK = L
	dec		r23				;while(--r23)
	brne	1b				;/

	sbi		_SFR_IO_ADDR(PORTA), A_DO	;MMC DI = H
	ret
.endfunc

;---------------------------------------------------------------------------;
; 100Hz interrupt process
;
; ISR(TIMER0_OVF_vect);

.global TIMER0_OVF_vect
.func TIMER0_OVF_vect
TIMER0_OVF_vect:
	push	r24
	in		r24, _SFR_IO_ADDR(SREG)
	push	r24
	push	r25
	ldi		r25, hi8(-(F_CPU / 64 / 100))
	ldi		r24, lo8(-(F_CPU / 64 / 100))
	out		_SFR_IO_ADDR(TCNT0H), r25
	out		_SFR_IO_ADDR(TCNT0L), r24		; within 60 clock

; ++counter
	lds		r24, counter
	lds		r25, counter+1
	adiw	r24, 1
	sts		counter+1, r25
	sts		counter, r24
; if counter >= 20000
	subi	r24, lo8(20000)
	sbci	r25, hi8(20000)
	brcs	1f
; then counter = 10000
	ldi		r24, lo8(10000)
	ldi		r25, hi8(10000)
	sts		counter+1, r25
	sts		counter, r24
1:
	pop		r25
	pop		r24				;
	out		_SFR_IO_ADDR(SREG), r24		;
	pop		r24				;/
	reti
.endfunc

;---------------------------------------------------------------------------;
; Buzzer(46.875kHz) interrupt process
;
; ISR(TIMER1_OVF_vect);

.global TIMER1_OVF_vect
.func TIMER1_OVF_vect
TIMER1_OVF_vect:
	push	r24
	in		r24, _SFR_IO_ADDR(SREG)
	push	r24
	push	ZL
	push	ZH

; --buzz_cnt
	lds		r24, buzz_cnt
	subi	r24, 1
	sts		buzz_cnt, r24
	brcc	9f
	ldi		r24, 2
	sts		buzz_cnt, r24

	ldi		ZH, hi8(buzz_wave)
	ldi		ZL, lo8(buzz_wave)
	lds		r24, buzz_pos
	add		ZL, r24

	subi	r24, -1
	cpi		r24, 42
	brcs	1f
	ldi		r24, 0
1:	sts		buzz_pos, r24

	lpm		r24, Z

	out		_SFR_IO_ADDR(OCR1B), r24
9:
	pop		ZH
	pop		ZL
	pop		r24
	out		_SFR_IO_ADDR(SREG), r24
	pop		r24
	reti
.endfunc

buzz_wave:
//	.byte	128, 191, 255, 0, 64, 0
//	.byte	128, 154, 255, 205, 230, 255, 26, 51, 30, 102

	.byte	194, 239, 194,  56,   1,  58, 195, 255, 188,  67
	.byte	 17,  94, 192, 216, 134,  58,  68, 149, 192, 159
	.byte	100,  77, 106, 150, 173, 141, 104,  92, 115, 173
	.byte	159,  98,  64, 150, 182, 140,  95, 143, 168, 100
	.byte	 43,  74
