;---------------------------------------------------------------------------;
;
;---------------------------------------------------------------------------;

.nolist
#include <avr/io.h>	// Include device specific definitions.
.list

#define A_DO	1
#define A_CS	4
#define B_CS	6

;---------------------------------------------------------------------------;
; Initialize USI
;
; void init_spi (void);

.global init_spi
.func init_spi
init_spi:
	ldi	r24, 0b00000001
	out	_SFR_IO_ADDR(USIPP), R24
	ldi	r24, 0b00001000			; Enable only SCK and DI. DO is controlled by software
	out	_SFR_IO_ADDR(USICR), r24
	sbi	_SFR_IO_ADDR(PORTA), A_DO	; MMC DI = H
	sbi	_SFR_IO_ADDR(PORTA), A_CS	; MMC CS = H
	sbi _SFR_IO_ADDR(PORTB), B_CS	; LED SS = H
	ret
.endfunc

;---------------------------------------------------------------------------;
; Transmit a byte (slow)
;
; void xmit_spi_slow (BYTE);

.global xmit_spi_slow
.func xmit_spi_slow
xmit_spi_slow:
	ldi	r25, 0b000100			; PA2(SCK)
	in	r22, _SFR_IO_ADDR(PORTA)

	ldi	r23, 8
1:	bst	r24, 7				;MMC DI = data bit to be sent
	bld	r22, A_DO			;
	out	_SFR_IO_ADDR(PORTA), r22	;
	lsl	r24				;/
	out	_SFR_IO_ADDR(PINA), r25		;SCK = H
	nop
	nop
	nop
	out	_SFR_IO_ADDR(PINA), r25		;SCK = L
	dec	r23				;while(--r23)
	brne	1b				;/

	sbi	_SFR_IO_ADDR(PORTA), A_DO	;MMC DI = H
	ret
.endfunc

;---------------------------------------------------------------------------;
; Transmit a byte
;
; void xmit_spi (BYTE);

.global xmit_spi
.func xmit_spi
xmit_spi:
	ldi	r25, 0b000100			; PA2(SCK)
	in	r22, _SFR_IO_ADDR(PORTA)

	ldi	r23, 8
1:	bst	r24, 7				;MMC DI = data bit to be sent
	bld	r22, A_DO			;
	out	_SFR_IO_ADDR(PORTA), r22	;
	lsl	r24				;/
	out	_SFR_IO_ADDR(PINA), r25		;SCK = H
	out	_SFR_IO_ADDR(PINA), r25		;SCK = L
	dec	r23				;while(--r23)
	brne	1b				;/

	sbi	_SFR_IO_ADDR(PORTA), A_DO	;MMC DI = H
	ret
.endfunc

;---------------------------------------------------------------------------;
; 100Hz interrupt process
;
; ISR(TIMER0_OVF_vect );


.global TIMER0_OVF_vect
.func TIMER0_OVF_vect
TIMER0_OVF_vect:
	push	r24
	in	r24, _SFR_IO_ADDR(SREG)
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
	cpi		r24, lo8(20000)
	sbci	r25, hi8(20000)
	brcs	1f
; then counter = 10000
	ldi		r24, lo8(10000)
	ldi		r25, hi8(10000)
	sts		counter+1, r25
	sts		counter, r24
1:
	pop	r25
	pop	r24				;
	out	_SFR_IO_ADDR(SREG), r24		;
	pop	r24				;/
	reti
.endfunc

