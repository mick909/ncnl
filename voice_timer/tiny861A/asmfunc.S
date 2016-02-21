;---------------------------------------------------------------------------;
;
;---------------------------------------------------------------------------;

.nolist
#include <avr/io.h>	// Include device specific definitions.
.list

#define	_FLAGS	_SFR_IO_ADDR(GPIOR0)

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

.global delay_us
.func delay_us
delay_us:
	ldi	r23, 2			; 1
1:	dec	r23				; 1 1
	nop					; 1 1
	brne	1b			; 2 1
	sbiw	r24, 1		; 2
	brne	delay_us	; 2
	ret
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
; Control CS
;
; void select_mmc (void);
; void deselect_mmc (void);
; void select_led (void);
; void deselect_led (void);

.global select_mmc
.func select_mmc
select_mmc:
	cbi	_SFR_IO_ADDR(PORTA), A_MMICS	; CS = L
	ret
.endfunc

.global deselect_mmc
.func deselect_mmc
deselect_mmc:
	sbi	_SFR_IO_ADDR(PORTA), A_MMICS	; CS = H
	ret
.endfunc

.global select_led
.func select_led
select_led:
	cbi	_SFR_IO_ADDR(PORTA), A_LEDSS	; SS = L
	ret
.endfunc

.global deselect_led
.func deselect_led
deselect_led:
	sbi	_SFR_IO_ADDR(PORTA), A_LEDSS	; SS = H
	ret
.endfunc


;---------------------------------------------------------------------------;
; Receive a byte
;
; BYTE rcv_spi (void);

.global rcv_spi
.func rcv_spi
rcv_spi:
	ldi	r24, 0b000100			;PA2(SCK)
	.rept 16				;Toggle SCK 16 times
	out	_SFR_IO_ADDR(PINA), r24		;
	.endr					;/
	nop					;Read shift register
	in	r24, _SFR_IO_ADDR(USIDR)	;/
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
; Receive a byte (slow)
;
; BYTE rcv_spi_slow (void);

.global rcv_spi_slow
.func rcv_spi_slow
rcv_spi_slow:
	ldi	r24, 0b000100			;PA2(SCK)
	nop
	nop
	nop
	nop
	nop
	nop

	.rept 16				;Toggle SCK 16 times
	out	_SFR_IO_ADDR(PINA), r24		;
	nop
	nop
	nop
	nop
	nop
	nop
	.endr					;/

	nop					;Read shift register
	in	r24, _SFR_IO_ADDR(USIDR)	;/
	ret
.endfunc


;---------------------------------------------------------------------------;;---------------------------------------------------------------------------;
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


; Read and forward a partial data block
;
; void read_blk_part (void*, WORD, WORD);

.global read_blk_part
.func read_blk_part
read_blk_part:
	movw	XL, r24			;X = R25:R24 (destination memory address)
	movw	ZL, r22			;Z = R23:R22 (byte offset in the sector)

	ldi		r18, lo8(514)		;R19:R18 = 514, Number of bytes to receive
	ldi		r19, hi8(514)		;/
	sub		r18, ZL			;R19:R18 -= Z
	sbc		r19, ZH			;/
	sub		r18, r20		;R19:R18 -= R21:R20
	sbc		r19, r21		;/

	; Skip leading data bytes
	ldi		r24, 0b000100		;PA2(SCK)
1:	sbiw	ZL, 1			;Skip leading data...
	brcs	2f			;
	.rept 16			;Discard a byte on SPI
	out		_SFR_IO_ADDR(PINA), r24	;
	.endr				;/
	rjmp	1b			;
2:	sbiw	XL, 0			;Destination?
	breq	fb_wave

fb_mem:	; Store the data bytes to the memory
	rcall	rcv_spi			;do
	st		X+, r24			; *X++ = rcv_spi()
	subi	r20, 1			;while (--r21:r20)
	sbci	r21, 0			;
	brne	fb_mem			;/
	rjmp	fb_exit

fb_wave: ; Store the data bytes to the audio FIFO
	sbic	_FLAGS, 4		;if (16bit data) R21:R20 /= 2;
	lsr		r21			;
	sbic	_FLAGS, 4		;
	ror		r20			;/
	sbic	_FLAGS, 1		;if (Stereo data) R21:R20 /= 2;
	lsr		r21			;
	sbic	_FLAGS, 1		;
	ror		r20			;/
	lds		r22, FifoWi		;r22 = FIFO write index

3:	ldi		XL, lo8(Buff)		;X = Buff + R22
	ldi		XH, hi8(Buff)		;
	add		XL, r22			;
	adc		XH, r1			;/
4:	lds		r24, FifoCt		;wait while FIFO full
	cpi		r24, 252		;
	brcc	4b			;/

; Single output
	sbic	_FLAGS, 4		;Get L-ch/Mono data into ZL
	rcall	rcv_spi			;
	rcall	rcv_spi			;
	sbic	_FLAGS, 4		;
	subi	r24, 0x80		;
	mov		ZL, r24			;/
	sbis	_FLAGS, 1		;if Mono data, do not process R-ch data
	rjmp	9f			;/
	sbic	_FLAGS, 4		;Get R-ch data
	rcall	rcv_spi			;
	rcall	rcv_spi			;
	sbic	_FLAGS, 4		;
	subi	r24, 0x80		;/
	add		ZL, r24			;ZL = (ZL + R-ch) / 2
	ror		ZL			;/
9:	st		X+, ZL			;Store ZL into FIFO
	cli				;
	lds		r24, FifoCt		;
	inc		r24			;
	sts		FifoCt, r24		;
	sei				;
	inc		r22			;/

	subi	r20, lo8(1)		;while(--R21:R20)
	sbci	r21, hi8(1)		;
	brne	3b			;/
	sts		FifoWi, r22		;Save FIFO write index

fb_exit:	/* Diacard trailing data bytes and CRC */
	ldi		r24, 0b000100		;PA2(SCK)
1:	.rept 16			;Discard a byte on SPI
	out		_SFR_IO_ADDR(PINA), r24	;
	.endr				;/
	subi	r18, lo8(1)		;Repeat r19:r18 times
	sbci	r19, hi8(1)		;
	brne	1b			;/

	ret
.endfunc


;---------------------------------------------------------------------------;
; Audio sampling interrupt process
;
; ISR(TIMER0_COMPA_vect);

.global TIMER0_COMPA_vect
.func TIMER0_COMPA_vect
TIMER0_COMPA_vect:
	push	r24				;Save regs.
	in		r24, _SFR_IO_ADDR(SREG)		;
	push	r24				;
	push	ZL				;
	push	ZH				;/

	lds		ZL, FifoRi			;Get FIFO read index
	clr		ZH				;Z = pointer to the top of FIFO
	subi	ZL, lo8(-(Buff))		;
	sbci	ZH, hi8(-(Buff))		;/
	lds		r24, FifoCt			;Load FIFO data counter

; Single output
	subi	r24, 1				;Check availability of the sampling data
	brcs	9f				;/
	sts		FifoCt, r24			;Save FIFO data counter
	ld		r24, Z+				;Send data to PWM
	out		_SFR_IO_ADDR(OCR1B), r24	;/

	subi	ZL, lo8(Buff)			;Save FIFO read index
	sts		FifoRi, ZL			;/

9:	pop		ZH				;Restore regs.
	pop		ZL				;
	pop		r24				;
	out		_SFR_IO_ADDR(SREG), r24		;
	pop		r24				;/
	reti
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
