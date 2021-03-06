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
; Enable Analog Comparator
;
; void enable_ac (void)

.global enable_ac
.func enable_ac
enable_ac:
	cbi		_SFR_IO_ADDR(ACSRA), 3			; clear ACIE
	ldi		r24, 0b01010010					; ACBG=1, ACI=1, ACIS=10
	out		_SFR_IO_ADDR(ACSRA), r24
	sbi		_SFR_IO_ADDR(ACSRA), 3			; set ACIE
	ret
.endfunc


;---------------------------------------------------------------------------;
; Disable Analog Comparator
;
; void disable_ac (void)

.global disable_ac
.func disable_ac
disable_ac:
	cbi		_SFR_IO_ADDR(ACSRA), 3			; clear ACIE
	ldi		r24, 0b10000000					; ACD = 1
	out		_SFR_IO_ADDR(ACSRA), r24
	ret
.endfunc


;---------------------------------------------------------------------------;
; Analog Comparator  interrupt process
;
; ISR(ANA_COMP_vect);

.global ANA_COMP_vect
.func ANA_COMP_vect
ANA_COMP_vect:
	push	r24
	in		r24, _SFR_IO_ADDR(SREG)
	push	r24
	push	r25
	push	ZL
	push	ZH

	/* disable Comparator */
	rcall	disable_ac

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

	nop
	nop

	lsl		r24				;/
	out		_SFR_IO_ADDR(PINA), r25		;SCK = H

	nop
	nop
	nop
	nop

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
;	subi	r24, lo8(20000)
;	sbci	r25, hi8(20000)
;	brcs	1f
; then counter = 10000
;	ldi		r24, lo8(10000)
;	ldi		r25, hi8(10000)
;	sts		counter+1, r25
;	sts		counter, r24
1:

	lds		r25, ac_blank
	subi	r25, 1
	brcs	2f
	sts		ac_blank, r25
	brne	2f
	rcall	enable_ac
2:

; if buzz_shut != 0
	lds		r24, buzz_shut
	subi	r24, 1
	brcs	9f

	cbi		_SFR_IO_ADDR(PORTB), 6		; AMP CS=L

; if --buzz_shut == 0
	sts		buzz_shut, r24
	brne	9f

;	TCCR1A = TCCR1B = 0
	clr		r24
	out		_SFR_IO_ADDR(TCCR1A), r24
	out		_SFR_IO_ADDR(TCCR1B), r24

; Timer1 power down
	ldi		r24, 0b00001000
	out		_SFR_IO_ADDR(PRR), r24		; PRTIM1=1
9:

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
	in		r24, _SFR_IO_ADDR(GPIOR1)
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

	ldi		r24, 0b00000010
	out		_SFR_IO_ADDR(TIMSK), r24		; TOIE1 = 0, (TOIE0 = 1 yet)
	ldi		r24, 5
	sts		buzz_shut, r24
	ldi		r24, 128
	rjmp	8f

3:	sts		buzz_pos, r24

	lpm		r24, Z
8:
	out		_SFR_IO_ADDR(OCR1B), r24
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
