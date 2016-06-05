#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define TWI_SERVER_ADDRESS (0x10)
#define TWI_SLAVE_ADDRESS (0x20)

#define USI_SCL	2
#define USI_SDA 0

void delay_ms(uint16_t);
void delay_06us(void);
void delay_13us(void);

void usi_twi_slave(void)
{
	uint8_t recd;

	/* Wait Start Condition */
		DDRB &= ~(1<<USI_SDA);
		USICR = 0b00101000;
		USISR = 0xF0;
		while (!(USISR & _BV(USISIF))) ;

	/* Wait Address */
		DDRB &= ~(1<<USI_SDA);
		while ((PINB & (1<<USI_SCL)) & !(USISR & (1<<USIPF))) ;

		USICR = 0b00111000;
		USISR = 0xF0;
		while (!(USISR & _BV(USIOIF))) ;
		if ( (USIDR>>1) != TWI_SLAVE_ADDRESS) {
			return;
		}
		if ( (USIDR & 0x01) == 1) {
			return;
		}

	/* Send Ack */
		USIDR = 0;
		DDRB |= (1<<USI_SDA);
		USISR = 0b01111110;
		while (!(USISR & _BV(USIOIF))) ;

	/* Receive Data */
		DDRB &= ~(1<<USI_SDA);
		USISR = 0b01110000;
		while (!(USISR & _BV(USIOIF))) ;

		recd = USIDR;

	/* Send Ack */
		USIDR = 0;
		DDRB |= (1<<USI_SDA);
		USISR = 0b01111110;
		while (!(USISR & _BV(USIOIF))) ;

	/* Wait Repeated Start */
		DDRB &= ~(1<<USI_SDA);
		USICR = 0b00101000;
		USISR = 0xF0;
		while (!(USISR & _BV(USISIF))) ;

	/* Wait Address */
		DDRB &= ~(1<<USI_SDA);
		while ((PINB & (1<<USI_SCL)) & !(USISR & (1<<USIPF))) ;
		USICR = 0b00111000;
		USISR = 0xF0;
		while (!(USISR & _BV(USIOIF))) ;
		if ( (USIDR>>1) != TWI_SLAVE_ADDRESS) {
			return;
		}
		if ( (USIDR & 0x01) == 0) {
			return;
		}

	/* Send Ack */
		USIDR = 0;
		DDRB |= (1<<USI_SDA);
		USISR = 0b01111110;
		while (!(USISR & _BV(USIOIF))) ;

	/* Send Data */
		USIDR = recd + 1;
		DDRB |= (1<<USI_SDA);
		USISR = 0b01110000;
		while (!(USISR & _BV(USIOIF))) ;

	/* Wait Ack (or Nack) */
		DDRB &= ~(1<<USI_SDA);
		USISR = 0b01111110;
		while (!(USISR & _BV(USIOIF))) ;
}

static
uint8_t usi_twi_check_condition(void)
{
	if ( USISR & (_BV(USISIF) | _BV(USIPF) | _BV(USIDC)) ) {
		return 0;
	}
	return 1;
}

uint8_t usi_twi_master_transfer(void)
{
	uint8_t cr = 0b00101011;
	uint8_t data;

	do {
		delay_13us();
		USICR = cr;		/* SCL = H */
		do {} while ( !(PINB & (1<<USI_SCL)));		/* Wait SCL H */
		delay_06us();
		USICR = cr;		/* SCL = L */
	} while ( !(USISR & (1<<USIOIF)) );

	delay_13us();

	data = USIDR;
	USIDR = 0xFF;
	DDRB |= _BV(USI_SDA);

	return data;
}

uint8_t usi_twi_master_stop(void)
{
	PORTB &= ~(_BV(USI_SDA));
	PORTB |= _BV(USI_SCL);
	do {} while (!(PINB & (_BV(USI_SCL))));
	delay_06us();
	PORTB |= _BV(USI_SDA);
	delay_13us();

	return USISR & (1<<USIPF);
}

void usi_twi_master(void)
{
	PORTB |= _BV(USI_SDA) | _BV(USI_SCL);
	DDRB  |= _BV(USI_SDA) | _BV(USI_SCL);

	USIDR = 0xFF;

	USICR = 0b00101010;
	USISR = 0xF0;

	if (!usi_twi_check_condition()) {
		return;
	}

	PORTB |= _BV(USI_SCL);		/* SCL = H */
	while (!(PINB & (1<<USI_SCL))) ;
	delay_06us();

	/* Make Start Condition */
	PORTB &= ~(_BV(USI_SDA));	/* SDA = L */
	delay_06us();
	PORTB &= ~(_BV(USI_SCL));	/* SCL = L */
	PORTB |= _BV(USI_SDA);	/* Release SDA */

	/* Check Start Condition */
	if (!(USISR & (1<<USISIF))) {
		return;
	}

	/* Write Address */
	PORTB &= ~(_BV(USI_SCL));	/* SCL = L */
	USIDR = (TWI_SERVER_ADDRESS << 1);

	USISR = 0xF0;				/* Transfer 8bit */
	usi_twi_master_transfer();

	/* Read ack bit */
	DDRB &= ~(_BV(USI_SDA));
	USISR = 0xFE;				/* Transfer 1bit */
	if (usi_twi_master_transfer() & 0x01) {
		/* Got NACK */
		return;
	}
	/* Got ACK */

	/* Write Data */
	PORTB &= ~(_BV(USI_SCL));	/* SCL = L */
	USIDR = 'x';

	USISR = 0xF0;				/* Transfer 8bit */
	usi_twi_master_transfer();

	/* Read ack bit */
	DDRB &= ~(_BV(USI_SDA));
	USISR = 0xFE;				/* Transfer 1bit */
	if (usi_twi_master_transfer() & 0x01) {
		/* Got NACK */
		return;
	}
	/* Got ACK */

	usi_twi_master_stop();
}

int main()
{
	MCUSR = 0;

	/*
	  PB0 : SDA
	  PB1 : AIN1
	  PB2 : SCL
	  PB3 : Output
	  PB4 : Output
	*/
	PORTB = 0b00000101;
	DDRB  = 0b00011100;

	/* PB1(AIN1) Digital Input Disable */
	DIDR0 = 0b00000010;

	/* Analog Comparator Disable */
	ACSR  = 0b10000000;

	/* Powerdown : Timer1, Timer0, AD Converter */
	PRR   = _BV(PRTIM1) | _BV(PRTIM0) | _BV(PRADC);

	do {
		usi_twi_slave();

		delay_ms(100);

		usi_twi_master();

		do {} while (1);
	} while (1);
}