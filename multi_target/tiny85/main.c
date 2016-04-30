#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#define TWI_SERVER_ADDRESS (0x10)
#define TWI_SLAVE_ADDRESS (0x20)

#define USI_SCL	2
#define USI_SDA 0

void delay_ms(uint16_t);
void delay_06us(void);
void delay_13us(void);

volatile uint8_t usi_twi_state;
volatile uint8_t usi_twi_slave_command;

#define STATE_NO_ACTIVE			(0)
#define STATE_WAIT_ADDRESS		(1)
#define STATE_WAIT_ADDRESS_ACK	(2)
#define STATE_WAIT_COMMAND		(3)
#define STATE_WAIT_COMMAND_ACK	(4)

void usi_twi_slave(void)
{
	/* Wait Start Condition Interrupt */
	PORTB |= (1<<USI_SCL) | (1<<USI_SDA);
	DDRB |=  (1<<USI_SCL);
	DDRB &= ~(1<<USI_SDA);

	USISR = 0b11110000;
	USICR = 0b10101000;

	usi_twi_state = STATE_NO_ACTIVE;
}

void usi_twi_send_ack(void)
{
	USIDR = 0;
	DDRB |= (1<<USI_SDA);
	USISR = 0b01111110;
}

ISR(USI_START_vect)
{
	DDRB &= ~(1<<USI_SDA);
	while ((PINB & (1<<USI_SCL)) & !(USISR & (1<<USIPF))) ;

	USICR = 0b11111000;
	USISR = 0b11110000;

	usi_twi_state = STATE_WAIT_ADDRESS;
}

ISR(USI_OVF_vect)
{
	switch (usi_twi_state) {
	case STATE_WAIT_ADDRESS:
		if ( (USIDR>>1) != TWI_SLAVE_ADDRESS ) {
			usi_twi_slave();
		}
		else if ( (USIDR & 0x01) == 1 ) {
			usi_twi_slave();
		}
		else {
			usi_twi_send_ack();
			usi_twi_state = STATE_WAIT_ADDRESS_ACK;
		}
		break;

	case STATE_WAIT_ADDRESS_ACK:
		DDRB &= ~(1<<USI_SDA);
		USISR = 0b01110000;
		usi_twi_state = STATE_WAIT_COMMAND;
		break;

	case STATE_WAIT_COMMAND:
		usi_twi_slave_command = USIDR;
		usi_twi_send_ack();
		usi_twi_state = STATE_WAIT_COMMAND_ACK;
		break;

	case STATE_WAIT_COMMAND_ACK:
		usi_twi_slave();
		break;
	}
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
	cli();
	USIDR = 0xFF;

	/* Clear USISUF and USIOSF before get SDA line */
	USICR = 0b00101010;
	USISR = 0xF0;

	PORTB |= _BV(USI_SDA) | _BV(USI_SCL);
	DDRB  |= _BV(USI_SDA) | _BV(USI_SCL);

	sei();

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
	USIDR = 0x80;

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
	PORTB = 0b00000111;
	DDRB  = 0b00011100;

//	/* PB1(AIN1) Digital Input Disable */
//	DIDR0 = 0b00000010;

	/* Analog Comparator Disable */
	ACSR  = 0b10000000;

	/* Powerdown : Timer1, Timer0, AD Converter */
	PRR   = _BV(PRTIM1) | _BV(PRTIM0) | _BV(PRADC);

	sei();

	do {
		PORTB &= ~(_BV(3) | _BV(4));
		usi_twi_slave_command = 0xFF;

		usi_twi_slave();

		do {} while (usi_twi_slave_command == 0xFF) ;

		if (usi_twi_slave_command != 0x10) {
			continue;
		}

		PORTB |= _BV(3);
		usi_twi_slave_command = 0xFF;
		do {} while (usi_twi_slave_command != 0xF0
		             && (PINB & _BV(1)));

		if (usi_twi_slave_command == 0xF0) {
			continue;
		}

		PORTB &= ~(_BV(3) | _BV(4));

		usi_twi_master();
	} while (1);
}