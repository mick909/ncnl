#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

void delay_ms (uint16_t ms);

void usart_init(void);
void usart_tx(uint8_t c);
void usart_tx_str(const char *b);
uint8_t usart_rx_empty();
uint8_t usart_rx(void);

#define TWI_SLAVE_ADDRESS (0x20)

#define TWI_TWBR	(2)
#define TWI_TWPS	(0)

/****************************************************************************
  TWI State codes
****************************************************************************/
// General TWI Master status codes
#define TWI_START                  0x08  // START has been transmitted
#define TWI_REP_START              0x10  // Repeated START has been transmitted
#define TWI_ARB_LOST               0x38  // Arbitration lost

// TWI Master Transmitter status codes
#define TWI_MTX_ADR_ACK            0x18  // SLA+W has been transmitted and ACK received
#define TWI_MTX_ADR_NACK           0x20  // SLA+W has been transmitted and NACK received
#define TWI_MTX_DATA_ACK           0x28  // Data byte has been transmitted and ACK received
#define TWI_MTX_DATA_NACK          0x30  // Data byte has been transmitted and NACK received

// TWI Master Receiver status codes
#define TWI_MRX_ADR_ACK            0x40  // SLA+R has been transmitted and ACK received
#define TWI_MRX_ADR_NACK           0x48  // SLA+R has been transmitted and NACK received
#define TWI_MRX_DATA_ACK           0x50  // Data byte has been received and ACK transmitted
#define TWI_MRX_DATA_NACK          0x58  // Data byte has been received and NACK transmitted

// TWI Slave Transmitter status codes
#define TWI_STX_ADR_ACK            0xA8  // Own SLA+R has been received; ACK has been returned
#define TWI_STX_ADR_ACK_M_ARB_LOST 0xB0  // Arbitration lost in SLA+R/W as Master; own SLA+R has been received; ACK has been returned
#define TWI_STX_DATA_ACK           0xB8  // Data byte in TWDR has been transmitted; ACK has been received
#define TWI_STX_DATA_NACK          0xC0  // Data byte in TWDR has been transmitted; NOT ACK has been received
#define TWI_STX_DATA_ACK_LAST_BYTE 0xC8  // Last data byte in TWDR has been transmitted (TWEA = “0”); ACK has been received

// TWI Slave Receiver status codes
#define TWI_SRX_ADR_ACK            0x60  // Own SLA+W has been received ACK has been returned
#define TWI_SRX_ADR_ACK_M_ARB_LOST 0x68  // Arbitration lost in SLA+R/W as Master; own SLA+W has been received; ACK has been returned
#define TWI_SRX_GEN_ACK            0x70  // General call address has been received; ACK has been returned
#define TWI_SRX_GEN_ACK_M_ARB_LOST 0x78  // Arbitration lost in SLA+R/W as Master; General call address has been received; ACK has been returned
#define TWI_SRX_ADR_DATA_ACK       0x80  // Previously addressed with own SLA+W; data has been received; ACK has been returned
#define TWI_SRX_ADR_DATA_NACK      0x88  // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
#define TWI_SRX_GEN_DATA_ACK       0x90  // Previously addressed with general call; data has been received; ACK has been returned
#define TWI_SRX_GEN_DATA_NACK      0x98  // Previously addressed with general call; data has been received; NOT ACK has been returned
#define TWI_SRX_STOP_RESTART       0xA0  // A STOP condition or repeated START condition has been received while still addressed as Slave


void twi_master(void)
{
	/***************************************************************/
		TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
		while (! (TWCR & (1<<TWINT))) ;
		if ((TWSR & 0xF8) != TWI_START) {
			usart_tx_str("Error: can't send start condition\r\n");

			uint8_t sr = TWSR & 0xF8;

			char c1 = "0123456789ABCDEF"[(sr>>4)];
			char c2 = "0123456789ABCDEF"[(sr & 0x0F)];
			usart_tx(c1);
			usart_tx(c2);
			usart_tx_str("\r\n");
			return;
		}

	/***************************************************************/
		usart_tx_str("Send address\r\n");
		TWDR = (TWI_SLAVE_ADDRESS << 1) | 0;
		TWCR = _BV(TWINT) | _BV(TWEN);

		while (! (TWCR & (1<<TWINT))) ;
		if ((TWSR & 0xF8) != TWI_MTX_ADR_ACK) {
			usart_tx_str("Error: can't receive ack for send address\r\n");

			uint8_t sr = TWSR & 0xF8;

			char c1 = "0123456789ABCDEF"[(sr>>4)];
			char c2 = "0123456789ABCDEF"[(sr & 0x0F)];
			usart_tx(c1);
			usart_tx(c2);
			usart_tx_str("\r\n");
			return;
		}

	/***************************************************************/
		usart_tx_str("Send data\r\n");
		TWDR = 'a';
		TWCR = _BV(TWINT) | _BV(TWEN);

		while (! (TWCR & (1<<TWINT))) ;
		if ((TWSR & 0xF8) != TWI_MTX_DATA_ACK) {
			usart_tx_str("Error: can't receive ack for send data\r\n");
			return;
		}

	/***************************************************************/
		usart_tx_str("Send re-start\r\n");
		TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
		while (! (TWCR & (1<<TWINT))) ;
		if ((TWSR & 0xF8) != TWI_REP_START && (TWSR & 0xF8) != TWI_START) {
			usart_tx_str("Error: can't send (re-)start condition\r\n");
			return;
		}

	/***************************************************************/
		usart_tx_str("Send address to read\r\n");
		TWDR = (TWI_SLAVE_ADDRESS << 1) | 1;
		TWCR = _BV(TWINT) | _BV(TWEN);

		while (! (TWCR & (1<<TWINT))) ;
		if ((TWSR & 0xF8) != TWI_MRX_ADR_ACK) {
			usart_tx_str("Error: can't receive ack for send address to read\r\n");
			return;
		}

	/***************************************************************/
		usart_tx_str("Receive data (and send NACK)\r\n");
		TWCR = _BV(TWINT) | _BV(TWEN);

		while (! (TWCR & (1<<TWINT))) ;

		{
			uint8_t sr = TWSR & 0xF8;

			char c1 = "0123456789ABCDEF"[(sr>>4)];
			char c2 = "0123456789ABCDEF"[(sr & 0x0F)];
			usart_tx(c1);
			usart_tx(c2);
			usart_tx_str("\r\n");
		}

		if ((TWSR & 0xF8) != TWI_MRX_DATA_NACK && (TWSR & 0xF8) != TWI_MRX_DATA_ACK ) {
			usart_tx_str("Error: can't receive data\r\n");

			return;
		}

		usart_tx_str("Received:");
		usart_tx(TWDR);
		usart_tx_str("\r\n");

		usart_tx_str("Stop Condition\r\n");
		TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);
}

void twi_slave(void)
{
	TWCR = 0x00;
	TWBR = 0x00;
	TWSR = 0x00;
	TWCR = 0x04;
	TWAMR = 0x00;
	TWAR = 0x10 << 1;

	TWCR = _BV(TWEN) | _BV(TWINT) | _BV(TWEA);

	/***************************************************************/
	usart_tx_str("Wait Start & Address\r\n");
	while (! (TWCR & (1<<TWINT))) ;

	{
		uint8_t sr = TWSR & 0xF8;
		char c1 = "0123456789ABCDEF"[(sr>>4)];
		char c2 = "0123456789ABCDEF"[(sr & 0x0F)];
		usart_tx(c1);
		usart_tx(c2);
		usart_tx_str("\r\n");
	}

	if ((TWSR & 0xF8) != TWI_SRX_ADR_ACK) {
		usart_tx_str("Address not match\r\n");
		return;
	}

	TWCR = _BV(TWEN) | _BV(TWINT) | _BV(TWEA);

	/***************************************************************/
	usart_tx_str("Wait Data\r\n");
	while (! (TWCR & (1<<TWINT))) ;

	{
		uint8_t sr = TWSR & 0xF8;
		char c1 = "0123456789ABCDEF"[(sr>>4)];
		char c2 = "0123456789ABCDEF"[(sr & 0x0F)];
		usart_tx(c1);
		usart_tx(c2);
		usart_tx_str("\r\n");
	}

	if ((TWSR & 0xF8) != TWI_SRX_ADR_DATA_ACK) {
		usart_tx_str("Error: can't receive data\r\n");
		return;
	}

	usart_tx_str("Received: ");
	usart_tx(TWDR);
	usart_tx_str("\r\n");

	TWCR = _BV(TWEN) | _BV(TWINT);

	/***************************************************************/
	usart_tx_str("Send Ack\r\n");
	while (! (TWCR & (1<<TWINT))) ;

	{
		uint8_t sr = TWSR & 0xF8;
		char c1 = "0123456789ABCDEF"[(sr>>4)];
		char c2 = "0123456789ABCDEF"[(sr & 0x0F)];
		usart_tx(c1);
		usart_tx(c2);
		usart_tx_str("\r\n");
	}

	if ((TWSR & 0xF8) != TWI_SRX_STOP_RESTART) {
		usart_tx_str("Error: can't detect stop condition\r\n");
		return;
	}

	TWCR = _BV(TWEN) | _BV(TWINT) | _BV(TWEA);
}

int main()
{
	delay_ms(1000);

	usart_init();

	TWCR = 0x00;
	TWBR = 0x02;
	TWSR = 0x00;
	TWCR = 0x04;

	sei();

	twi_master();

	twi_slave();

	do {} while (1);
}