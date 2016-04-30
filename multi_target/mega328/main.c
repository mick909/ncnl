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

volatile uint8_t twi_tx_addr;
volatile uint8_t twi_tx_cmd;

volatile uint8_t twi_rx_rep;

volatile uint8_t twi_status;

ISR(TWI_vect)
{
	uint8_t st = TWSR & 0xF8;

	twi_status = st;

	switch (st) {

// General TWI Master status codes
	case TWI_REP_START:
//		usart_tx_str("Repeated START has been transmitted\r\n");
	case TWI_START:
//		usart_tx_str("START has been transmitted -> Send address\r\n");
		TWDR = (twi_tx_addr << 1) | 0;
		TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWIE);
		break;

	case TWI_ARB_LOST:
//		usart_tx_str("Error: Arbitration lost -> restart\r\n");
		TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTA) | _BV(TWIE);
		break;


// TWI Master Transmitter status codes
	case TWI_MTX_ADR_ACK:
//		usart_tx_str("SLA+W has been transmitted and ACK received -> Send Command\r\n");
		TWDR = twi_tx_cmd;
		TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWIE);
		break;

	case TWI_MTX_DATA_ACK:
//		usart_tx_str("Data byte has been transmitted and ACK received -> Exit Transmit\r\n");
		TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);
		do {} while (TWCR & _BV(TWSTO));
		break;


	case TWI_SRX_ADR_ACK:
//		usart_tx_str("Own SLA+W has been received ACK has been returned -> Wait Data\r\n");
		TWCR = _BV(TWEN) | _BV(TWINT) | _BV(TWEA) | _BV(TWIE);
		break;

	case TWI_SRX_ADR_DATA_ACK:
//		usart_tx_str("Previously addressed with own SLA+W; data has been received; ACK has been returned\r\n");
		twi_rx_rep = TWDR;
		TWCR = _BV(TWEN) | _BV(TWINT);
		break;

	case TWI_MTX_ADR_NACK:
	case TWI_MTX_DATA_NACK:
	default:
		TWCR = _BV(TWEN) | _BV(TWINT);
	}
}

void twi_master(void)
{
	TWBR = 0x02;
	TWDR = 0xFF;

	twi_tx_addr = TWI_SLAVE_ADDRESS;
	twi_tx_cmd = 0x10;

	TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN) | _BV(TWIE);
//	TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);


#if 0
		while (! (TWCR & (1<<TWINT))) ;

		{
			uint8_t sr = TWSR & 0xF8;
			char c1 = "0123456789ABCDEF"[(sr>>4)];
			char c2 = "0123456789ABCDEF"[(sr & 0x0F)];
			usart_tx(c1);
			usart_tx(c2);
			usart_tx_str("\r\n");
		}

		if ((TWSR & 0xF8) != TWI_START) {
			usart_tx_str("Error: can't send start condition\r\n");
			return;
		}

	/***************************************************************/
		usart_tx_str("Send address\r\n");
		TWDR = (TWI_SLAVE_ADDRESS << 1) | 0;
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

		if ((TWSR & 0xF8) != TWI_MTX_ADR_ACK) {
			usart_tx_str("Error: can't receive ack for send address\r\n");
			return;
		}

	/***************************************************************/
		usart_tx_str("Send data\r\n");
		TWDR = 0x10;
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

		if ((TWSR & 0xF8) != TWI_MTX_DATA_ACK) {
			usart_tx_str("Error: can't receive ack for send data\r\n");
			return;
		}

		usart_tx_str("Stop Condition\r\n");
		TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);

		while (TWCR & _BV(TWSTO)) ;
#endif
}

void twi_slave(void)
{
	TWBR = 0x00;
	TWAMR = 0x00;
	TWAR = 0x10 << 1;

	TWCR = _BV(TWEN) | _BV(TWINT) | _BV(TWEA) | _BV(TWIE);

#if 0
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
	{
		uint8_t sr = TWDR;
		char c1 = "0123456789ABCDEF"[(sr>>4)];
		char c2 = "0123456789ABCDEF"[(sr & 0x0F)];
		usart_tx(c1);
		usart_tx(c2);
	}
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
#endif
}

int main()
{
	usart_init();

	sei();

	do {
		delay_ms(1000);

		usart_tx_str("Transmit: \r\n");
		twi_master();
		usart_tx_str("Transmit Done: \r\n");

		do {} while ( TWCR & (1<<_BV(TWIE)) );

		{
			uint8_t sr = twi_status;
			char c1 = "0123456789ABCDEF"[(sr>>4)];
			char c2 = "0123456789ABCDEF"[(sr & 0x0F)];
			usart_tx_str("TWSR = ");
			usart_tx(c1);
			usart_tx(c2);
			usart_tx_str("\r\n");
		}

		usart_tx_str("Start Slave: \r\n");
		twi_rx_rep = 0;

		twi_slave();

//		do {} while ( TWCR & (1<<_BV(TWIE)) );
		do {} while (twi_rx_rep == 0);

		{
			uint8_t sr = twi_status;
			char c1 = "0123456789ABCDEF"[(sr>>4)];
			char c2 = "0123456789ABCDEF"[(sr & 0x0F)];
			usart_tx_str("TWSR = ");
			usart_tx(c1);
			usart_tx(c2);
			usart_tx_str("\r\n");
		}

		usart_tx_str("Received: ");
		{
			uint8_t sr = twi_rx_rep;
			char c1 = "0123456789ABCDEF"[(sr>>4)];
			char c2 = "0123456789ABCDEF"[(sr & 0x0F)];
			usart_tx(c1);
			usart_tx(c2);
			usart_tx_str("\r\n");
		}
	} while (1);
}