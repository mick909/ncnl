#include <avr/io.h>
#include <avr/interrupt.h>

/****************************************************************************
  USART codes
****************************************************************************/
/* for 8MHz, 19200bps, 8bit, parity:no, stop:1 */
#define UBRR_VAL			(25)

#define USART_RXI_ENABLE	(_BV(RXCIE0) | _BV(RXEN0) | _BV(TXEN0))
#define USART_RXTXI_ENALBE	(USART_RXI_ENABLE | _BV(UDRIE0))

#define USART_TX_BUFFER_SIZE 16
#define USART_TX_BUFFER_MASK 0x0F

static uint8_t USART_tx_buffer[USART_TX_BUFFER_SIZE];
volatile uint8_t usart_tx_ri;
volatile uint8_t usart_tx_wi;
volatile uint8_t usart_tx_cnt;

#define USART_RX_BUFFER_SIZE 16
#define USART_RX_BUFFER_MASK 0x0F

static uint8_t USART_rx_buffer[USART_RX_BUFFER_SIZE];
volatile uint8_t usart_rx_ri;
volatile uint8_t usart_rx_wi;
volatile uint8_t usart_rx_cnt;

ISR(USART_RX_vect)
{
	uint8_t data = UDR0;
	uint8_t cnt = usart_rx_cnt;
	uint8_t wi = usart_rx_wi;

	if (cnt == USART_RX_BUFFER_SIZE) {
		/* Overflow, Can't put to buffer */
		return;
	}

	USART_rx_buffer[wi] = data;
	usart_rx_cnt = cnt+1;
	usart_rx_wi = (wi+1) & USART_RX_BUFFER_MASK;
}

ISR(USART_UDRE_vect)
{
	uint8_t cnt = usart_tx_cnt;
	uint8_t ri = usart_tx_ri;

	if (cnt == 0) {
		UCSR0B = USART_RXI_ENABLE;
		return;
	}

	UDR0 = USART_tx_buffer[ri];
	usart_tx_ri = (ri+1) & USART_TX_BUFFER_MASK;
	usart_tx_cnt = cnt-1;
}

void usart_init(void)
{
	UBRR0 = UBRR_VAL;
	UCSR0A = 0b11111100;
	UCSR0C = 0b00000110;	/* 非同期, 8bit, パリティ無し, stop:1bit */
	UCSR0B = USART_RXI_ENABLE;

	usart_tx_ri = usart_tx_wi = usart_tx_cnt = usart_rx_ri = usart_rx_wi = usart_rx_cnt = 0;
}

void usart_tx(uint8_t c)
{
	uint8_t wi;

	while (usart_tx_cnt >= USART_TX_BUFFER_SIZE) ;

	wi = usart_tx_wi;
	USART_tx_buffer[wi] = c;
	cli();
	++usart_tx_cnt;
	sei();

	usart_tx_wi = (wi+1) & USART_TX_BUFFER_MASK;

	UCSR0B = USART_RXTXI_ENALBE;
}

void usart_tx_str(const char *b)
{
	while (*b != 0) {
		usart_tx(*b++);
	}
}

/*
void usart_tx_buffer(const uint8_t *b, uint8_t len)
{
	while(len-- > 0) {
		usart_tx(*b++);
	}
}
*/

/*
void usart_tx_cr(void) {
	usart_tx('\r');
	usart_tx('\n');
}
*/

uint8_t usart_rx_empty()
{
	return usart_rx_cnt == 0;
}

uint8_t usart_rx(void)
{
	uint8_t c;
	uint8_t ri;

	while (usart_rx_cnt == 0) ;

	ri = usart_rx_ri;
	c = USART_rx_buffer[ri];
	usart_rx_ri = (ri+1) & USART_RX_BUFFER_MASK;
	cli();
	--usart_rx_cnt;
	sei();

	return c;
}
/****************************************************************************/
