#include <avr/io.h>
#include <avr/interrupt.h>

#include "usart.h"

// USART処理 =====================================
#define USART_TX_BUFFER_SIZE 64
#define USART_TX_BUFFER_MASK 0x3f

volatile uint8_t tx_buffer[USART_TX_BUFFER_SIZE];
volatile uint8_t tx_ri;
volatile uint8_t tx_wi;
volatile uint8_t tx_cnt;


void usart_init(void)
{
  // USART設定
  UBRR0 = 25;           // 19200bps
  UCSR0A &= ~0b00000010;
  UCSR0C = 0b00000110;  // 非同期, 8bit, パリティ無し, stop:1bit
  UCSR0B = _BV(TXEN0);

  tx_ri = tx_wi = tx_cnt = 0;
}

ISR(USART_UDRE_vect)
{
  uint8_t n = tx_cnt;

  if (n == 0) {
    UCSR0B = _BV(TXEN0);
    return;
  }

  uint8_t i = tx_ri;
  UDR0 = tx_buffer[i];
  tx_ri  = (i + 1) & USART_TX_BUFFER_MASK;
  tx_cnt = --n;

  if (tx_cnt == 0) {
    UCSR0B = _BV(TXEN0);
  }
}

void usart_write(uint8_t c)
{
  while (tx_cnt >= USART_TX_BUFFER_SIZE) ;

  uint8_t i = tx_wi;
  tx_buffer[i] = c;

  cli();
  tx_cnt++;
  sei();

  tx_wi = (i + 1) & USART_TX_BUFFER_MASK;

  UCSR0B = _BV(TXEN0) | _BV(UDRIE0);
}

void usart_write_string(const char *b)
{
  while (*b != 0) {
    usart_write(*b++);
  }
}

void usart_write_buffer(const uint8_t *b, uint8_t l)
{
  while (l-- > 0) {
    usart_write(*b++);
  }
}

void usart_write_cr(void)
{
  usart_write('\r');
  usart_write('\n');
}
// =============================================
