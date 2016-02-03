
/* Prototypes */

void usart_init(void);
void usart_write(uint8_t c);
void usart_write_string(const char *b);
void usart_write_buffer(const uint8_t *b, uint8_t l);
void usart_write_cr(void);
