#ifndef UART_POLLING_H__
#define UART_POLLING_H__
int uart_poll_init(void);
void uart_poll_print(char *message);
void uart_printf(char *fmt, ...);
int uart_get_char(char *character);
#endif
