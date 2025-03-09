#include <stdio.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>

#define MAX_PRINTF_SIZE 200

static const struct device *const uart_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

int uart_poll_init(void)
{
	if (usb_enable(NULL)) {
		return -1;
	}else{
		return 0;
	}
}

void uart_poll_print(char *message)
{
	while(*message != '\0'){
		if(*message == '\n'){
		uart_poll_out(uart_dev, '\r');
		}
		uart_poll_out(uart_dev, *message);
		message++;
	}
}

void uart_printf(char *fmt, ...)
{
	char formatted_string[MAX_PRINTF_SIZE];
	va_list argptr;
	va_start(argptr,fmt);
	vsnprintf(formatted_string, sizeof(formatted_string), fmt, argptr);
	va_end(argptr);
	uart_poll_print(formatted_string);
}

int uart_get_char(char *character)
{
	return uart_poll_in(uart_dev, character);
}
