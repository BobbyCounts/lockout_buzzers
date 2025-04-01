#include <stdio.h>
#include <string.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/init.h>

#define MAX_PRINTF_SIZE 200
#define UART_CONSOLE_TX_RING_BUF_SIZE 512
#define UART_CONSOLE_RX_RING_BUF_SIZE 128
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

RING_BUF_DECLARE(tx_ring_buf, UART_CONSOLE_TX_RING_BUF_SIZE);
RING_BUF_DECLARE(rx_ring_buf, UART_CONSOLE_RX_RING_BUF_SIZE);

static const struct device *const uart_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

static void uart_console_irq_handler(const struct device *dev, void *user_data){
    ARG_UNUSED(user_data);

    while (uart_irq_update(dev) && uart_irq_is_pending(dev)){
        int empty_spaces = uart_irq_tx_ready(dev);
        if(empty_spaces > 0){
            char tx_buffer[128];
            int bytes_to_read = MIN(sizeof(tx_buffer), empty_spaces);
            int bytes_from_ring_buf = ring_buf_get(&tx_ring_buf, tx_buffer, bytes_to_read);
            if(!bytes_from_ring_buf){
                // Ring buffer empty, disable tx
                uart_irq_tx_disable(dev);
                continue;
            }
            int sent_bytes = uart_fifo_fill(dev, tx_buffer, bytes_from_ring_buf);
            if(sent_bytes != bytes_from_ring_buf){
                printk("Dropped uart bytes\n");
            }
        }
    }
    return;
}

static int uart_console_init(void)
{
    if (usb_enable(NULL)) {
        return -1;
    }

    uart_irq_callback_user_data_set(uart_dev, uart_console_irq_handler, NULL);

    printk("UART init complete\n");
    return 0;

}

void uart_console_printf(char *fmt, ...)
{
    char formatted_string[MAX_PRINTF_SIZE];
    va_list argptr;
    va_start(argptr,fmt);
    vsnprintf(formatted_string, sizeof(formatted_string), fmt, argptr);
    va_end(argptr);
    uart_irq_tx_enable(uart_dev);
    // Send to the ring buffer
    ring_buf_put(&tx_ring_buf, formatted_string, strlen(formatted_string));
}

SYS_INIT(uart_console_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
