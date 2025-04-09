#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include "buzzer_control.h"

#define MAX_PRINTF_SIZE 128
#define RING_BUFFER_SIZE 512

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

RING_BUF_DECLARE(rx_ring_buf, RING_BUFFER_SIZE);
RING_BUF_DECLARE(tx_ring_buf, RING_BUFFER_SIZE);

K_SEM_DEFINE(response_sem, 0, 1);
K_MUTEX_DEFINE(printf_mutex);

static const struct device *const uart_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

static void uart_console_irq_handler(const struct device *dev, void *user_data){
    ARG_UNUSED(user_data);

    while (uart_irq_update(dev) && uart_irq_is_pending(dev)){
        printk("UART IRQ\n");

        // Handle TX
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
        // Handle RX
        int rx_bytes = uart_irq_rx_ready(dev);
        if(rx_bytes > 0){
            uint8_t *rx_buffer;
            int buffer_size = ring_buf_put_claim(&rx_ring_buf, &rx_buffer, rx_bytes);
            int bytes_read = uart_fifo_read(dev, rx_buffer, buffer_size);
            ring_buf_put_finish(&rx_ring_buf, bytes_read);
            printk("Read %d bytes\n", bytes_read);
            printk("%d bytes in ring\n", ring_buf_size_get(&rx_ring_buf));
            k_sem_give(&response_sem);
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
    uart_irq_rx_enable(uart_dev);

    printk("UART init complete\n");
    return 0;

}

void uart_console_printf(char *fmt, ...)
{
    // Note: Can't really use ring buffer claim here because
    // the ring buffer wraps cause only small buffers to be 
    // available occasionally

    // Aquire lock
    // k_mutex_lock(&printf_mutex, K_FOREVER);

    char tx_buffer[MAX_PRINTF_SIZE];
    va_list argptr;
    va_start(argptr,fmt);
    int bytes_written = vsnprintf(tx_buffer, MAX_PRINTF_SIZE, fmt, argptr);
    va_end(argptr);

    ring_buf_put(&tx_ring_buf, tx_buffer, bytes_written);
    uart_irq_tx_enable(uart_dev);

    // Release lock
    // k_mutex_unlock(&printf_mutex);
}

void uart_console_command_loop(void)
{
    uart_console_printf("Ready for commands\n");
    while(1){
        k_sem_take(&response_sem, K_FOREVER);
        while(!ring_buf_is_empty(&rx_ring_buf)){
            char byte;
            ring_buf_get(&rx_ring_buf, &byte, 1);
            switch(tolower(byte)){
            case 'a':
                // ARM
                controller_arm();
                break;
            case 'r':
                // Reset
                controller_reset();
            default:
                break;
            }
        }
    }
}

SYS_INIT(uart_console_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
