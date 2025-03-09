/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/console/console.h>
#include "conn_time_sync.h"
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>

int main(void)
{
	if(uart_poll_init()){
		printk("ERROR: Could not init USB UART!\n");
		return -1;
	}

	char role;
	console_init();
	
	do {
		uart_poll_print("Choose device role - type c (central) or p (peripheral): ");

		while(uart_get_char(&role));

		switch (role) {
		case 'p':
			uart_poll_print("\nPeripheral. Starting advertising\n");
			peripheral_start();
			break;
		case 'c':
			uart_poll_print("\nCentral. Starting scanning\n");
			central_start();
			break;
		default:
			uart_poll_print("\n");
			break;
		}
	} while (role != 'c' && role != 'p');
}
