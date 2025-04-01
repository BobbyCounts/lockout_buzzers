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
#include "uart_console.h"

int main(void)
{
	console_init();
	
#ifdef CONFIG_BUZZER_ROLE_MODE_CONTROLLER
	uart_console_printf("Configured as buzzer controller\n");
	central_start();
#else
	uart_console_printf("Configured as buzzer\n");
	peripheral_start();
#endif
}
