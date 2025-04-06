#ifndef BUZZER_CONTROL_H
#define BUZZER_CONTROL_H
#include <stdint.h>
#include <zephyr/bluetooth/conn.h>

void buzzer_controller_init(void);
int get_buzzer_chars(struct bt_conn *conn, uint16_t *timed_handle);

void controller_arm(void);
void controller_reset(void);

#endif // BUZZER_CONTROL_H