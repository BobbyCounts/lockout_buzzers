#ifndef BUZZER_CONTROL_H
#define BUZZER_CONTROL_H
#include <stdint.h>

void buzzer_controller_init(void);
int get_buzzer_chars(struct bt_conn *conn, uint16_t *timed_handle);

#endif // BUZZER_CONTROL_H