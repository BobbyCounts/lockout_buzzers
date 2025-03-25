#ifndef BUZZER_H
#define BUZZER_H
#include <stdint.h>
#include <zephyr/bluetooth/gatt.h>

ssize_t buzzer_arm_recieved(struct bt_conn *conn,
    const struct bt_gatt_attr *attr,
    const void *buf, uint16_t len,
    uint16_t offset, uint8_t flags);
ssize_t buzzer_light_recieved(struct bt_conn *conn,
        const struct bt_gatt_attr *attr,
        const void *buf, uint16_t len,
        uint16_t offset, uint8_t flags);
void buzzer_device_init(void);
void pressed_ccc_cfg_changed(const struct bt_gatt_attr *attr,
	uint16_t value);
void buzzer_device_reset(void);
void buzzer_update_offset(uint64_t central_timestamp, uint64_t peripheral_timestamp);
void buzzer_set_arm_flag(void);

#endif // BUZZER_H