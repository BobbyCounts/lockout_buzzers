#ifndef PARAM_STACK_H
#define PARAM_STACK_H
#include <zephyr/bluetooth/gatt.h>
int param_stack_get_write(struct bt_gatt_write_params **data);
int param_stack_get_read(struct bt_gatt_read_params **data);
int param_stack_free_write(struct bt_gatt_write_params *data);
int param_stack_free_read(struct bt_gatt_read_params *data);
#endif