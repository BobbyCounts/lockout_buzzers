#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include "zephyr/bluetooth/gatt.h"

#define MAX_ITEMS 50

static struct bt_gatt_write_params write_params[MAX_ITEMS];
static struct bt_gatt_read_params read_params[MAX_ITEMS];

K_STACK_DEFINE(write_stack, MAX_ITEMS);
K_STACK_DEFINE(read_stack, MAX_ITEMS);

static int param_stack_init(void)
{
    // Push to stacks
    int i;
    for(i = 0; i < MAX_ITEMS; i++){
        k_stack_push(&write_stack, (stack_data_t)&write_params[i]);
        k_stack_push(&read_stack, (stack_data_t)&read_params[i]);
    }
    return 0;
}

int param_stack_get_write(struct bt_gatt_write_params **data)
{
    return k_stack_pop(&write_stack, (stack_data_t*)data, K_NO_WAIT);
}

int param_stack_get_read(struct bt_gatt_read_params **data)
{
    return k_stack_pop(&read_stack,(stack_data_t*)data, K_NO_WAIT);  
}

int param_stack_free_write(struct bt_gatt_write_params *data)
{
    return k_stack_push(&write_stack, (stack_data_t)data);
}

int param_stack_free_read(struct bt_gatt_read_params *data)
{
    return k_stack_push(&read_stack, (stack_data_t)data);
}

SYS_INIT(param_stack_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);






