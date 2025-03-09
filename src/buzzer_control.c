#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/gpio.h>
#include "param_stack.h"
#include "buzzer_control.h"
#include "uart_polling.h"
#include "buzzer_gatt.h"
#include "conn_time_sync.h"

#define BUZZER_ARM_DELAY_US 120000

// Time to wait for buzzes before deciding the winner in ms
#define BUZZER_DECIDE_WINNER_DELAY 120

// Data structure definitions
struct controller_params {
	uint64_t arm_timestamp;
    atomic_t recievced_first_buzz;
	atomic_t lock;
};

struct device_params {
	uint64_t last_buzz_time;
	atomic_t last_buzz_time_valid;
	const bt_addr_le_t *buzzer_addr;
	uint16_t buzzer_char_handle;
 	uint16_t pressed_char_handle;
 	uint16_t *timed_char_handle_ptr;
};

struct device_ids {
	const bt_addr_t buzzer_addr;
	const char buzzer_name[25];
};

// Consts
static const struct gpio_dt_spec input_button = GPIO_DT_SPEC_GET(DT_NODELABEL(button0), gpios);
static const struct bt_uuid *gatt_ccc_uuid = BT_UUID_GATT_CCC;

// Buzzer IDs
const struct device_ids buzzer_ids[] = {
	{
		.buzzer_addr = {{0xEE, 0xA1, 0x46, 0x6F, 0x1B, 0xCA}},
		.buzzer_name = "Buzzer 1"
	},
	{
		.buzzer_addr = {{0x6B, 0x65, 0xF0, 0x32, 0xB1, 0xD7}},
		.buzzer_name = "Buzzer 2"
	},
	{
		.buzzer_addr = {{0}},
		.buzzer_name = "Buzzer 3"
	}
};

// State Variables
struct controller_params controller_state;
static struct bt_gatt_subscribe_params subscribe_params;
static struct bt_gatt_discover_params char_params;
static struct bt_gatt_discover_params ccc_params;
static struct device_params device_state[CONFIG_BT_MAX_CONN];

// Callback data for gpio
static struct gpio_callback pin_cb_data;

// Function declarations
static void gpio_input_config(void);
static void send_arm_packet(struct bt_conn *conn, void *data);

static void on_send_buzzer_arm(struct k_work *work)
{
	uart_printf("Send ARM\n");

	controller_state.arm_timestamp = controller_time_us_get() + BUZZER_ARM_DELAY_US;
	bt_conn_foreach(BT_CONN_TYPE_LE, send_arm_packet, &controller_state.arm_timestamp);


}
K_WORK_DEFINE(send_arm_signal, on_send_buzzer_arm);

// Initialize the buzzer controller
void buzzer_controller_init(void)
{
	gpio_input_config();
    // Clear state
    controller_state.arm_timestamp = 0;
    atomic_clear_bit(&controller_state.lock, 0);
    atomic_clear_bit(&controller_state.recievced_first_buzz, 0);
}

static void controller_arm_button_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    // Queue the arm signal packet
	k_work_submit(&send_arm_signal);
}

// Configure the input button
static void gpio_input_config(void)
{
	gpio_pin_configure_dt(&input_button, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&input_button, GPIO_INT_EDGE_TO_ACTIVE);

    gpio_init_callback(&pin_cb_data, controller_arm_button_isr, BIT(input_button.pin));

	gpio_add_callback(input_button.port, &pin_cb_data);
}

// Callback for when a write packet is sent
static void write_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_write_params *params)
{
    if (err) {
        uart_printf("GATT Write failed: %d\n", err);
    } else {
        uart_printf("GATT Write successful\n");
    } 
	param_stack_free_write(params);
}

static void send_arm_packet(struct bt_conn *conn, void *data)
{
	int err;
	struct bt_conn_info conn_info;

	err = bt_conn_get_info(conn, &conn_info);
	if (err) {
		return;
	}

	uint8_t conn_index = bt_conn_index(conn);

	if (conn_info.state != BT_CONN_STATE_CONNECTED) {
		return;
	}

	if (device_state[conn_index].buzzer_char_handle == 0) {
		/* Service discovery not yet complete. */
		return;
	}

	struct bt_gatt_write_params *write_ptr;
	if(param_stack_get_write(&write_ptr)){
		uart_printf("Error getting write buffer\n");
		return;
	}
	// Clear last buzz time before sending arm packet
	atomic_clear_bit(&device_state[conn_index].last_buzz_time_valid, 0);
	write_ptr->handle = device_state[conn_index].buzzer_char_handle;
	write_ptr->data = data;
	write_ptr->length = sizeof(uint64_t);
	write_ptr->func = write_cb;

	bt_gatt_write(conn, write_ptr);
}

static const char *get_buzzer_name(const bt_addr_t *addr)
{
	int i;
	for(i = 0; i < sizeof(buzzer_ids)/sizeof(buzzer_ids[0]); i++){
		if(bt_addr_cmp(addr, &buzzer_ids[i].buzzer_addr) == 0){
			return buzzer_ids[i].buzzer_name;
		}
	}
	return "Unknown";
}

static void on_decide_winner(struct k_work *work)
{
    // Get all recieved buzzer times and compare (find the lowest value)
    uart_printf("Deciding winner\n");

	int i;
	uint64_t lowest_time = UINT64_MAX;
	uint8_t lowest_index = 0;
	for(i = 0; i < CONFIG_BT_MAX_CONN; i++){
		if(atomic_test_bit(&device_state[i].last_buzz_time_valid, 0)){
			if(device_state[i].last_buzz_time < lowest_time){
				lowest_time = device_state[i].last_buzz_time;
				lowest_index = i;
			}
		}
	}

	uart_printf("Winner: %s\n", get_buzzer_name(&device_state[lowest_index].buzzer_addr->a));

    // Disarm buzzers
    controller_state.arm_timestamp = 0;
    atomic_clear_bit(&controller_state.recievced_first_buzz, 0);
	bt_conn_foreach(BT_CONN_TYPE_LE, send_arm_packet, &controller_state.arm_timestamp);
    return;
}
K_WORK_DELAYABLE_DEFINE(decide_winner, on_decide_winner);


// Callback for when a indication packet is recieved
static uint8_t indicate_callback(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length){
	uint8_t conn_index = bt_conn_index(conn);

	if(conn == NULL){
		uart_printf("Device is being unpaired\n");\
		return BT_GATT_ITER_STOP;
	}
	if(data == NULL){
		uart_printf("Subscription was removed\n");
		return BT_GATT_ITER_STOP;
	}

	if (length != sizeof(device_state[conn_index].last_buzz_time)) {
		uart_printf("Error: ARM Timestamp\n");
		return BT_GATT_ITER_CONTINUE;
	}

    // Data is a valid buzz
    // Check if this is the first buzzer to buzz in
    if(!atomic_test_bit(&controller_state.recievced_first_buzz, 0)){
        atomic_set_bit(&controller_state.recievced_first_buzz, 0);
        uart_printf("First buzz\n");
        k_work_schedule(&decide_winner, K_MSEC(BUZZER_DECIDE_WINNER_DELAY));
    }
	
	memcpy(&device_state[conn_index].last_buzz_time, data, sizeof(device_state[conn_index].last_buzz_time));
	atomic_set_bit(&device_state[conn_index].last_buzz_time_valid, 0);
	
	uart_printf("Got pressed time: %llu\n", device_state[conn_index].last_buzz_time);

	return BT_GATT_ITER_CONTINUE;
}

static int buzzer_subscribe(struct bt_conn *conn)
{
	uint8_t conn_index = bt_conn_index(conn);
	subscribe_params.notify = indicate_callback;
	subscribe_params.value = BT_GATT_CCC_INDICATE;
	subscribe_params.value_handle = device_state[conn_index].pressed_char_handle;
	// subscribe_params.ccc_handle set by discovery
	return bt_gatt_subscribe(conn, &subscribe_params);
}

static uint8_t on_discover_ccc(struct bt_conn *conn,
	const struct bt_gatt_attr *attr,
	struct bt_gatt_discover_params *params)
{
	if (attr) {
		// uart_printf("Found Pressed CCC\n");
		subscribe_params.ccc_handle = attr->handle;

		buzzer_subscribe(conn);

		return BT_GATT_ITER_STOP;
	} else {
		uart_printf("CCC Done\n");
		return BT_GATT_ITER_STOP;
	}
	return BT_GATT_ITER_STOP;
}

static uint8_t on_discover_char(struct bt_conn *conn,
	const struct bt_gatt_attr *attr,
	struct bt_gatt_discover_params *params)
{
	uint8_t conn_index = bt_conn_index(conn);
	if (attr) {
		struct bt_gatt_chrc *char_val = attr->user_data;
		if(bt_uuid_cmp(char_val->uuid, BT_UUID_BUZZER_ARM_CHAR) == 0){
			// uart_printf("Found Arm Characteristic\n");
			device_state[conn_index].buzzer_char_handle = bt_gatt_attr_value_handle(attr);
		}
		else if(bt_uuid_cmp(char_val->uuid, BT_UUID_TIMED_ACTION_CHAR) == 0){
			// uart_printf("Found Time Characteristic\n");
			*device_state[conn_index].timed_char_handle_ptr = bt_gatt_attr_value_handle(attr);
		}
		else if(bt_uuid_cmp(char_val->uuid, BT_UUID_BUZZER_PRESSED_CHAR) == 0){
			// uart_printf("Found Pressed Characteristic\n");
			device_state[conn_index].pressed_char_handle = bt_gatt_attr_value_handle(attr);
			ccc_params.uuid = gatt_ccc_uuid;
			ccc_params.start_handle = attr->handle + 2;
			ccc_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
			ccc_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
			ccc_params.func = on_discover_ccc;
			bt_gatt_discover(conn, &ccc_params);
		}
		else if(bt_uuid_cmp(char_val->uuid, BT_UUID_BUZZER_LIGHT_CHAR) == 0){
			// uart_printf("Found Light Characteristic\n");
		}
		return BT_GATT_ITER_CONTINUE;
	} else {
		uart_printf("Discovery Done\n");
		return BT_GATT_ITER_STOP;
	}
	return BT_GATT_ITER_STOP;
}

int get_buzzer_chars(struct bt_conn *conn, uint16_t *timed_handle)
{
	char_params.uuid = NULL;
	char_params.func = on_discover_char;
	char_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	char_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	char_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

	uint8_t conn_index = bt_conn_index(conn);
	device_state[conn_index].timed_char_handle_ptr = timed_handle;

	// Set address
	device_state[conn_index].buzzer_addr = bt_conn_get_dst(conn);

	// if(bt_addr_cmp(&device_state[conn_index].buzzer_addr->a, &buzzer_ids[0].buzzer_addr) == 0){
	// 	uart_printf("Found buzzer: %s\n", buzzer_ids[0].buzzer_name);
	// }

	return bt_gatt_discover(conn, &char_params);
}
