#include <zephyr/kernel.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/gpio.h>
#include "uart_console.h"
#include "conn_time_sync.h"
#include "param_stack.h"
#include "buzzer_gatt.h"

#define BUZZER_LOCKOUT 2000
#define BUZZER_LED_FLASH_DELAY_MS 100

// Data structure definitions
// Buzzer state struct
struct buzzer_params {
	uint64_t buzzer_pushed_timestamp; 
	uint64_t armed_timestamp;
	atomic_t lock;
	atomic_t armed;
	uint8_t lights;
};

struct buzzer_sync {
    int64_t offset_to_central;
    atomic_t offset_valid;
};

// Consts
static const struct gpio_dt_spec input_button = GPIO_DT_SPEC_GET(DT_NODELABEL(push_button), gpios);
static const struct gpio_dt_spec led_drive = GPIO_DT_SPEC_GET(DT_NODELABEL(led_drive), gpios);

// Function Declarations
void buzzer_led_init(void);
void buzzer_led_on(void);
void buzzer_led_off(void);
void buzzer_led_start_flash(void);

// Callback data for gpio
static struct gpio_callback pin_cb_data;

// Device state
static struct buzzer_params buzzer_state;
static atomic_t indicate_flag;
static struct bt_gatt_indicate_params ind_params;
static struct buzzer_sync buzzer_timing;
static int led_state = 0;

// Function Declarations
static void on_indicate_buzzer_time(struct k_work *work);

void buzzer_update_offset(uint64_t central_timestamp, uint64_t peripheral_timestamp)
{
    buzzer_timing.offset_to_central = central_timestamp - peripheral_timestamp;
    atomic_set_bit(&buzzer_timing.offset_valid, 0);
}

static void on_reenable_buzzer(struct k_work *work)
{
	gpio_pin_interrupt_configure_dt(&input_button, GPIO_INT_EDGE_TO_ACTIVE);
}
K_WORK_DELAYABLE_DEFINE(buzzer_reenable, on_reenable_buzzer);


K_WORK_DEFINE(indicate_buzzer_time, on_indicate_buzzer_time);

// Buzzer button ISR
static void pin_isr_buzzer(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	// Check if buzzer is not armed
	if(!atomic_test_bit(&buzzer_state.armed, 0)){
		// Disable buzzer for BUZZER_LOCKOUT
		uart_console_printf("Buzzer Locked\n");
		gpio_pin_interrupt_configure_dt(&input_button, GPIO_INT_DISABLE);
		k_work_schedule(&buzzer_reenable, K_MSEC(BUZZER_LOCKOUT));
	}
	// Buzzer is armed, check if we already have a valid buzz time and valid offset
	else if(!buzzer_state.buzzer_pushed_timestamp && atomic_test_bit(&buzzer_timing.offset_valid, 0)){
		// Set the buzzer time
		buzzer_state.buzzer_pushed_timestamp = controller_time_us_get() + buzzer_timing.offset_to_central;
		uart_console_printf("Buzzed in @ %llu central\n", buzzer_state.buzzer_pushed_timestamp);
		k_work_submit(&indicate_buzzer_time);
	}
}


static void gpio_input_config(void)
{
	gpio_pin_configure_dt(&input_button, GPIO_INPUT | GPIO_PULL_UP);
	gpio_pin_interrupt_configure_dt(&input_button, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&pin_cb_data, pin_isr_buzzer, BIT(input_button.pin));
	gpio_add_callback(input_button.port, &pin_cb_data);
}

void buzzer_device_init(void)
{
    gpio_input_config();
	buzzer_led_init();
}

void buzzer_device_reset(void)
{
    buzzer_state.buzzer_pushed_timestamp = 0;
    buzzer_state.armed_timestamp = 0;
    buzzer_state.lights = 0;
    atomic_clear_bit(&buzzer_state.armed, 0);
    atomic_clear_bit(&buzzer_state.lock, 0);
    atomic_clear_bit(&indicate_flag, 0);

    buzzer_timing.offset_to_central = 0;
    atomic_clear_bit(&buzzer_timing.offset_valid, 0);

	buzzer_led_off();
}


void pressed_ccc_cfg_changed(const struct bt_gatt_attr *attr,
	uint16_t value)
{
	if(value == BT_GATT_CCC_INDICATE){
		atomic_set_bit(&indicate_flag, 0);
		uart_console_printf("Client Subscribed\n");
	}else{
		atomic_clear_bit(&indicate_flag, 0);
		uart_console_printf("Client Unsubscribed\n");
	}
}


static void on_indicate_buzzer_time(struct k_work *work)
{
	// Check to see if we are still armed and indications are enabled
	if(atomic_test_bit(&buzzer_state.armed, 0) && atomic_test_bit(&indicate_flag, 0)){
		uart_console_printf("Indicate\n");
		ind_params.uuid = BT_UUID_BUZZER_PRESSED_CHAR;
		ind_params.func = NULL;
		ind_params.destroy = NULL;
		ind_params.data = &buzzer_state.buzzer_pushed_timestamp ;
		ind_params.len = sizeof(buzzer_state.buzzer_pushed_timestamp );
		bt_gatt_indicate(NULL, &ind_params);
	}
}
// Callback for the buzzer arming
ssize_t buzzer_arm_recieved(struct bt_conn *conn,
    const struct bt_gatt_attr *attr,
    const void *buf, uint16_t len,
    uint16_t offset, uint8_t flags)
{
    if (len != sizeof(buzzer_state.armed_timestamp) || offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    // Obtain a lock for the buzzer struct
    if(atomic_test_and_set_bit(&buzzer_state.lock, 0)){
    uart_console_printf("ERROR ARM: Already Locked\n");
        return 0;
    }
    memcpy(&buzzer_state.armed_timestamp, buf, sizeof(buzzer_state.armed_timestamp));

    // Set the ARM flag
    if(buzzer_state.armed_timestamp){
        uart_console_printf("Arm buzzer @ %llu central\n", buzzer_state.armed_timestamp);
        // Set the buzzer time trigger
        timed_led_toggle_trigger_at(led_state, buzzer_state.armed_timestamp - buzzer_timing.offset_to_central);
		if(led_state){
			led_state = 0;
		}else{
			led_state = 1;
		}
    }else{
        atomic_clear_bit(&buzzer_state.armed, 0);
        buzzer_state.buzzer_pushed_timestamp = 0;
		buzzer_led_off();
        uart_console_printf("Buzzer Disarmed\n");
    }

    atomic_clear_bit(&buzzer_state.lock, 0);
    return len;
}

ssize_t buzzer_light_recieved(struct bt_conn *conn,
    const struct bt_gatt_attr *attr,
    const void *buf, uint16_t len,
    uint16_t offset, uint8_t flags)
{
    if (len != sizeof(buzzer_state.lights) || offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    // Obtain a lock for the buzzer struct
    if(atomic_test_and_set_bit(&buzzer_state.lock, 0)){
    uart_console_printf("ERROR ARM: Already Locked\n");
        return 0;
    }
    memcpy(&buzzer_state.lights, buf, sizeof(buzzer_state.lights));

    // Set the ARM flag
	if(buzzer_state.lights){
		buzzer_led_on();
	}else{
		buzzer_led_off();
	}

    atomic_clear_bit(&buzzer_state.lock, 0);
    return len;
}

// Function used by the timer isr to set the arming flag
void buzzer_set_arm_flag(void)
{
    atomic_set_bit(&buzzer_state.armed, 0);
	buzzer_led_start_flash();
    uart_console_printf("Buzzer ARMED\n");
}

///////////////////////////////////////////////////////
// Buzzer LED Functions
///////////////////////////////////////////////////////

void buzzer_led_init(void)
{
    gpio_pin_configure_dt(&led_drive, GPIO_OUTPUT_ACTIVE);
	buzzer_led_off();
}

void buzzer_led_on(void)
{
    gpio_pin_set_dt(&led_drive, 1);
}

void buzzer_led_off(void)
{
    gpio_pin_set_dt(&led_drive, 0);
}

static void on_buzzer_led_flash(struct k_work *work)
{
	// Make sure buzzer is still armed
	if(atomic_test_bit(&buzzer_state.armed, 0)){
		gpio_pin_toggle_dt(&led_drive);
		k_work_schedule(k_work_delayable_from_work(work), K_MSEC(BUZZER_LED_FLASH_DELAY_MS));
	}
}
K_WORK_DELAYABLE_DEFINE(buzzer_led_flash, on_buzzer_led_flash);

void buzzer_led_start_flash(void)
{
    buzzer_led_on();
    k_work_schedule(&buzzer_led_flash, K_MSEC(BUZZER_LED_FLASH_DELAY_MS));
}