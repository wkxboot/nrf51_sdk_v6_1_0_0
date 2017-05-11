#ifndef  __BSP_BTN_SMART_LOCKER_H__
#define  __BSP_BTN_SMART_LOCKER_H__

#if     UART_DEBUG > 0
#define DEBUG_INFO(X)  simple_uart_putstring(X)
#else
#define DEBUG_INFO(X)  
#endif


#define BUTTON_DETECTION_DELAY                   50  //in ms
#define SYS_RUN_LED_INTERVAL_IN_MS               500//in ms



#define  LOCK_WAIT_TIMEOUT_MS                      200
#define  COIN_BOX_WAIT_TIMEOUT_SEC                 8
#define  CONTROL_TIMER_TIMEOUT_SEC                 1
#define  DEFAULT_MAX_COIN_CNT                      3

#define  DEVICE_TYPE_UV_LAMP                       1
#define  DEVICE_TYPE_UV_LAMP_DOOR                  2
#define  DEVICE_TYPE_FAN_NEGATIVE_ION              3
#define  DEVICE_TYPE_ELEC_LOCK                     4

#define  DEVICE_STATUS_RUN                         1//closed 
#define  DEVICE_STATUS_STOP                        2//opened


#define  CONTROL_CMD_RUN                           1
#define  CONTROL_CMD_STOP                          2


#define DEFAULT_OPEN_LOCK_TRY_TIMES                3

#if LEDS_NUMBER > 0
bool bsp_board_led_state_get(uint32_t led_pin);
void bsp_board_led_on(uint32_t led_pin);
void bsp_board_led_off(uint32_t led_pin);
void bsp_board_leds_off(void);
void bsp_board_leds_on(void);
void bsp_board_led_invert(uint32_t led_pin);
void bsp_board_leds_init(void);

#endif

#if SWITCHS_NUMBER > 0
bool bsp_board_switch_state_get(uint32_t switch_pin);
void bsp_board_switch_on(uint32_t switch_pin);
void bsp_board_switch_off(uint32_t switch_pin);
void bsp_board_switchs_on(void);
void bsp_board_switch_invert(uint32_t switch_pin);
void bsp_board_switchs_init(void);

#endif
#if BUTTONS_NUMBER > 0
bool bsp_board_button_state_get(uint32_t button_pin);
void bsp_board_buttons_init(void);
#endif

 void bsp_cb_on_uv_lamp_cmd_write(ble_sls_t * p_sls, ble_gatts_evt_write_t * p_evt_write);
 void bsp_cb_on_fan_negative_ion_cmd_write(ble_sls_t * p_sls, ble_gatts_evt_write_t * p_evt_write);
 void bsp_cb_on_elec_lock_cmd_write(ble_sls_t * p_sls, ble_gatts_evt_write_t * p_evt_write);

uint32_t bsp_smart_locker_board_init(uint32_t prescale);

uint8_t *uint8_to_string(uint8_t src);
#endif
