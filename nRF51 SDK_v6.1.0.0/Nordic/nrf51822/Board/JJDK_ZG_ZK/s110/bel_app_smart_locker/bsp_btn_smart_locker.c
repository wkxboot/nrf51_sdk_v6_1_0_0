#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "boards.h"
#include "nordic_common.h"
#include "simple_uart.h"
#include "ble_sls.h"
#include "app_timer.h"
#include "app_button.h"
#include "bsp_btn_smart_locker.h"



app_timer_id_t lock_wait_timer_id;//开电磁锁脉冲延时
app_timer_id_t sys_run_led_timer_id;//系统运行灯

 extern uint8_t uv_lamp_cmd;
 extern uint8_t fan_negative_ion_cmd;
 extern uint8_t elec_lock_cmd;

 extern uint8_t uv_lamp_status;
 extern uint8_t uv_lamp_door_status;
 extern uint8_t fan_negative_ion_status;
 extern uint8_t elec_lock_status;


 uint8_t uv_lamp_control_cmd=CONTROL_CMD_STOP;
 uint8_t elec_lock_control_cmd=CONTROL_CMD_STOP;

static uint32_t smart_locker_local_prescal=0;
static void bsp_sls_env_value_init(void);

static void bsp_open_uv_lamp_and_indicator_led(void);
static void bsp_close_uv_lamp_and_indicator_led(void);
static void bsp_open_fan_negative_ion_and_indicator_led(void);
static void bsp_close_fan_negative_ion_and_indicator_led(void);
static void bsp_open_elec_lock(void);
static void bsp_open_elec_lock_wait_timeout_handler(void * p_context);

static void bsp_button_event_handler(uint8_t pin_no, uint8_t button_action);
static void bsp_sys_run_led_timeout_handler(void * p_context);
/**@brief This macro will return from the current function if err_code
 *        is not NRF_SUCCESS or NRF_ERROR_INVALID_PARAM.
 */
#define RETURN_ON_ERROR_NOT_INVALID_PARAM(err_code)                             \
do                                                                              \
{                                                                               \
    if (((err_code) != NRF_SUCCESS) && ((err_code) != NRF_ERROR_INVALID_PARAM)) \
    {                                                                           \
        return err_code;                                                        \
    }                                                                           \
}                                                                               \
while (0)


/*
#if LEDS_NUMBER > 0
static const uint8_t m_board_led_list[LEDS_NUMBER] = LEDS_LIST;
#endif

#if BUTTONS_NUMBER > 0
static const uint8_t m_board_btn_list[BUTTONS_NUMBER] = BUTTONS_LIST;
#endif

#if SWITCHS_NUMBER > 0
static const uint8_t m_board_sw_list[SWITCHS_NUMBER] = SWITCHS_LIST;
#endif
*/



#if LEDS_NUMBER > 0
bool bsp_board_led_state_get(uint32_t led_pin)
{
    //ASSERT(led_idx < LEDS_NUMBER);
    bool pin_set = nrf_gpio_pin_read(led_pin) ? true : false;
    return (pin_set == (LEDS_ACTIVE_STATE ? true : false));
}

void bsp_board_led_on(uint32_t led_pin)
{
        //ASSERT(led_pin < LEDS_NUMBER);
        nrf_gpio_pin_write(led_pin, LEDS_ACTIVE_STATE ? 1 : 0);
}

void bsp_board_led_off(uint32_t led_pin)
{
    //ASSERT(led_idx < LEDS_NUMBER);
    nrf_gpio_pin_write(led_pin, LEDS_ACTIVE_STATE ? 0 : 1);
}

void bsp_board_leds_off(void)
{

 bsp_board_led_off(LED_NEGATIVE_ION_PIN_NO);
 bsp_board_led_off(LED_UV_LAMP_PIN_NO);
 bsp_board_led_off(LED_SYS_RUN_PIN_NO);
}

void bsp_board_leds_on(void)
{
 bsp_board_led_on(LED_NEGATIVE_ION_PIN_NO);
 bsp_board_led_on(LED_UV_LAMP_PIN_NO);
 bsp_board_led_on(LED_SYS_RUN_PIN_NO);
}

void bsp_board_led_invert(uint32_t led_pin)
{
    //ASSERT(led_idx < LEDS_NUMBER);
    nrf_gpio_pin_toggle(led_pin);
}

void bsp_board_leds_init(void)
{

  nrf_gpio_cfg_output(LED_NEGATIVE_ION_PIN_NO);
  nrf_gpio_cfg_output(LED_UV_LAMP_PIN_NO);
	nrf_gpio_cfg_output(LED_SYS_RUN_PIN_NO);
  bsp_board_leds_off();
}

#endif //LEDS_NUMBER > 0

#if SWITCHS_NUMBER > 0
//bool bsp_board_switch_state_get(uint32_t switch_pin)
//{
//    //ASSERT(switch_idx < SWITCHS_NUMBER);
//	         nrf_gpio_port_out_read(
//    bool pin_set = nrf_gpio_pin_out_read(m_board_sw_list[switch_idx]) ? true : false;
//    return (pin_set == (SWITCHS_ACTIVE_STATE ? true : false));
//}

void bsp_board_switch_on(uint32_t switch_pin)
{
       // ASSERT(switch_pin < SWITCHS_NUMBER);
        nrf_gpio_pin_write(switch_pin, SWITCHS_ACTIVE_STATE ? 1 : 0);
}

void bsp_board_switch_off(uint32_t switch_pin)
{
    //ASSERT(switch_pin < SWITCHS_NUMBER);
    nrf_gpio_pin_write(switch_pin, SWITCHS_ACTIVE_STATE ? 0 : 1);
}

void bsp_board_switchs_off(void)
{

    bsp_board_switch_off(SWITCH_UV_LAMP_PIN_NO);
		bsp_board_switch_off(SWITCH_FAN_NEGATIVE_ION_PIN_NO);
		bsp_board_switch_off(SWITCH_ELEC_LOCK_PIN_NO);
}

void bsp_board_switchs_on(void)
{

  bsp_board_switch_on(SWITCH_UV_LAMP_PIN_NO);
	bsp_board_switch_on(SWITCH_FAN_NEGATIVE_ION_PIN_NO);
	bsp_board_switch_on(SWITCH_ELEC_LOCK_PIN_NO);
   
}

void bsp_board_switch_invert(uint32_t switch_pin)
{
   // ASSERT(switch_pin < SWITCHS_NUMBER);
    nrf_gpio_pin_toggle(switch_pin);
}

void bsp_board_switchs_init(void)
{
    nrf_gpio_cfg_output(SWITCH_UV_LAMP_PIN_NO);
		nrf_gpio_cfg_output(SWITCH_FAN_NEGATIVE_ION_PIN_NO);
		nrf_gpio_cfg_output(SWITCH_ELEC_LOCK_PIN_NO);
    bsp_board_switchs_off();
}

#endif //SWITCHS_NUMBER > 0


#if BUTTONS_NUMBER > 0
bool bsp_board_button_state_get(uint32_t button_pin)
{
    //ASSERT(button_pin < BUTTONS_NUMBER);
    bool pin_set = nrf_gpio_pin_read(button_pin) ? true : false;
    return (pin_set == (BUTTONS_ACTIVE_STATE ? true : false));
}

void bsp_board_buttons_init(void)
{
   // Configure HR_INC_BUTTON_PIN_NO and HR_DEC_BUTTON_PIN_NO as wake up buttons and also configure
    // for 'pull up' because the eval board does not have external pull up resistors connected to
    // the buttons.
    static app_button_cfg_t buttons[] =
    {
        {BUTTON_ELEC_LOCK_PIN_NO, BUTTONS_ACTIVE_STATE, BUTTON_PULL, bsp_button_event_handler},
        {BUTTON_ELEC_LOCK_PIN_NO, BUTTONS_ACTIVE_STATE, BUTTON_PULL, bsp_button_event_handler}, // Note: This pin is also BONDMNGR_DELETE_BUTTON_PIN_NO
        {BUTTON_UV_LAMP_DOOR_PIN_NO, BUTTONS_ACTIVE_STATE, BUTTON_PULL, bsp_button_event_handler},
			  {BUTTON_COIN_BOX_PIN_NO, BUTTONS_ACTIVE_STATE, BUTTON_PULL, bsp_button_event_handler},
		};
    
    APP_BUTTON_INIT(buttons, sizeof(buttons) / sizeof(buttons[0]), APP_TIMER_TICKS(BUTTON_DETECTION_DELAY,smart_locker_local_prescal), false); 
}

#endif //BUTTONS_NUMBER > 0

static void bsp_sls_env_value_init(void)
{
	 uv_lamp_cmd          = CONTROL_CMD_STOP;
   fan_negative_ion_cmd = CONTROL_CMD_STOP;
   elec_lock_cmd        = CONTROL_CMD_STOP;

   uv_lamp_status          = DEVICE_STATUS_STOP;
   uv_lamp_door_status     = DEVICE_STATUS_STOP;
   fan_negative_ion_status = DEVICE_STATUS_STOP;
   elec_lock_status        = DEVICE_STATUS_STOP;
}
	
uint32_t bsp_smart_locker_board_init(uint32_t prescale)
{
   uint32_t err_code;
   smart_locker_local_prescal=prescale;
	
   bsp_sls_env_value_init();
	 bsp_board_leds_init();
	 bsp_board_switchs_init();
	 bsp_board_buttons_init();
   err_code=app_timer_create(&lock_wait_timer_id,APP_TIMER_MODE_SINGLE_SHOT,bsp_open_elec_lock_wait_timeout_handler);
   RETURN_ON_ERROR_NOT_INVALID_PARAM(err_code);
   err_code=app_timer_create(&sys_run_led_timer_id,APP_TIMER_MODE_REPEATED,bsp_sys_run_led_timeout_handler);
   RETURN_ON_ERROR_NOT_INVALID_PARAM(err_code);
	 app_timer_start(sys_run_led_timer_id,APP_TIMER_TICKS(SYS_RUN_LED_INTERVAL_IN_MS,smart_locker_local_prescal),NULL);
   return err_code;
  
}

static void bsp_button_event_handler(uint8_t pin_no, uint8_t button_action)
{
   if( button_action== APP_BUTTON_PUSH )
	{
         switch (pin_no)
        {
			    case BUTTON_ELEC_LOCK_PIN_NO :
            elec_lock_status=DEVICE_STATUS_RUN;
            DEBUG_INFO("\r\nelec lock is locked!");
          break;					
					case BUTTON_UV_LAMP_DOOR_PIN_NO:
					  uv_lamp_door_status=DEVICE_STATUS_RUN;
				   if(uv_lamp_control_cmd==CONTROL_CMD_RUN)
				    bsp_open_uv_lamp_and_indicator_led();
            DEBUG_INFO("\r\nlock door is closed!");
					break;
					case BUTTON_COIN_BOX_PIN_NO :
						  DEBUG_INFO("\r\ncoin box get low!");
          break;						
          default:
          APP_ERROR_HANDLER(pin_no);
          break;
				}
	}
	else if( button_action == APP_BUTTON_RELEASE )
	{
        switch (pin_no)
        {
			    case BUTTON_ELEC_LOCK_PIN_NO :
            elec_lock_status=DEVICE_STATUS_STOP;
            DEBUG_INFO("\r\nelec lock is opened!");
          break;					
					case BUTTON_UV_LAMP_DOOR_PIN_NO:
					  uv_lamp_door_status=DEVICE_STATUS_STOP;
				   if(uv_lamp_control_cmd==CONTROL_CMD_RUN)
				    bsp_open_uv_lamp_and_indicator_led();
            DEBUG_INFO("\r\nlock door is open!");
					break;
					case BUTTON_COIN_BOX_PIN_NO :
						  DEBUG_INFO("\r\ncoin box get high!");
          break;						
          default:
          APP_ERROR_HANDLER(pin_no);
          break;
				}
		}
}
void bsp_btn_smart_locker_evt_handler_callback()//智能储物柜的事件处理
{
  
	
}     


 void bsp_cb_on_uv_lamp_cmd_write(ble_sls_t * p_sls, ble_gatts_evt_write_t * p_evt_write)
{
	DEBUG_INFO("\r\nwrite uv_lamp cmd->");
	DEBUG_INFO("len:");
	DEBUG_INFO(uint8_to_string(p_evt_write->len));
  DEBUG_INFO("cmd:");
	DEBUG_INFO(uint8_to_string(*p_evt_write->data));
	DEBUG_INFO("\r\ncurrent uv_lamp cmd->");
	DEBUG_INFO(uint8_to_string(uv_lamp_cmd));
	
	if(uv_lamp_cmd==CONTROL_CMD_RUN)
	{
		
	bsp_open_uv_lamp_and_indicator_led();	
  DEBUG_INFO("\r\nopen uv_lamp!");
		
	}
	else if(uv_lamp_cmd==CONTROL_CMD_STOP)
	{
		bsp_close_uv_lamp_and_indicator_led();
    DEBUG_INFO("\r\nclose uv_lamp!");		
	}
	else
	{
		DEBUG_INFO("\r\nuv_lamp cmd err!");
	}
}

 void bsp_cb_on_fan_negative_ion_cmd_write(ble_sls_t * p_sls, ble_gatts_evt_write_t * p_evt_write)
{
	DEBUG_INFO("write fan cmd:");
	DEBUG_INFO("len:");
	DEBUG_INFO(uint8_to_string(p_evt_write->len));
  DEBUG_INFO("cmd:");
	DEBUG_INFO(uint8_to_string(*p_evt_write->data));
	DEBUG_INFO("\r\ncurrent fan cmd->");
	DEBUG_INFO(uint8_to_string(fan_negative_ion_cmd));
	
		if(fan_negative_ion_cmd==CONTROL_CMD_RUN)
	{
		
	bsp_open_fan_negative_ion_and_indicator_led();	
  DEBUG_INFO("\r\nopen fan_ngt_ion!");
		
	}
	else if(uv_lamp_cmd==CONTROL_CMD_STOP)
	{
	  bsp_close_fan_negative_ion_and_indicator_led();
    DEBUG_INFO("\r\nclose fan_ngt_ion!");		
	}
	else
	{
		DEBUG_INFO("\r\nfan_ngt_ion cmd err!");
	}
}
 void bsp_cb_on_elec_lock_cmd_write(ble_sls_t * p_sls, ble_gatts_evt_write_t * p_evt_write)
{
  DEBUG_INFO("write elec_lock cmd:");
	DEBUG_INFO("len:");
	DEBUG_INFO(uint8_to_string(p_evt_write->len));
  DEBUG_INFO("cmd:");
	DEBUG_INFO(uint8_to_string(*p_evt_write->data));
	DEBUG_INFO("\r\ncurrent elec_cmd cmd->");
	DEBUG_INFO(uint8_to_string(elec_lock_cmd));
	
	if(elec_lock_cmd==CONTROL_CMD_RUN)
	{
		
	 bsp_open_elec_lock();	
   DEBUG_INFO("\r\nopen elec_lock!");
		
	}
	else
	{
		DEBUG_INFO("\r\nelec_lock cmd err!");
	}
		  
}


static void bsp_open_uv_lamp_and_indicator_led(void)
{
   bsp_board_switch_on(SWITCH_UV_LAMP_PIN_NO);
   bsp_board_led_on(LED_UV_LAMP_PIN_NO);   
   uv_lamp_status=DEVICE_STATUS_RUN;
}
static void bsp_close_uv_lamp_and_indicator_led(void)
{
   bsp_board_switch_off(SWITCH_UV_LAMP_PIN_NO);
   bsp_board_led_off(LED_UV_LAMP_PIN_NO); 
   uv_lamp_status=DEVICE_STATUS_STOP;
}
static void bsp_open_fan_negative_ion_and_indicator_led(void)
{
   bsp_board_switch_on(SWITCH_FAN_NEGATIVE_ION_PIN_NO);
   bsp_board_led_on(LED_NEGATIVE_ION_PIN_NO);
   fan_negative_ion_status=DEVICE_STATUS_RUN;
}
static void bsp_close_fan_negative_ion_and_indicator_led(void)
{
   bsp_board_switch_off(SWITCH_FAN_NEGATIVE_ION_PIN_NO);
   bsp_board_led_off(LED_NEGATIVE_ION_PIN_NO);
   fan_negative_ion_status=DEVICE_STATUS_STOP;
}
static void bsp_open_elec_lock(void)
{
	static uint8_t open_try_times=DEFAULT_OPEN_LOCK_TRY_TIMES;
  if(elec_lock_status==DEVICE_STATUS_RUN && open_try_times>0)
  {
	 open_try_times--;
   bsp_board_switch_on(SWITCH_ELEC_LOCK_PIN_NO);
   app_timer_start(lock_wait_timer_id,APP_TIMER_TICKS(LOCK_WAIT_TIMEOUT_MS,smart_locker_local_prescal),NULL);
	}
	else
	{
		open_try_times=DEFAULT_OPEN_LOCK_TRY_TIMES;
	}
}
static void bsp_open_elec_lock_wait_timeout_handler(void * p_context)
{
	 UNUSED_PARAMETER(p_context);
   bsp_board_switch_off(SWITCH_ELEC_LOCK_PIN_NO);
	 bsp_open_elec_lock();//再次尝试开锁
}

static void bsp_sys_run_led_timeout_handler(void * p_context)
{
	bsp_board_led_invert(LED_SYS_RUN_PIN_NO);
}

/******uint8 --->0x string***/
uint8_t *uint8_to_string(uint8_t src)
{
#if UART_DEBUG > 0
  uint8_t       i;
  uint8_t     dst[1];
  uint8_t*pAddr =dst;
  dst[0]=src;
  
  uint8_t        hex[] = "0123456789ABCDEF";
  static uint8_t str[5+2];
  uint8_t        *pStr = str;
  
  *pStr++ = '0';
  *pStr++ = 'x';
  
  // Start from end of addr
  pAddr += 1;
  
  for ( i = 1; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }
  
  //*pStr = 0;
  str[4]='\r';
  str[5]='\n';
  str[6]=0;
 return str;
#else
  (void)src;
  return (char*)0;
#endif
}

