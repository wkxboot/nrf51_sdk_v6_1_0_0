#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "boards.h"
#include "nordic_common.h"
#include "simple_uart.h"
#include "bsp_btn_smart_locker.h"

#ifndef BSP_SIMPLE
#include "app_timer.h"
#include "app_button.h"
#endif // BSP_SIMPLE



app_timer_id_t lock_wait_timer_id;//开电磁锁脉冲延时
app_timer_id_t sys_run_led_timer_id;//系统运行灯

 uint32_t uv_lamp_exec_time_sec;
 uint32_t fan_negative_ion_exec_time_sec;
 uint32_t elec_lock_exec_times;


 uint8_t uv_lamp_status=DEVICE_STATUS_STOP;
 uint8_t fan_negative_ion_status=DEVICE_STATUS_STOP;
 uint8_t elec_lock_status=DEVICE_STATUS_STOP;
 uint8_t uv_lamp_door_status=DEVICE_STATUS_STOP;

 uint8_t uv_lamp_control_cmd=CONTROL_CMD_STOP;
 uint8_t elec_lock_control_cmd=CONTROL_CMD_STOP;

static uint32_t smart_locker_local_prescal=0;

static void bsp_open_elec_lock_wait_timeout_handler(void * p_context);
static void bsp_open_uv_lamp_and_indicator_led(void);
static void bsp_close_uv_lamp_and_indicator_led(void);
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
/*
uint32_t bsp_board_led_idx_to_pin(uint32_t led_idx)
{
    ASSERT(led_idx < LEDS_NUMBER);
    return m_board_led_list[led_idx];
}

uint32_t bsp_board_pin_to_led_idx(uint32_t pin_number)
{
    uint32_t ret = 0xFFFFFFFF;
    uint32_t i;
    for(i = 0; i < LEDS_NUMBER; ++i)
    {
        if (m_board_led_list[i] == pin_number)
        {
            ret = i;
            break;
        }
    }
    return ret;
}
*/
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
/*
uint32_t bsp_board_switch_idx_to_pin(uint32_t switch_idx)
{
    ASSERT(switch_idx < SWITCHS_NUMBER);
    return m_board_sw_list[switch_idx];
}

uint32_t bsp_board_pin_to_switch_idx(uint32_t pin_number)
{
    uint32_t ret = 0xFFFFFFFF;
    uint32_t i;
    for(i = 0; i < SWITCHS_NUMBER; ++i)
    {
        if (m_board_sw_list[i] == pin_number)
        {
            ret = i;
            break;
        }
    }
    return ret;
}
*/
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
/*
uint32_t bsp_board_pin_to_button_idx(uint32_t pin_number)
{
    uint32_t i;
    uint32_t ret = 0xFFFFFFFF;
    for(i = 0; i < BUTTONS_NUMBER; ++i)
    {
        if (m_board_btn_list[i] == pin_number)
        {
            ret = i;
            break;
        }
    }
    return ret;
}

uint32_t bsp_board_button_idx_to_pin(uint32_t button_idx)
{
    ASSERT(button_idx < BUTTONS_NUMBER);
    return m_board_btn_list[button_idx];
}
*/
#endif //BUTTONS_NUMBER > 0


uint32_t bsp_smart_locker_board_init(uint32_t prescale)
{
   uint32_t err_code;
   smart_locker_local_prescal=prescale;
   
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
////给所有按键（检测引脚）的行为分配相应的事件
//uint32_t smart_locker_buttons_configure()
//{
//    uint32_t err_code;

//    err_code = bsp_event_to_button_action_assign(BUTTON_ID_ELEC_LOCK,
//                                                 BSP_BUTTON_ACTION_PUSH,
//                                                 BSP_USER_EVT_ELEC_LOCK_IS_LOCKED);//电磁锁锁住事件
//    RETURN_ON_ERROR_NOT_INVALID_PARAM(err_code);
//    err_code = bsp_event_to_button_action_assign(BUTTON_ID_ELEC_LOCK,
//                                                 BSP_BUTTON_ACTION_RELEASE,
//                                                 BSP_USER_EVT_ELEC_LOCK_IS_UNLOCKED);//电磁锁释放事件
//    RETURN_ON_ERROR_NOT_INVALID_PARAM(err_code);
//    err_code = bsp_event_to_button_action_assign(BUTTON_ID_UV_LAMP_DOOR,
//                                                 BSP_BUTTON_ACTION_PUSH,
//                                                 BSP_USER_EVT_UV_LAMP_DOOR_IS_CLOSED);//消毒柜的门关闭事件
//    RETURN_ON_ERROR_NOT_INVALID_PARAM(err_code);
//    err_code = bsp_event_to_button_action_assign(BUTTON_ID_UV_LAMP_DOOR,
//                                                 BSP_BUTTON_ACTION_RELEASE,
//                                                 BSP_USER_EVT_UV_LAMP_DOOR_IS_OPENED);//消毒柜的门打开事件
//    RETURN_ON_ERROR_NOT_INVALID_PARAM(err_code);
//    err_code = bsp_event_to_button_action_assign(BUTTON_ID_COIN_BOX,
//                                                 BSP_BUTTON_ACTION_PUSH,
//                                                 BSP_USER_EVT_COIN_BOX_GET_COIN);//投币器得到一个硬币事件
//    RETURN_ON_ERROR_NOT_INVALID_PARAM(err_code);

//    return NRF_SUCCESS;
//}
static void bsp_button_event_handler(uint8_t pin_no, uint8_t button_action)
{
   if( button_action== APP_BUTTON_PUSH )
	{
         switch (pin_no)
        {
			    case BUTTON_ELEC_LOCK_PIN_NO :
            elec_lock_status=DEVICE_STATUS_RUN;
            simple_uart_putstring("elec lock is locked\r\n");
          break;					
					case BUTTON_UV_LAMP_DOOR_PIN_NO:
					  uv_lamp_door_status=DEVICE_STATUS_RUN;
				   if(uv_lamp_control_cmd==CONTROL_CMD_RUN)
				    bsp_open_uv_lamp_and_indicator_led();
            simple_uart_putstring("lock door is closed\r\n");
					break;
					case BUTTON_COIN_BOX_PIN_NO :
						  simple_uart_putstring("coin box get low\r\n");
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
            simple_uart_putstring("elec lock is opened\r\n");
          break;					
					case BUTTON_UV_LAMP_DOOR_PIN_NO:
					  uv_lamp_door_status=DEVICE_STATUS_STOP;
				   if(uv_lamp_control_cmd==CONTROL_CMD_RUN)
				    bsp_open_uv_lamp_and_indicator_led();
            simple_uart_putstring("lock door is open\r\n");
					break;
					case BUTTON_COIN_BOX_PIN_NO :
						  simple_uart_putstring("coin box get high\r\n");
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


//static void bsp_btn_smart_locker_process_coin_evt()
//{
//    if(coin_cnt==0)
//    {
//    app_timer_start(coin_box_timer_id,APP_TIMER_TICKS(COIN_BOX_WAIT_TIMEOUT_SEC*1000, smart_locker_local_prescal),NULL);  
//    }
//    if(coin_cnt<DEFAULT_MAX_COIN_CNT)
//      coin_cnt++;
//    else
//      coin_cnt_increase_max_process();
//}


//static void coin_cnt_increase_max_process()
//{
//  app_timer_stop(coin_box_timer_id);
//  coin_box_timer_handler(NULL);
//}

//
//static void sys_cmd_handler(void * p_context)
//{
//
//
//}

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
 void bsp_open_fan_negative_ion_and_indicator_led(void)
{
   bsp_board_switch_on(SWITCH_FAN_NEGATIVE_ION_PIN_NO);
   bsp_board_led_on(LED_NEGATIVE_ION_PIN_NO);
   fan_negative_ion_status=DEVICE_STATUS_RUN;
}
 void bsp_close_fan_negative_ion_and_indicator_led(void)
{
   bsp_board_switch_off(SWITCH_FAN_NEGATIVE_ION_PIN_NO);
   bsp_board_led_off(LED_NEGATIVE_ION_PIN_NO);
   fan_negative_ion_status=DEVICE_STATUS_STOP;
}
void bsp_open_elec_lock(void)
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
//static void coin_box_timer_handler(void * p_context)
//{
//   UNUSED_PARAMETER(p_context);
//   
//   switch(coin_cnt)
//   {
//   case 3:
//     uv_lamp_exec_time_sec+=UV_LAMP_RUN_SECONDS_PER_TIMES;
//     break;
//   case 2:
//    fan_negative_ion_exec_time_sec+=FAN_NEGATIVE_ION_RUN_SECONDS_PER_TIMES;
//     break;
//   case 1:
//     elec_lock_exec_times=3;
//     break;
//   default:
//     break;    
//    }  
//   coin_cnt=0;
//   }

//static void bsp_smart_locker_control_timer_handler(void * p_context)
//{

//  if((uv_lamp_exec_time_sec > 0 && uv_lamp_door_status == DEVICE_STATUS_RUN)|| uv_lamp_exec_control_cmd == CONTROL_CMD_RUN)
//  {  
//    if(uv_lamp_status==DEVICE_STATUS_STOP)
//    bsp_open_uv_lamp_and_indicator_led();
//    
//    if(uv_lamp_exec_time_sec>CONTROL_TIMER_TIMEOUT_SEC)
//    uv_lamp_exec_time_sec-=CONTROL_TIMER_TIMEOUT_SEC;
//    else
//    uv_lamp_exec_time_sec=0;    
//  }
//  else
//  {
//    if(uv_lamp_status==DEVICE_STATUS_RUN)
//    bsp_close_uv_lamp_and_indicator_led();
//  }

//    if(fan_negative_ion_exec_time_sec>0 || fan_negative_ion_exec_control_cmd == CONTROL_CMD_RUN)
//  {  
//    if(fan_negative_ion_status==DEVICE_STATUS_STOP)
//    bsp_open_fan_negative_ion_and_indicator_led();
//    
//    if(fan_negative_ion_exec_time_sec>CONTROL_TIMER_TIMEOUT_SEC)
//    fan_negative_ion_exec_time_sec-=CONTROL_TIMER_TIMEOUT_SEC;
//    else
//    fan_negative_ion_exec_time_sec=0;    
//  }
//  else
//  {
//    if(fan_negative_ion_status==DEVICE_STATUS_RUN)
//    bsp_close_fan_negative_ion_and_indicator_led();
//  }
//  
//    if(elec_lock_exec_times>0 || elec_lock_exec_control_cmd == CONTROL_CMD_RUN)
//  {  
//    if(elec_lock_status==DEVICE_STATUS_RUN)
//    bsp_open_elec_lock();
//    
//    if(elec_lock_exec_times>0)
//    elec_lock_exec_times--;   
//  }
//  else
//  {
//    
//  }

//}
