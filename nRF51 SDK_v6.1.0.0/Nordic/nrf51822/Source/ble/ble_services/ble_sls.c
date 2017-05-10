/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/* Attention! 
*  To maintain compliance with Nordic Semiconductor ASA抯 Bluetooth profile 
*  qualification listings, this section of source code must not be modified.
*/

#include <string.h>
#include "nordic_common.h"
#include "ble_l2cap.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "simple_uart.h"
#include "ble_sls.h"
#include "bsp_btn_smart_locker.h"



#define OPCODE_LENGTH 1     /**< Length of opcode inside Heart Rate Measurement packet. */
#define HANDLE_LENGTH 2     /**< Length of handle inside Heart Rate Measurement packet. */
#define MAX_SLS_LEN        (BLE_L2CAP_MTU_DEF - OPCODE_LENGTH - HANDLE_LENGTH)  /**< Maximum size of a transmitted Heart Rate Measurement. */

#define BLE_UUID_SMART_LOCKER_SERVICE          0xFFEE
#define BLE_UUID_UV_LAMP_CHAR                  0xFF01
#define BLE_UUID_UV_LAMP_DOOR_CHAR             0xFF02
#define BLE_UUID_FAN_NEGATIVE_ION_CHAR         0xFF03
#define BLE_UUID_ELEC_LOCK_CHAR                0xFF04
         

uint8_t uv_lamp_cmd[MAX_SLS_LEN],uv_lamp_door_cmd[MAX_SLS_LEN],fan_negative_ion_cmd[MAX_SLS_LEN],elec_lock_cmd[MAX_SLS_LEN];//属性内的数据值

extern   uint8_t uv_lamp_status;
extern   uint8_t fan_negative_ion_status;
extern   uint8_t elec_lock_status;
extern   uint8_t uv_lamp_door_status;

static void on_uv_lamp_cccd_write(ble_sls_t * p_sls, ble_gatts_evt_write_t * p_evt_write);
static void on_uv_lamp_value_write(ble_sls_t * p_sls, ble_gatts_evt_write_t * p_evt_write);
static void on_uv_lamp_door_value_write(ble_sls_t * p_sls, ble_gatts_evt_write_t * p_evt_write);
static void on_fan_negative_ion_value_write(ble_sls_t * p_sls, ble_gatts_evt_write_t * p_evt_write);
static void on_elec_lock_value_write(ble_sls_t * p_sls, ble_gatts_evt_write_t * p_evt_write);
/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_sls       Heart Rate Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_sls_t * p_sls, ble_evt_t * p_ble_evt)
{
    p_sls->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_sls       Heart Rate Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_sls_t * p_sls, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_sls->conn_handle = BLE_CONN_HANDLE_INVALID;
}



/**@brief Function for handling the Write event.
 *
 * @param[in]   p_sls       Heart Rate Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_sls_t * p_sls, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;


		   if (p_evt_write->handle == p_sls->fan_negative_ion_handles.value_handle)
    {
        on_fan_negative_ion_value_write(p_sls, p_evt_write);
    }
		else if (p_evt_write->handle == p_sls->uv_lamp_handles.cccd_handle)
    {
        on_uv_lamp_cccd_write(p_sls, p_evt_write);
    }
		   else if (p_evt_write->handle == p_sls->uv_lamp_handles.value_handle)
    {
        on_uv_lamp_value_write(p_sls, p_evt_write);
    }
			  else if (p_evt_write->handle == p_sls->uv_lamp_handles.value_handle)
    {
        on_uv_lamp_door_value_write(p_sls, p_evt_write);
    }
		    else if (p_evt_write->handle == p_sls->elec_lock_handles.value_handle)
    {
        on_elec_lock_value_write(p_sls, p_evt_write);
    }
}

/**@brief Function for handling write events to the Heart Rate Measurement characteristic.
 *
 * @param[in]   p_sls         Heart Rate Service structure.
 * @param[in]   p_evt_write   Write event received from the BLE stack.
 */
static void on_uv_lamp_cccd_write(ble_sls_t * p_sls, ble_gatts_evt_write_t * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // CCCD written, update notification state
        if (p_sls->evt_handler != NULL)
        {
            ble_sls_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = ble_sls_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = ble_sls_EVT_NOTIFICATION_DISABLED;
            }

            p_sls->evt_handler(p_sls, &evt);
        }
    }
}
static void on_uv_lamp_value_write(ble_sls_t * p_sls, ble_gatts_evt_write_t * p_evt_write)
{
	simple_uart_putstring("write uv lamp:");
	for(uint8_t i=0;i<p_evt_write->len;i++)
	simple_uart_put(*(p_evt_write->data+i));
}
static void on_uv_lamp_door_value_write(ble_sls_t * p_sls, ble_gatts_evt_write_t * p_evt_write)
{
		simple_uart_putstring("write uv lamp door:");
	for(uint8_t i=0;i<p_evt_write->len;i++)
	simple_uart_put(*(p_evt_write->data+i));
}
static void on_fan_negative_ion_value_write(ble_sls_t * p_sls, ble_gatts_evt_write_t * p_evt_write)
{
		simple_uart_putstring("write fan:");
	for(uint8_t i=0;i<p_evt_write->len;i++)
	simple_uart_put(*(p_evt_write->data+i));
}
static void on_elec_lock_value_write(ble_sls_t * p_sls, ble_gatts_evt_write_t * p_evt_write)
{
		simple_uart_putstring("write elec lock:");
	for(uint8_t i=0;i<p_evt_write->len;i++)
	simple_uart_put(*(p_evt_write->data+i));
}



void ble_sls_on_ble_evt(ble_sls_t * p_sls, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_sls, p_ble_evt);
				    simple_uart_putstring("connected!\r\n");
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_sls, p_ble_evt);
				simple_uart_putstring("disconnected!\r\n");
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_sls, p_ble_evt);
				simple_uart_putstring("write!\r\n");
            break;

        default:
            // No implementation needed.
            break;
    }
}



/**@brief Function for adding the Heart Rate Measurement characteristic.
 *
 * @param[in]   p_sls        Heart Rate Service structure.
 * @param[in]   p_sls_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t uv_lamp_char_add(ble_sls_t            * p_sls,
                                                const ble_sls_init_t * p_sls_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    //cccd_md.write_perm = p_sls_init->sls_hrm_attr_md.cccd_write_perm;
	  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.notify = 1;
	  char_md.char_props.read  = 1;
	  char_md.char_props.write = 1;
	  char_md.char_user_desc_max_size=16;
	  char_md.char_user_desc_size=sizeof("uv_lamp_value");
	  char_md.p_char_user_desc=(uint8_t*)"uv_lamp_value";
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_UV_LAMP_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));

    //attr_md.read_perm  = p_sls_init->sls_hrm_attr_md.read_perm;
    //attr_md.write_perm = p_sls_init->sls_hrm_attr_md.write_perm;
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_USER;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = MAX_SLS_LEN;//hrm_encode(p_sls, INITIAL_VALUE_HRM, encoded_initial_hrm);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = MAX_SLS_LEN;
    attr_char_value.p_value   =uv_lamp_cmd;

    return sd_ble_gatts_characteristic_add(p_sls->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_sls->uv_lamp_handles);
}

static uint32_t uv_lamp_door_char_add(ble_sls_t * p_sls, const ble_sls_init_t * p_sls_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.notify = 1;
    char_md.char_props.read  = 1;
	  char_md.char_props.write = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;
    char_md.p_cccd_md        = NULL;
    char_md.p_sccd_md        = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_UV_LAMP_DOOR_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));

		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
		//BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    //attr_md.read_perm  = p_sls_init->sls_bsl_attr_md.read_perm;
    //attr_md.write_perm = p_sls_init->sls_bsl_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_USER;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = MAX_SLS_LEN;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = MAX_SLS_LEN;
    attr_char_value.p_value   =uv_lamp_door_cmd;

    return sd_ble_gatts_characteristic_add(p_sls->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_sls->uv_lamp_door_handles);
}
/**@brief Function for adding the Body Sensor Location characteristic.
 *
 * @param[in]   p_sls        Heart Rate Service structure.
 * @param[in]   p_sls_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t fan_negative_ion_char_add(ble_sls_t * p_sls, const ble_sls_init_t * p_sls_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read  = 1;
	  char_md.char_props.write = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;
    char_md.p_cccd_md        = NULL;
    char_md.p_sccd_md        = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_FAN_NEGATIVE_ION_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));

		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    //attr_md.read_perm  = p_sls_init->sls_bsl_attr_md.read_perm;
    //attr_md.write_perm = p_sls_init->sls_bsl_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_USER;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = MAX_SLS_LEN;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = MAX_SLS_LEN;
    attr_char_value.p_value   =fan_negative_ion_cmd;// p_sls_init->p_body_sensor_location;

    return sd_ble_gatts_characteristic_add(p_sls->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_sls->fan_negative_ion_handles);
}

/**@brief Function for adding the elec_lock characteristic.
 *
 * @param[in]   p_sls        smart locker Service structure.
 * @param[in]   p_sls_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t elec_lock_char_add(ble_sls_t * p_sls, const ble_sls_init_t * p_sls_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read  = 1;
	  char_md.char_props.write = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;
    char_md.p_cccd_md        = NULL;
    char_md.p_sccd_md        = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_ELEC_LOCK_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));

		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    //attr_md.read_perm  = p_sls_init->sls_bsl_attr_md.read_perm;
    //attr_md.write_perm = p_sls_init->sls_bsl_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_USER;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 2;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = MAX_SLS_LEN;
    attr_char_value.p_value   =elec_lock_cmd;// p_sls_init->p_body_sensor_location;

    return sd_ble_gatts_characteristic_add(p_sls->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_sls->elec_lock_handles);
}

uint32_t ble_sls_init(ble_sls_t * p_sls, const ble_sls_init_t * p_sls_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_sls->evt_handler                 = p_sls_init->evt_handler;

    p_sls->conn_handle                 = BLE_CONN_HANDLE_INVALID;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_SMART_LOCKER_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_sls->service_handle);

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add heart rate measurement characteristic
    err_code = uv_lamp_char_add(p_sls, p_sls_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    // Add heart rate measurement characteristic
    err_code = uv_lamp_door_char_add(p_sls, p_sls_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
       // Add body sensor location characteristic
      err_code =fan_negative_ion_char_add(p_sls, p_sls_init);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
       err_code =elec_lock_char_add(p_sls, p_sls_init);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }

    return NRF_SUCCESS;
}
