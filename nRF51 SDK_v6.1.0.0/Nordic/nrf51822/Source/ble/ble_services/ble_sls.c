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
*  To maintain compliance with Nordic Semiconductor ASA’s Bluetooth profile 
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


#define MAX_SLS_LEN        (BLE_L2CAP_MTU_DEF - OPCODE_LENGTH - HANDLE_LENGTH)  /**< Maximum size of a transmitted Heart Rate Measurement. */

#define BLE_UUID_SMART_LOCKER_SERVICE          0xFFEE
#define BLE_UUID_UV_LAMP_CMD_CHAR              0xFF01
#define BLE_UUID_UV_LAMP_STATUS_CHAR           0xFF02
#define BLE_UUID_UV_LAMP_DOOR_STATUS_CHAR      0xFF03
#define BLE_UUID_FAN_NEGATIVE_ION_CMD_CHAR     0xFF04
#define BLE_UUID_FAN_NEGATIVE_ION_STATUS_CHAR  0xFF05
#define BLE_UUID_ELEC_LOCK_CMD_CHAR            0xFF06
#define BLE_UUID_ELEC_LOCK_STATUS_CHAR         0xFF07


#define MAX_SLS_CMD_LEN                        1
#define MAX_SLS_STATUS_LEN                     1
#define MAX_SLS_USER_DESC_LEN                  20


uint8_t uv_lamp_cmd;//[MAX_SLS_CMD_LEN];
uint8_t fan_negative_ion_cmd;//[MAX_SLS_CMD_LEN];
uint8_t elec_lock_cmd;//[MAX_SLS_CMD_LEN];

uint8_t uv_lamp_status;
uint8_t uv_lamp_door_status;
uint8_t fan_negative_ion_status;
uint8_t elec_lock_status;


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_sls       Smart Locker Service structure..
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_sls_t * p_sls, ble_evt_t * p_ble_evt)
{
    p_sls->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_sls       Smart Locker Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_sls_t * p_sls, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_sls->conn_handle = BLE_CONN_HANDLE_INVALID;
}



/**@brief Function for handling the Write event.
 *
 * @param[in]   p_sls       smart locker Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_sls_t * p_sls, ble_evt_t * p_ble_evt)
{
     ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

		   if (p_evt_write->handle == p_sls->fan_negative_ion_cmd_handles.value_handle)
    {
        bsp_cb_on_fan_negative_ion_cmd_write(p_sls, p_evt_write);
    }
		   else if (p_evt_write->handle == p_sls->uv_lamp_cmd_handles.value_handle)
    {
        bsp_cb_on_uv_lamp_cmd_write(p_sls, p_evt_write);
    }
		   else if (p_evt_write->handle == p_sls->elec_lock_cmd_handles.value_handle)
    {
        bsp_cb_on_elec_lock_cmd_write(p_sls, p_evt_write);
    }
}

///**@brief Function for handling write events to the Heart Rate Measurement characteristic.
// *
// * @param[in]   p_sls         Heart Rate Service structure.
// * @param[in]   p_evt_write   Write event received from the BLE stack.
// */
//static void on_uv_lamp_cccd_write(ble_sls_t * p_sls, ble_gatts_evt_write_t * p_evt_write)
//{
//    if (p_evt_write->len == 2)
//    {
//        // CCCD written, update notification state
//        if (p_sls->evt_handler != NULL)
//        {
//            ble_sls_evt_t evt;

//            if (ble_srv_is_notification_enabled(p_evt_write->data))
//            {
//                evt.evt_type = ble_sls_EVT_NOTIFICATION_ENABLED;
//            }
//            else
//            {
//                evt.evt_type = ble_sls_EVT_NOTIFICATION_DISABLED;
//            }

//            p_sls->evt_handler(p_sls, &evt);
//        }
//    }
//}


void ble_sls_on_ble_evt(ble_sls_t * p_sls, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_sls, p_ble_evt);
				    DEBUG_INFO("\r\nconnected!");
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_sls, p_ble_evt);
				    DEBUG_INFO("\r\ndisconnected!");
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_sls, p_ble_evt);
				    DEBUG_INFO("\r\nwrited!");
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
static uint32_t uv_lamp_cmd_char_add(ble_sls_t            * p_sls,
                                     const ble_sls_init_t * p_sls_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
	
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.notify = 1;
	  char_md.char_props.read  = 1;
	  char_md.char_props.write = 1;
	  char_md.char_user_desc_max_size=16;
	  char_md.char_user_desc_size=sizeof("uv_lamp_cmd");
	  char_md.p_char_user_desc=(uint8_t*)"uv_lamp_cmd";
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_UV_LAMP_CMD_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));

		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_USER;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = MAX_SLS_CMD_LEN;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = MAX_SLS_CMD_LEN;
    attr_char_value.p_value   =(uint8_t*)&uv_lamp_cmd;

    return sd_ble_gatts_characteristic_add(p_sls->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_sls->uv_lamp_cmd_handles);
}

static uint32_t uv_lamp_status_char_add(ble_sls_t            * p_sls,
                                        const ble_sls_init_t * p_sls_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.notify = 1;
	  char_md.char_props.read  = 1;
	  //char_md.char_props.write  = 1;
	  char_md.char_user_desc_max_size=MAX_SLS_USER_DESC_LEN;
	  char_md.char_user_desc_size=sizeof("uv_lamp_status");
	  char_md.p_char_user_desc=(uint8_t*)"uv_lamp_status";
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_UV_LAMP_STATUS_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));

		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
		//BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_USER;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = MAX_SLS_STATUS_LEN;//hrm_encode(p_sls, INITIAL_VALUE_HRM, encoded_initial_hrm);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = MAX_SLS_STATUS_LEN;
    attr_char_value.p_value   =(uint8_t*)&uv_lamp_status;

    return sd_ble_gatts_characteristic_add(p_sls->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_sls->uv_lamp_status_handles);
}

static uint32_t uv_lamp_door_status_char_add(ble_sls_t            * p_sls,
                                           	 const ble_sls_init_t * p_sls_init)
{
    ble_gatts_char_md_t char_md;
	  ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.notify = 1;
    char_md.char_props.read  = 1;
	  //char_md.char_props.write = 1;
	
		char_md.char_user_desc_max_size=MAX_SLS_USER_DESC_LEN;
	  char_md.char_user_desc_size=sizeof("uv_lamp_door_status");
	  char_md.p_char_user_desc=(uint8_t*)"uv_lamp_door_status";
    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;
    char_md.p_cccd_md        = NULL;
    char_md.p_sccd_md        = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_UV_LAMP_DOOR_STATUS_CHAR);
    //cccd cfg
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&attr_md, 0, sizeof(attr_md));
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	  //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_USER;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = MAX_SLS_STATUS_LEN;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = MAX_SLS_STATUS_LEN;
    attr_char_value.p_value   =(uint8_t*)&uv_lamp_door_status;

    return sd_ble_gatts_characteristic_add(p_sls->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_sls->uv_lamp_door_status_handles);
}
/**@brief Function for adding the Body Sensor Location characteristic.
 *
 * @param[in]   p_sls        Heart Rate Service structure.
 * @param[in]   p_sls_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t fan_negative_ion_cmd_char_add(ble_sls_t              * p_sls,
                                            	const ble_sls_init_t   * p_sls_init)
{
    ble_gatts_char_md_t char_md;
		ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));
	
    char_md.char_props.notify= 1;
    char_md.char_props.read  = 1;
	  char_md.char_props.write = 1;
		char_md.char_user_desc_max_size=MAX_SLS_USER_DESC_LEN;
	  char_md.char_user_desc_size=sizeof("fan_ngt_ion_cmd");
	  char_md.p_char_user_desc=(uint8_t*)"fan_ngt_ion_cmd";

    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;
    char_md.p_cccd_md        = NULL;
    char_md.p_sccd_md        = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_FAN_NEGATIVE_ION_CMD_CHAR);
    //cccd cfg
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&attr_md, 0, sizeof(attr_md));
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_USER;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = MAX_SLS_CMD_LEN;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = MAX_SLS_CMD_LEN;
    attr_char_value.p_value   =(uint8_t*)&fan_negative_ion_cmd;

    return sd_ble_gatts_characteristic_add(p_sls->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_sls->fan_negative_ion_cmd_handles);
}
static uint32_t fan_negative_ion_status_char_add(ble_sls_t              * p_sls,
                                            	   const ble_sls_init_t   * p_sls_init)
{
    ble_gatts_char_md_t char_md;
		ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));
	
    char_md.char_props.notify= 1;
    char_md.char_props.read  = 1;
	  //char_md.char_props.write = 1;
		char_md.char_user_desc_max_size=MAX_SLS_USER_DESC_LEN;
	  char_md.char_user_desc_size=sizeof("fan_ngt_ion_status");
	  char_md.p_char_user_desc=(uint8_t*)"fan_ngt_ion_status";

    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;
    char_md.p_cccd_md        = NULL;
    char_md.p_sccd_md        = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_FAN_NEGATIVE_ION_STATUS_CHAR);
    //cccd cfg
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&attr_md, 0, sizeof(attr_md));
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
		//BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_USER;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = MAX_SLS_STATUS_LEN;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = MAX_SLS_STATUS_LEN;
    attr_char_value.p_value   =(uint8_t*)&fan_negative_ion_status;

    return sd_ble_gatts_characteristic_add(p_sls->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_sls->fan_negative_ion_status_handles);
}
/**@brief Function for adding the elec_lock characteristic.
 *
 * @param[in]   p_sls        smart locker Service structure.
 * @param[in]   p_sls_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t elec_lock_cmd_char_add(ble_sls_t                 * p_sls, 
	                                     const ble_sls_init_t      * p_sls_init)
{
    ble_gatts_char_md_t char_md;
	  ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.notify= 1;
    char_md.char_props.read  = 1;
	  char_md.char_props.write = 1;
	  char_md.char_user_desc_max_size=MAX_SLS_USER_DESC_LEN;
	  char_md.char_user_desc_size=sizeof("elec_lock_cmd");
	  char_md.p_char_user_desc=(uint8_t*)"elec_lock_cmd";
    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;
    char_md.p_cccd_md        = NULL;
    char_md.p_sccd_md        = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_ELEC_LOCK_CMD_CHAR);
   //cccd cfg
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
		
    memset(&attr_md, 0, sizeof(attr_md));
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc       = BLE_GATTS_VLOC_USER;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = MAX_SLS_CMD_LEN;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = MAX_SLS_CMD_LEN;
    attr_char_value.p_value   = (uint8_t*)&elec_lock_cmd;

    return sd_ble_gatts_characteristic_add(p_sls->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_sls->elec_lock_cmd_handles);
}
static uint32_t elec_lock_status_char_add(ble_sls_t                 * p_sls, 
	                                        const ble_sls_init_t      * p_sls_init)
{
    ble_gatts_char_md_t char_md;
	  ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.notify= 1;
    char_md.char_props.read  = 1;
	  //char_md.char_props.write = 1;
	  char_md.char_user_desc_max_size=MAX_SLS_USER_DESC_LEN;
	  char_md.char_user_desc_size=sizeof("elec_lock_status");
	  char_md.p_char_user_desc=(uint8_t*)"elec_lock_status";
    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;
    char_md.p_cccd_md        = NULL;
    char_md.p_sccd_md        = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_ELEC_LOCK_STATUS_CHAR);
   //cccd cfg
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
		
    memset(&attr_md, 0, sizeof(attr_md));
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
		//BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc       = BLE_GATTS_VLOC_USER;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = MAX_SLS_STATUS_LEN;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = MAX_SLS_STATUS_LEN;
    attr_char_value.p_value   = (uint8_t*)&elec_lock_status;

    return sd_ble_gatts_characteristic_add(p_sls->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_sls->elec_lock_status_handles);
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
    err_code = uv_lamp_cmd_char_add(p_sls, p_sls_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		    // Add heart rate measurement characteristic
    err_code = uv_lamp_status_char_add(p_sls, p_sls_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		    // Add heart rate measurement characteristic
    err_code = uv_lamp_door_status_char_add(p_sls, p_sls_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

       err_code =fan_negative_ion_cmd_char_add(p_sls, p_sls_init);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
	     err_code =fan_negative_ion_status_char_add(p_sls, p_sls_init);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
       err_code =elec_lock_cmd_char_add(p_sls, p_sls_init);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
       err_code =elec_lock_status_char_add(p_sls, p_sls_init);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    return NRF_SUCCESS;
}
