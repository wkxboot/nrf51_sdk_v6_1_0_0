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

/** @file
 *
 * @defgroup ble_sdk_srv_sls Heart Rate Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief Heart Rate Service module.
 *
 * @details This module implements the Heart Rate Service with the Heart Rate Measurement,
 *          Body Sensor Location and Heart Rate Control Point characteristics.
 *          During initialization it adds the Heart Rate Service and Heart Rate Measurement
 *          characteristic to the BLE stack database. Optionally it also adds the
 *          Body Sensor Location and Heart Rate Control Point characteristics.
 *
 *          If enabled, notification of the Heart Rate Measurement characteristic is performed
 *          when the application calls ble_sls_heart_rate_measurement_send().
 *
 *          The Heart Rate Service also provides a set of functions for manipulating the
 *          various fields in the Heart Rate Measurement characteristic, as well as setting
 *          the Body Sensor Location characteristic value.
 *
 *          If an event handler is supplied by the application, the Heart Rate Service will
 *          generate Heart Rate Service events to the application.
 *
 * @note The application must propagate BLE stack events to the Heart Rate Service module by calling
 *       ble_sls_on_ble_evt() from the from the @ref ble_stack_handler callback.
 *
 * @note Attention! 
 *  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile 
 *  qualification listings, this section of source code must not be modified.
 */

#ifndef ble_sls_H__
#define ble_sls_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

// Body Sensor Location values
#define ble_sls_BODY_SENSOR_LOCATION_OTHER      0
#define ble_sls_BODY_SENSOR_LOCATION_CHEST      1
#define ble_sls_BODY_SENSOR_LOCATION_WRIST      2
#define ble_sls_BODY_SENSOR_LOCATION_FINGER     3
#define ble_sls_BODY_SENSOR_LOCATION_HAND       4
#define ble_sls_BODY_SENSOR_LOCATION_EAR_LOBE   5
#define ble_sls_BODY_SENSOR_LOCATION_FOOT       6

#define ble_sls_MAX_BUFFERED_RR_INTERVALS       20      /**< Size of RR Interval buffer inside service. */

/**@brief Heart Rate Service event type. */
typedef enum
{
    ble_sls_EVT_NOTIFICATION_ENABLED,                   /**< Heart Rate value notification enabled event. */
    ble_sls_EVT_NOTIFICATION_DISABLED                   /**< Heart Rate value notification disabled event. */
} ble_sls_evt_type_t;

/**@brief Heart Rate Service event. */
typedef struct
{
    ble_sls_evt_type_t evt_type;                        /**< Type of event. */
} ble_sls_evt_t;

// Forward declaration of the ble_sls_t type. 
typedef struct ble_sls_s ble_sls_t;

/**@brief Heart Rate Service event handler type. */
typedef void (*ble_sls_evt_handler_t) (ble_sls_t * p_sls, ble_sls_evt_t * p_evt);

/**@brief Heart Rate Service init structure. This contains all options and data needed for
 *        initialization of the service. */
typedef struct
{
    ble_sls_evt_handler_t        evt_handler;                                          /**< Event handler to be called for handling events in the Heart Rate Service. */
    
    
    ble_srv_cccd_security_mode_t sls_uv_lamp_attr_md;                                      /**< Initial security level for heart rate service measurement attribute */
    ble_srv_security_mode_t      sls_uv_lamp_door_attr_md;                                      /**< Initial security level for body sensor location attribute */
	  ble_srv_security_mode_t      sls_fan_ngt_ion_attr_md;                                      /**< Initial security level for body sensor location attribute */
	  ble_srv_security_mode_t      sls_elec_lock_attr_md;                                      /**< Initial security level for body sensor location attribute */
} ble_sls_init_t;

/**@brief Heart Rate Service structure. This contains various status information for the service. */
typedef struct ble_sls_s
{
    ble_sls_evt_handler_t        evt_handler;                                  /**< Event handler to be called for handling events in the Heart Rate Service. */
 
    uint16_t                     service_handle;                               /**< Handle of Heart Rate Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     uv_lamp_cmd_handles;                              /**< Handles related to the Heart Rate Measurement characteristic. */
		ble_gatts_char_handles_t     uv_lamp_status_handles;
	  ble_gatts_char_handles_t     uv_lamp_door_status_handles;                         /**< Handles related to the Heart Rate Control Point characteristic. */
    ble_gatts_char_handles_t     fan_negative_ion_cmd_handles;                     /**< Handles related to the Body Sensor Location characteristic. */
    ble_gatts_char_handles_t     fan_negative_ion_status_handles;
	  ble_gatts_char_handles_t     elec_lock_cmd_handles;
  	ble_gatts_char_handles_t     elec_lock_status_handles;                            /**< Handles related to the Heart Rate Control Point characteristic. */
    uint16_t                     conn_handle;                                  /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */ 
  
} ble_sls_t;

/**@brief Function for initializing the Heart Rate Service.
 *
 * @param[out]  p_sls       Heart Rate Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_sls_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_sls_init(ble_sls_t * p_sls, const ble_sls_init_t * p_sls_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Heart Rate Service.
 *
 * @param[in]   p_sls      Heart Rate Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_sls_on_ble_evt(ble_sls_t * p_sls, ble_evt_t * p_ble_evt);
void ble_sls_device_notify_dev_info_timer_handler(void *p_context);

#endif // ble_sls_H__

/** @} */
