#ifndef __ZIGBEE_LAMPS_CONTROLLER_MAIN_H__
#define __ZIGBEE_LAMPS_CONTROLLER_MAIN_H__

#include <stdio.h>
#include "esp_log.h"
#include "esp_system.h"

#include "esp_zigbee_core.h"

#define INSTALLCODE_POLICY_ENABLE false
#define MAX_CHILDREN 10

#define ZIGBEE_LAMPS_CONTROLLER_ENDPOINT_START 10

#define ESP_ZB_PRIMARY_CHANNEL_MASK 0x07FFF800

/**
 * optional basic manufacturer information
 * Taken from:
 * https://github.com/espressif/esp-zigbee-sdk/blob/main/examples/common/zcl_utility/include/zcl_utility.h
 */
typedef struct zcl_basic_manufacturer_info_s {
  char* manufacturer_name;
  char* model_identifier;
} zcl_basic_manufacturer_info_t;

/**
 * @brief Adds manufacturer information to the ZCL basic cluster of endpoint
 *
 * @param[in] ep_list The pointer to the endpoint list with @p endpoint_id
 * @param[in] endpoint_id The endpoint identifier indicating where the ZCL basic
 * cluster resides
 * @param[in] info The pointer to the basic manufacturer information
 * @return
 *      - ESP_OK: On success
 *      - ESP_ERR_INVALID_ARG: Invalid argument
 */
esp_err_t esp_zcl_utility_add_ep_basic_manufacturer_info(esp_zb_ep_list_t* ep_list,
                                                         uint8_t endpoint_id,
                                                         zcl_basic_manufacturer_info_t* info);

#endif  // __ZIGBEE_LAMPS_CONTROLLER_MAIN_H__
