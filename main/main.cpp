/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "main.h"
#include <inttypes.h>
#include <stdio.h>
#include <memory>
#include <vector>
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "light.h"
#include "light_controller.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "zcl/esp_zigbee_zcl_common.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#define TAG "main"

#define HA_ONOFF_SWITCH_ENDPOINT 1 /* esp light switch device endpoint */

typedef struct light_bulb_device_params_s {
  esp_zb_ieee_addr_t ieee_addr;
  uint8_t endpoint;
  uint16_t short_addr;
} light_bulb_device_params_t;

std::unique_ptr<LightsController> lights_controller = std::make_unique<LightsController>();

void lights_controller_task_entry(void*) {
  lights_controller->lights_task_main_loop();
}

void report_light_states(void* ep_cast_as_ptr) {
  uint8_t ep = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(ep_cast_as_ptr));

  std::optional<LightState> state = lights_controller->get_state_to_report(ep);
  if (!state.has_value()) {
    ESP_LOGI(TAG, "%s: No need to report state for endpoint %d", __FUNCTION__, ep);
    return;
  }

  bool on_off = state->power_state == POWER_ON;
  uint8_t level = static_cast<uint8_t>(state->brightness);
  uint16_t color_temp = static_cast<uint16_t>(state->color_temperature);

  ESP_LOGI(TAG, "%s: Reported State: { .power=%d, .color_temperature=%d, .brightness=%d }",
           __FUNCTION__, state->power_state, state->color_temperature, state->brightness);

  esp_zb_zcl_set_attribute_val(ep, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                               ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, &on_off, false);
  esp_zb_zcl_set_attribute_val(ep, ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
                               ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                               ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID, &level, false);
  esp_zb_zcl_set_attribute_val(
      ep, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
      ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID, &color_temp, false);
}

esp_err_t deferred_driver_init(void) {
  std::vector<uint8_t> endpoints;
  endpoints.reserve(ZIGBEE_LAMPS_CONTROLLER_NUM_LIGHTS);
  for (uint8_t i = 0; i < ZIGBEE_LAMPS_CONTROLLER_NUM_LIGHTS; i++) {
    endpoints.push_back(i + ZIGBEE_LAMPS_CONTROLLER_ENDPOINT_START);
  }
  ESP_RETURN_ON_ERROR(lights_controller->initialize(endpoints), TAG,
                      "%s: Failed to initialized LightsController.", __FUNCTION__);

  xTaskCreate(lights_controller_task_entry, "LightsControllerTask", 4096, NULL, 5, NULL);

  return ESP_OK;
}

void bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
  ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG,
                      "Failed to start Zigbee bdb commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t* signal_struct) {
  uint32_t* p_sg_p = signal_struct->p_app_signal;
  esp_err_t err_status = signal_struct->esp_err_status;
  esp_zb_app_signal_type_t sig_type = static_cast<esp_zb_app_signal_type_t>(*p_sg_p);
  switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
      ESP_LOGI(TAG, "%s: Initialize Zigbee stack", __FUNCTION__);
      esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
      break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
      if (err_status == ESP_OK) {
        ESP_LOGI(TAG, "%s: Deferred driver initialization %s", __FUNCTION__,
                 deferred_driver_init() ? "failed" : "successful");
        ESP_LOGI(TAG, "%s: Device started up in%s factory-reset mode", __FUNCTION__,
                 esp_zb_bdb_is_factory_new() ? "" : " non");
        if (esp_zb_bdb_is_factory_new()) {
          ESP_LOGI(TAG, "%s: Start network steering", __FUNCTION__);
          esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
          ESP_LOGI(TAG, "%s: Device rebooted", __FUNCTION__);
        }
      } else {
        ESP_LOGW(TAG, "%s: %s failed with status: %s, retrying", __FUNCTION__,
                 esp_zb_zdo_signal_to_string(sig_type), esp_err_to_name(err_status));
        esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                               ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
      }
      break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
      if (err_status == ESP_OK) {
        esp_zb_ieee_addr_t extended_pan_id;
        esp_zb_get_extended_pan_id(extended_pan_id);
        ESP_LOGI(TAG,
                 "%s: Joined network successfully (Extended PAN ID: "
                 "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, "
                 "Channel:%d, Short Address: 0x%04hx)",
                 __FUNCTION__, extended_pan_id[7], extended_pan_id[6], extended_pan_id[5],
                 extended_pan_id[4], extended_pan_id[3], extended_pan_id[2], extended_pan_id[1],
                 extended_pan_id[0], esp_zb_get_pan_id(), esp_zb_get_current_channel(),
                 esp_zb_get_short_address());
      } else {
        ESP_LOGI(TAG, "%s: Network steering was not successful (status: %s)", __FUNCTION__,
                 esp_err_to_name(err_status));
        esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                               ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
      }
      break;
    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
      if (err_status == ESP_OK) {
        if (*(uint8_t*)esp_zb_app_signal_get_params(p_sg_p)) {
          ESP_LOGI(TAG, "%s: Network(0x%04hx) is open for %d seconds", __FUNCTION__,
                   esp_zb_get_pan_id(), *(uint8_t*)esp_zb_app_signal_get_params(p_sg_p));
        } else {
          ESP_LOGW(TAG, "%s: Network(0x%04hx) closed, devices joining not allowed.", __FUNCTION__,
                   esp_zb_get_pan_id());
        }
      }
      break;
    case ESP_ZB_ZDO_SIGNAL_LEAVE: {
      esp_zb_zdo_signal_leave_params_t* leave_params =
          static_cast<esp_zb_zdo_signal_leave_params_t*>(esp_zb_app_signal_get_params(p_sg_p));
      switch (leave_params->leave_type) {
        case ESP_ZB_NWK_LEAVE_TYPE_RESET:
          ESP_LOGI(TAG, "%s: Received no-rejoin leave signal. Factory Resetting.", __FUNCTION__);
          esp_zb_factory_reset();
          break;
        case ESP_ZB_NWK_LEAVE_TYPE_REJOIN:
          ESP_LOGI(TAG, "%s: Received rejoin leave signal. Restarting.", __FUNCTION__);
          esp_restart();
          break;
        default:
          ESP_LOGE(TAG, "%s: Invalid leave_param->leave_type: %d. Ignoring.", __FUNCTION__,
                   leave_params->leave_type);
          break;
      }
    } break;
    default:
      ESP_LOGI(TAG, "%s: ZDO signal: %s (0x%x), status: %s", __FUNCTION__,
               esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
      break;
  }
}

esp_err_t zb_on_off_control_attribute_handler(uint8_t ep, const esp_zb_zcl_attribute_t& attr) {
  uint16_t attribute_id = attr.id;

  ESP_RETURN_ON_FALSE(attribute_id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID &&
                          attr.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL,
                      ESP_OK, TAG,
                      "Unhandled ON_OFF_CONTROL attribute: 0x%x of type 0x%x. Ignoring.",
                      attribute_id, attr.data.type);

  bool new_state = *static_cast<bool*>(attr.data.value);
  ESP_LOGI(TAG, "%s: New on off state: %d", __FUNCTION__, new_state);
  lights_controller->update_on_off_state(ep, new_state);
  esp_zb_scheduler_user_alarm(report_light_states, reinterpret_cast<void*>(ep), 100);
  return ESP_OK;
}

esp_err_t zb_level_control_attribute_handler(uint8_t ep, const esp_zb_zcl_attribute_t& attr) {
  uint16_t attribute_id = attr.id;
  ESP_RETURN_ON_FALSE(attribute_id == ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID &&
                          attr.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8,
                      ESP_OK, TAG,
                      "Unhandled LEVEL_CONTROL attribute: 0x%x of type 0x%x. Ignoring.",
                      attribute_id, attr.data.type);

  uint8_t new_level = *static_cast<uint8_t*>(attr.data.value);
  ESP_LOGI(TAG, "%s: New level state: %d", __FUNCTION__, new_level);
  lights_controller->update_brightness(ep, new_level);
  esp_zb_scheduler_user_alarm(report_light_states, reinterpret_cast<void*>(ep), 100);
  return ESP_OK;
}

esp_err_t zb_color_control_attribute_handler(uint8_t ep, const esp_zb_zcl_attribute_t& attr) {
  uint16_t attribute_id = attr.id;

  ESP_RETURN_ON_FALSE(attribute_id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID &&
                          attr.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16,
                      ESP_OK, TAG,
                      "Unhandled COLOR_CONTROL attribute: 0x%x of type 0x%x. Ignoring.",
                      attribute_id, attr.data.type);

  uint16_t new_temp = *static_cast<uint16_t*>(attr.data.value);
  ESP_LOGI(TAG, "%s: Requested color temperature: %d", __FUNCTION__, new_temp);
  lights_controller->update_color_temperature(ep, new_temp);
  esp_zb_scheduler_user_alarm(report_light_states, reinterpret_cast<void*>(ep), 100);

  return ESP_OK;
}

esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t* message) {
  ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "%s: Empty message", __FUNCTION__);
  ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG,
                      "%s: Received message: error status(%d)", __FUNCTION__, message->info.status);

  uint16_t cluster_id = message->info.cluster;

  switch (cluster_id) {
    case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF: {
      return zb_on_off_control_attribute_handler(message->info.dst_endpoint, message->attribute);
    }; break;
    case ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL: {
      return zb_level_control_attribute_handler(message->info.dst_endpoint, message->attribute);
    }; break;
    case ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL: {
      return zb_color_control_attribute_handler(message->info.dst_endpoint, message->attribute);
    }; break;
    default: {
      ESP_LOGI(TAG,
               "%s: Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)",
               __FUNCTION__, message->info.dst_endpoint, message->info.cluster,
               message->attribute.id, message->attribute.data.size);
      ESP_LOGW(TAG, "%s: Unhandled Cluster ID: %d", __FUNCTION__, cluster_id);
      return ESP_OK;
    }; break;
  }
}

esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void* message) {
  esp_err_t ret = ESP_OK;
  switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
      ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t*)message);
      break;
    case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID: {
      const esp_zb_zcl_cmd_default_resp_message_t* resp =
          static_cast<const esp_zb_zcl_cmd_default_resp_message_t*>(message);
      ESP_LOGW(TAG, "%s: CMD_DEFAULT_RESP: Status: 0x%x", __FUNCTION__, resp->status_code);
    }; break;
    default:
      ESP_LOGW(TAG, "%s: Receive Zigbee action(0x%x) callback", __FUNCTION__, callback_id);
      break;
  }
  return ret;
}

esp_zb_ep_list_t* create_ep_list() {
  esp_zb_color_dimmable_light_cfg_t master_cfg = {
      .basic_cfg =
          {
              .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
              .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,
          },
      .identify_cfg =
          {
              .identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE,
          },
      .groups_cfg =
          {
              .groups_name_support_id = ESP_ZB_ZCL_GROUPS_NAME_SUPPORT_DEFAULT_VALUE,
          },
      .scenes_cfg =
          {
              .scenes_count = ESP_ZB_ZCL_SCENES_SCENE_COUNT_DEFAULT_VALUE,
              .current_scene = ESP_ZB_ZCL_SCENES_CURRENT_SCENE_DEFAULT_VALUE,
              .current_group = ESP_ZB_ZCL_SCENES_CURRENT_GROUP_DEFAULT_VALUE,
              .scene_valid = ESP_ZB_ZCL_SCENES_SCENE_VALID_DEFAULT_VALUE,
              .name_support = ESP_ZB_ZCL_SCENES_NAME_SUPPORT_DEFAULT_VALUE,
          },
      .on_off_cfg =
          {
              .on_off = ESP_ZB_ZCL_ON_OFF_ON_OFF_DEFAULT_VALUE,
          },
      .level_cfg =
          {
              .current_level = 255,
          },
      .color_cfg =
          {
              .current_x = ESP_ZB_ZCL_COLOR_CONTROL_CURRENT_X_DEF_VALUE,
              .current_y = ESP_ZB_ZCL_COLOR_CONTROL_CURRENT_Y_DEF_VALUE,
              .color_mode = 0x02,
              .options = ESP_ZB_ZCL_COLOR_CONTROL_OPTIONS_DEFAULT_VALUE,
              .enhanced_color_mode = ESP_ZB_ZCL_COLOR_CONTROL_ENHANCED_COLOR_MODE_DEFAULT_VALUE,
              .color_capabilities = 0x010,
          },
  };

  esp_zb_ep_list_t* ep_head = esp_zb_ep_list_create();
  for (uint8_t i = 0; i < ZIGBEE_LAMPS_CONTROLLER_NUM_LIGHTS; i++) {
    esp_zb_endpoint_config_t ep_config = {
        .endpoint = static_cast<uint8_t>(i + ZIGBEE_LAMPS_CONTROLLER_ENDPOINT_START),
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_COLOR_DIMMABLE_LIGHT_DEVICE_ID,
        .app_device_version = 0};
    esp_zb_cluster_list_t* cluster_list = esp_zb_color_dimmable_light_clusters_create(&master_cfg);
    {
      esp_zb_attribute_list_t* attr_list = esp_zb_cluster_list_get_cluster(
          cluster_list, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

      uint16_t curr_temp = TEMPERATURE_3000K_MIRED;
      uint16_t min_temp = TEMPERATURE_5000K_MIRED;
      uint16_t max_temp = TEMPERATURE_3000K_MIRED;

      esp_zb_color_control_cluster_add_attr(
          attr_list, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID, &curr_temp);
      esp_zb_color_control_cluster_add_attr(
          attr_list, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MIN_MIREDS_ID, &min_temp);
      esp_zb_color_control_cluster_add_attr(
          attr_list, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MAX_MIREDS_ID, &max_temp);
    }
    {
      esp_zb_attribute_list_t* attr_list = esp_zb_cluster_list_get_cluster(
          cluster_list, ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

      uint16_t min_level = BRIGHTNESS_STATE_DIM;
      uint16_t max_level = BRIGHTNESS_STATE_BRIGHT;

      esp_zb_level_cluster_add_attr(attr_list, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_MIN_LEVEL_ID,
                                    &min_level);
      esp_zb_level_cluster_add_attr(attr_list, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_MAX_LEVEL_ID,
                                    &max_level);
    }

    esp_zb_ep_list_add_ep(ep_head, cluster_list, ep_config);
  }
  return ep_head;
}

void esp_zcl_utility_add_ep_list_basic_manufacturer_info(esp_zb_ep_list_t* ep_list,
                                                         zcl_basic_manufacturer_info_t* info) {
  for (esp_zb_ep_list_t* ep = ep_list; ep != nullptr; ep = ep->next) {
    uint8_t ep_id = ep->endpoint.ep_id;
    if (ep_id == 0) {
      continue;
    }
    esp_zcl_utility_add_ep_basic_manufacturer_info(ep_list, ep_id, info);
  }
}

/**
 * Taken from
 * https://github.com/espressif/esp-zigbee-sdk/blob/main/examples/common/zcl_utility/src/zcl_utility.c
 */
esp_err_t esp_zcl_utility_add_ep_basic_manufacturer_info(esp_zb_ep_list_t* ep_list,
                                                         uint8_t endpoint_id,
                                                         zcl_basic_manufacturer_info_t* info) {
  esp_err_t ret = ESP_OK;
  esp_zb_cluster_list_t* cluster_list = NULL;
  esp_zb_attribute_list_t* basic_cluster = NULL;

  ESP_LOGI(TAG, "%s: Setting up manufacturer information.", __FUNCTION__);

  cluster_list = esp_zb_ep_list_get_ep(ep_list, endpoint_id);
  ESP_RETURN_ON_FALSE(cluster_list, ESP_ERR_INVALID_ARG, TAG,
                      "%s: Failed to find endpoint id: %d in list: %p", __FUNCTION__, endpoint_id,
                      ep_list);
  basic_cluster = esp_zb_cluster_list_get_cluster(cluster_list, ESP_ZB_ZCL_CLUSTER_ID_BASIC,
                                                  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  ESP_RETURN_ON_FALSE(basic_cluster, ESP_ERR_INVALID_ARG, TAG,
                      "%s: Failed to find basic cluster in endpoint: %d", __FUNCTION__,
                      endpoint_id);
  ESP_RETURN_ON_FALSE((info && info->manufacturer_name), ESP_ERR_INVALID_ARG, TAG,
                      "%s: Invalid manufacturer name", __FUNCTION__);
  ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(
      basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, info->manufacturer_name));
  ESP_RETURN_ON_FALSE((info && info->model_identifier), ESP_ERR_INVALID_ARG, TAG,
                      "Invalid model identifier");
  ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(
      basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, info->model_identifier));

  ESP_LOGI(TAG, "%s: Successfully set up manufacturer information.", __FUNCTION__);
  return ret;
}

void esp_zb_task(void*) {
  ESP_LOGI(TAG, "%s: Starting Zigbee task", __FUNCTION__);

  esp_zb_cfg_t zb_network_config = {.esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,
                                    .install_code_policy = INSTALLCODE_POLICY_ENABLE,
                                    .nwk_cfg = {.zczr_cfg = {.max_children = MAX_CHILDREN}}};

  static char manufacturer_name[] = "\x09HighOnH2O";
  static char model_identifier[] = "\x07" CONFIG_IDF_TARGET;

  zcl_basic_manufacturer_info_t manufacturer_info = {.manufacturer_name = manufacturer_name,
                                                     .model_identifier = model_identifier};

  esp_zb_init(&zb_network_config);
  esp_zb_ep_list_t* ep_list = create_ep_list();
  esp_zcl_utility_add_ep_list_basic_manufacturer_info(ep_list, &manufacturer_info);

  esp_zb_device_register(ep_list);
  esp_zb_core_action_handler_register(zb_action_handler);

  esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

  ESP_ERROR_CHECK(esp_zb_start(false));
  esp_zb_stack_main_loop();
}

extern "C" void app_main(void) {
  esp_zb_platform_config_t config = {
      .radio_config = {.radio_mode = ZB_RADIO_MODE_NATIVE, .radio_uart_config = {}},
      .host_config = {.host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,
                      .host_uart_config = {}}};

  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_zb_platform_config(&config));

  xTaskCreate(esp_zb_task, "zigbee_main", 4096, NULL, 5, NULL);
}
