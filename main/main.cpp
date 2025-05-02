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

// #define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#define TAG "main"

#define HA_ONOFF_SWITCH_ENDPOINT 1 /* esp light switch device endpoint */

/*
 * Global controller for lights. This handles the actual logic to changing the state of the lights
 */
std::unique_ptr<LightsController> lights_controller = std::make_unique<LightsController>();

/**
 * Entry point for lights controller task. This task updates the state of the
 * lights separately from what is reported to zigbee endpoints.
 */
void lights_controller_task_entry(void*) {
  lights_controller->lights_task_main_loop();
}

/**
 * Reports the current state of the lights to the zigbee endpoint. This is needed because the lights
 * only support specific color temperatures and brightnesses. So, after the command to change
 * brightness/color temperature is received, this method reports the actual value that was set.
 *
 * The value of the attribute can NOT be written to the in zb_action_handler. The writes need
 * to happen outside of the callback. So, zb_action_handler schedules this function whenever
 * the coordinator writes to the relevant attributes.
 *
 * @param ep_cast_as_ptr endpoint at which state must be reported, cast as pointer. DO NOT
 * DEREFERENCE.
 */
void report_light_states(void* ep_cast_as_ptr) {
  uint8_t ep = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(ep_cast_as_ptr));

  // return value of get_state_to_report is std::nullopt if there was no attempt
  // to change the state of the light between the previous call and this call.
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

  // Strictly speaking, on/off isn't needed, but while we're here good to make sure the reported
  // state is consistent with internal state.
  // Reporting is simply modeled as setting the relevant attribute to the desired value. If the
  // attribute was configured with REPORTABLE access and the coordinator is subscribed, the new
  // value will be reported as configured.
  esp_zb_zcl_set_attribute_val(ep, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                               ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, &on_off, false);
  esp_zb_zcl_set_attribute_val(ep, ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
                               ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                               ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID, &level, false);
  esp_zb_zcl_set_attribute_val(
      ep, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
      ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID, &color_temp, false);
}

/**
 * Called on device boot. We use this spot to initialize the lights controller which initializes
 * GPIO pins and allocates bookkeeping memory as needed.
 */
esp_err_t initialize_lights_controller(void) {
  std::array<uint8_t, ZIGBEE_LAMPS_CONTROLLER_NUM_LIGHTS> endpoints;
  for (uint8_t i = 0; i < ZIGBEE_LAMPS_CONTROLLER_NUM_LIGHTS; i++) {
    endpoints[i] = (i + ZIGBEE_LAMPS_CONTROLLER_ENDPOINT_START);
  }

  ESP_RETURN_ON_ERROR(lights_controller->initialize(endpoints), TAG,
                      "%s: Failed to initialized LightsController.", __FUNCTION__);

  for (uint8_t ep : endpoints) {
    // Schedule reporting of initial states.
    esp_zb_scheduler_user_alarm(report_light_states, reinterpret_cast<void*>(ep), 1000);
  }

  // Create the task that will handle the lights controller updates.
  xTaskCreate(lights_controller_task_entry, "LightsControllerTask", 4096, NULL, 5, NULL);

  return ESP_OK;
}

/**
 * Callback for when Zigbee network commissioning succeeds or fails. Network commissioning refers
 * to whether the device has joined a zigbee network or not.
 *
 * This callback simply tries to join a network again.
 */
void bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
  if (esp_zb_bdb_start_top_level_commissioning(mode_mask) != ESP_OK) {
    ESP_LOGI(TAG, "%s: Failed to start Zigbee bdb commissioning", __FUNCTION__);
  }
}

/**
 * Callback for zigbee's high level signals. This handler isn't registered manually. The SDK
 * simply assumes this function to exist.
 */
void esp_zb_app_signal_handler(esp_zb_app_signal_t* signal_struct) {
  uint32_t* p_sg_p = signal_struct->p_app_signal;
  esp_err_t err_status = signal_struct->esp_err_status;
  esp_zb_app_signal_type_t sig_type = static_cast<esp_zb_app_signal_type_t>(*p_sg_p);
  switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
      // Called when network is ready to be commissioned -- i.e. all data required for this device
      // to be identified by the network is set up. This is the result of using autostart=false
      // when calling esp_zb_start.
      ESP_LOGI(TAG, "%s: Initialize Zigbee stack", __FUNCTION__);
      esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
      break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
      // Called when the device joins or rejoins a network. ESP_OK = sucessfully joined network
      if (err_status == ESP_OK) {
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
      // Called when the device has successfully joined a network.
      // BDB = Base Device Behavior
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
      // Called when the network that the device has joined is open for more connections.
      // We don't actually do anything here
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
      // Called when the coordinator has booted the current device off the network.
      // This can happen with two cases: The coordinator wants the device to rejoin
      // after a bit, or the coordinator wants the device to leave permanently.
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

/**
 * Function to handle writes to on_off control cluster attributes. It updates the target state of
 * the light corresponding to the endpoint and queues up a callback to update the reported value.
 */
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

/**
 * Function to handle writes to level control cluster attributes. It updates the target state of the
 * light corresponding to the endpoint and queues up a callback to update the reported value.
 *
 * Level control means brightness.
 */
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

/**
 * Function to handle writes to color control cluster attributes. It updates the target state of the
 * light corresponding to the endpoint and queues up a callback to update the reported value.
 *
 * In this case, the color contol is limited to color temperature.
 */
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

/**
 * Called whenever the coordinator tries to write a value to an attribute.
 *
 * See zb_action_handler.
 */
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
      ESP_LOGV(TAG,
               "%s: Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)",
               __FUNCTION__, message->info.dst_endpoint, message->info.cluster,
               message->attribute.id, message->attribute.data.size);
      ESP_LOGW(TAG, "%s: Unhandled Cluster ID: %d", __FUNCTION__, cluster_id);
      return ESP_ERR_INVALID_ARG;
    }; break;
  }
}

/**
 * Callback for whenever a coordinator tries to issues a command to the device. Configured using
 * esp_zb_core_action_handler_register in esp_zb_task.
 */
esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void* message) {
  esp_err_t ret = ESP_OK;
  switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
      // Called when coodinator attempts to set a value of an attribute.
      // For example, to turn a light on, the coordinator will write the value ON to the OnOff
      // attribute of OnOff cluster
      ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t*)message);
      break;
    case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID: {
      // Called whenever the zigbee stack sends a default response to a coordinator command.
      // This is here to log when the coordinator subscribes to a cluster/attribute pair.
      const esp_zb_zcl_cmd_default_resp_message_t* resp =
          static_cast<const esp_zb_zcl_cmd_default_resp_message_t*>(message);
      ESP_LOGI(TAG, "%s: CMD_DEFAULT_RESP: Status: 0x%x", __FUNCTION__, resp->status_code);
    }; break;
    default:
      ESP_LOGI(TAG, "%s: Receive Zigbee action(0x%x) callback", __FUNCTION__, callback_id);
      break;
  }
  return ret;
}

/**
 * Creates a list of zigbee endpoints. A zigbee endpoint is the smallest complete zigbee device. A
 * real device (with one network chip, MAC adress etc.) can have up to 255 endpoints -- although not
 * all can be used under the current specification. Each endpoint will act as a complete device in
 * itself.
 *
 * In our case, we use one esp32c6 chip to create 3 endpoints, each of which controls one light.
 *
 * Each endpoint consists of a list of clusters, each of which has a list of attributes. The
 * coordinator reads these lists to figure out the capabilities of the device.
 *
 * @return A complete list of endpoints to be configured.
 */
esp_zb_ep_list_t* create_ep_list() {
  // This is a helper struct to make configuring a set of clusters and their attributes easier.
  // Theoretically, rather than use this struct, we could manually create a list of clusters and
  // manually add the required attributes to each cluster.
  // In the struct below, each '_cfg' member maps to a cluster, and the structs under the '_cfg'
  // members are attributes that will be configured.
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
              .on_off = true,
          },
      .level_cfg =
          {
              .current_level = 254,
          },
      .color_cfg =
          {
              .current_x = ESP_ZB_ZCL_COLOR_CONTROL_CURRENT_X_DEF_VALUE,
              .current_y = ESP_ZB_ZCL_COLOR_CONTROL_CURRENT_Y_DEF_VALUE,
              .color_mode =
                  0x02,  // specifies that the "color" can be completely defined by the temperature
              .options = ESP_ZB_ZCL_COLOR_CONTROL_OPTIONS_DEFAULT_VALUE,
              .enhanced_color_mode = ESP_ZB_ZCL_COLOR_CONTROL_ENHANCED_COLOR_MODE_DEFAULT_VALUE,
              .color_capabilities =
                  0x010,  // Sets the 4th bit -- only color temperature is supported.
          },
  };

  // We need to create 3 endpoints. These endpoints need to be added to a list to be configured.
  // The first entry is that of endpoint 0 returned by esp_zb_list_create.
  esp_zb_ep_list_t* ep_head = esp_zb_ep_list_create();
  for (uint8_t i = 0; i < ZIGBEE_LAMPS_CONTROLLER_NUM_LIGHTS; i++) {
    esp_zb_endpoint_config_t ep_config = {
        .endpoint = static_cast<uint8_t>(i + ZIGBEE_LAMPS_CONTROLLER_ENDPOINT_START),
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,  // Defines the ep as a Home Automation device.
        .app_device_id = ESP_ZB_HA_COLOR_DIMMABLE_LIGHT_DEVICE_ID,
        .app_device_version = 0};

    // This is the helper function that takes in the config from above and creates a list of
    // required clusters with the proper attributes
    esp_zb_cluster_list_t* cluster_list = esp_zb_color_dimmable_light_clusters_create(&master_cfg);
    {
      // The helper function and struct only configure the CurrentX/Y attributes. It does not set up
      // the attributes for color temperatures. So we need to add the color temperature attributes
      // manually.
      esp_zb_attribute_list_t* attr_list = esp_zb_cluster_list_get_cluster(
          cluster_list, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

      uint16_t curr_temp = TEMPERATURE_3000K_MIRED;
      uint16_t min_temp = TEMPERATURE_5000K_MIRED;
      uint16_t max_temp = TEMPERATURE_3000K_MIRED;

      // Add min/max/current color temperature attributes to the color control cluster.
      // These methods automatically configure reporting access as well.
      esp_zb_color_control_cluster_add_attr(
          attr_list, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID, &curr_temp);
      esp_zb_color_control_cluster_add_attr(
          attr_list, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MIN_MIREDS_ID, &min_temp);
      esp_zb_color_control_cluster_add_attr(
          attr_list, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MAX_MIREDS_ID, &max_temp);
    }
    {
      // The helper function configures the min/max to 0/254. Our light doesn't support 0
      // brightness, so override min brightness to BRIGHTNESS_STATE_DIM instead
      esp_zb_attribute_list_t* attr_list = esp_zb_cluster_list_get_cluster(
          cluster_list, ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

      uint16_t min_level = BRIGHTNESS_STATE_DIM;
      uint16_t max_level = BRIGHTNESS_STATE_BRIGHT;

      // On Level refers to the brigthness level that the light defaults to when turned on (without
      // setting brightness explicitly). A value of 255(0xff) means that the light will turn on at
      // the brightness at which it was turned off (aka "previous")
      uint8_t on_level = 0xff;

      esp_zb_level_cluster_add_attr(attr_list, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_MIN_LEVEL_ID,
                                    &min_level);
      esp_zb_level_cluster_add_attr(attr_list, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_MAX_LEVEL_ID,
                                    &max_level);
      esp_zb_level_cluster_add_attr(attr_list, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_ON_LEVEL_ID,
                                    &on_level);
    }

    // Finally add the current endpoint to the list of endpoints.
    esp_zb_ep_list_add_ep(ep_head, cluster_list, ep_config);
  }
  return ep_head;
}

/**
 * This function takes a list of endpoints and adds basic manufacturer info for each of the
 * endpoints.
 *
 * To be honest, I am not convinced that this needs to be done for every endpoint, but this seems to
 * be working for now.
 *
 * @param ep_list The list of endpoints to add basic manufacturer info for.
 * @param info The basic manufacturer information to add.
 */
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
 * Adds the passed manufaturer info to the endpoint corresponding to endpoint_id.
 * Taken as-is from
 * https://github.com/espressif/esp-zigbee-sdk/blob/main/examples/common/zcl_utility/src/zcl_utility.c
 *
 * @param ep_list The complete list of endpoints in the device.
 * @param endpoint_id The ID of the endpoint to add the manufacturer info to.
 * @param info The manufacturer info to add.
 *
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t esp_zcl_utility_add_ep_basic_manufacturer_info(esp_zb_ep_list_t* ep_list,
                                                         uint8_t endpoint_id,
                                                         zcl_basic_manufacturer_info_t* info) {
  esp_err_t ret = ESP_OK;
  esp_zb_cluster_list_t* cluster_list = NULL;
  esp_zb_attribute_list_t* basic_cluster = NULL;

  ESP_LOGI(TAG, "%s: Setting up manufacturer information for ep: %d", __FUNCTION__, endpoint_id);

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

  // "Adding" manufacturer info is basically just adding a few attributes to the basic cluster.
  ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(
      basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, info->manufacturer_name));
  ESP_RETURN_ON_FALSE((info && info->model_identifier), ESP_ERR_INVALID_ARG, TAG,
                      "Invalid model identifier");
  ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(
      basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, info->model_identifier));

  ESP_LOGI(TAG, "%s: Successfully set up manufacturer information.", __FUNCTION__);
  return ret;
}

/**
 * Entry point for the zigbee_main task. Needs to set up the zigbee stack and run the zigbee main
 * loop.
 */
void esp_zb_task(void*) {
  ESP_LOGI(TAG, "%s: Starting Zigbee task", __FUNCTION__);

  // Configure the device as a router instead of a coordinator or an edge.
  esp_zb_cfg_t zb_network_config = {.esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,
                                    .install_code_policy = INSTALLCODE_POLICY_ENABLE,
                                    .nwk_cfg = {.zczr_cfg = {.max_children = MAX_CHILDREN}}};

  // Pulled out of the hat!
  // I am not sure what the significance of the \x09 and \x07 are, but all the samples had it.
  static char manufacturer_name[] = "\x09HighOnH2O";
  static char model_identifier[] = "\x07" CONFIG_IDF_TARGET;

  zcl_basic_manufacturer_info_t manufacturer_info = {.manufacturer_name = manufacturer_name,
                                                     .model_identifier = model_identifier};

  // Initialize zigbee stack
  esp_zb_init(&zb_network_config);

  // Create the various endpoints and add manufacturer info to the endpoints.
  esp_zb_ep_list_t* ep_list = create_ep_list();
  esp_zcl_utility_add_ep_list_basic_manufacturer_info(ep_list, &manufacturer_info);

  // Register the endpoints.
  esp_zb_device_register(ep_list);

  // Register the action handler -- this is the function that will be called
  // whenever coordinator issues a command.
  esp_zb_core_action_handler_register(zb_action_handler);

  // Don't restrict to a particular network channel
  esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

  // Set up the zigbee network network layers
  ESP_ERROR_CHECK(esp_zb_start(false));

  ESP_LOGI(TAG, "%s: Initializing Lights Controller: %s", __FUNCTION__,
           initialize_lights_controller() ? "failed" : "successful");

  // Start the stack main loop -- does not return.
  esp_zb_stack_main_loop();
}

extern "C" void app_main(void) {
  esp_zb_platform_config_t config = {
      .radio_config = {.radio_mode = ZB_RADIO_MODE_NATIVE, .radio_uart_config = {}},
      .host_config = {.host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,
                      .host_uart_config = {}}};

  // This is required for the zigbee stack to work properly. There needs to be specific zigbee
  // paritions defined in the partition table -- see partitions.csv
  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_zb_platform_config(&config));

  xTaskCreate(esp_zb_task, "zigbee_main", 4096, NULL, 5, NULL);
}
