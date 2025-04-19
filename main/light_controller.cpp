#include "light_controller.h"
#include <cmath>
#include "driver/gpio.h"
#include "esp_check.h"
#include "light.h"

#define TAG "LightsController"

esp_err_t LightsController::initialize(const std::vector<uint8_t>& endpoints) {
  assert(endpoints.size() == ZIGBEE_LAMPS_CONTROLLER_NUM_LIGHTS);

  for (uint32_t i = 0; i < endpoints.size(); i++) {
    uint8_t ep = endpoints[i];
    ControlPins control_pins = LIGHT_PINS[i];

    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << control_pins.power) | (1ULL << control_pins.color_temperature) |
                        (1ULL << control_pins.brightness_up) |
                        (1ULL << control_pins.brightness_down),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "%s: Failed to configure GPIO pins for endpoint %d",
                        __FUNCTION__, ep);

    std::shared_ptr light = std::make_shared<Light>(control_pins);
    if (light->mutex_handle == NULL) {
      ESP_LOGE(TAG, "%s: Failed to initialize mutex for light ep: %d", __FUNCTION__, ep);
      return ESP_ERR_NO_MEM;
    }

    ep_to_light[ep] = light;
    ep_to_reported_modified_count[ep] = 255;  // Ensures that the first call to
                                              // light->get_state_and_modified_count_if_newer()
                                              // will return the initial state
  }

  signal_sem_handle = xSemaphoreCreateBinary();
  if (signal_sem_handle == NULL) {
    ESP_LOGE(TAG, "%s: Failed to create semaphore", __FUNCTION__);
    return ESP_ERR_NO_MEM;
  }

  xSemaphoreGive(signal_sem_handle);  // Signal once to get the lights task going.

  is_initialized = true;
  return ESP_OK;
}

void LightsController::lights_task_main_loop() {
  assert(is_initialized);

  ESP_LOGI(TAG, "%s: Starting Lights Task Main Loop", __FUNCTION__);

  while (true) {
    // Don't care why the task woke up, attempt to trasition all lights to the
    // target state.
    xSemaphoreTake(signal_sem_handle, portMAX_DELAY);

    for (auto& [ep, light] : ep_to_light) {
      light->transition_to_target_state();
    }
  }
}

void LightsController::update_on_off_state(uint8_t ep, bool on_off) {
  auto itr = ep_to_light.find(ep);
  if (itr == ep_to_light.end()) {
    ESP_LOGE(TAG, "%s: Could not find light of endpoint %d. Ignoring.", __FUNCTION__, ep);
  }

  Power new_power = on_off ? POWER_ON : POWER_OFF;
  itr->second->update_target_power_state(new_power);
  xSemaphoreGive(signal_sem_handle);
}

void LightsController::update_brightness(uint8_t ep, uint8_t brightness) {
  auto itr = ep_to_light.find(ep);
  if (itr == ep_to_light.end()) {
    ESP_LOGE(TAG, "%s: Could not find light of endpoint %d. Ignoring.", __FUNCTION__, ep);
    return;
  }

  int32_t req_brightness = brightness;
  int32_t diff_up = abs(req_brightness - BRIGHTNESS_STATE_BRIGHT);
  int32_t diff_down = abs(req_brightness - BRIGHTNESS_STATE_DIM);

  Brightness new_brightness = diff_up <= diff_down ? BRIGHTNESS_STATE_BRIGHT : BRIGHTNESS_STATE_DIM;
  itr->second->update_target_brightness(new_brightness);
  xSemaphoreGive(signal_sem_handle);
}

void LightsController::update_color_temperature(uint8_t ep, uint16_t color_temperature) {
  auto itr = ep_to_light.find(ep);
  if (itr == ep_to_light.end()) {
    ESP_LOGE(TAG, "%s: Could not find light of endpoint %d. Ignoring.", __FUNCTION__, ep);
    return;
  }

  int32_t req_temp = color_temperature;
  int32_t diffs[] = {abs(req_temp - TEMPERATURE_3000K_MIRED),
                     abs(req_temp - TEMPERATURE_4000K_MIRED),
                     abs(req_temp - TEMPERATURE_5000K_MIRED)};

  Temperature new_temp = TEMPERATURE_5000K_MIRED;
  if (diffs[0] < diffs[1] && diffs[0] < diffs[2]) {
    new_temp = TEMPERATURE_3000K_MIRED;
  } else if (diffs[1] < diffs[0] && diffs[1] < diffs[2]) {
    new_temp = TEMPERATURE_4000K_MIRED;
  } else {
    new_temp = TEMPERATURE_5000K_MIRED;
  }

  itr->second->update_target_temperature(new_temp);
  xSemaphoreGive(signal_sem_handle);
}

std::optional<LightState> LightsController::get_state_to_report(uint8_t ep) {
  auto reported_count_itr = ep_to_reported_modified_count.find(ep);
  auto light_itr = ep_to_light.find(ep);
  if (reported_count_itr == ep_to_reported_modified_count.end() || light_itr == ep_to_light.end()) {
    ESP_LOGE("LightsController", "Invalid endpoint (%d) to report.", ep);
  }

  uint8_t reported_count = reported_count_itr->second;

  std::optional<std::pair<uint8_t, LightState>> reporting_pair =
      light_itr->second->get_state_and_modified_count_if_newer(reported_count);

  if (!reporting_pair.has_value()) {
    return std::nullopt;
  }

  reported_count_itr->second = reporting_pair->first;
  return reporting_pair->second;
}
