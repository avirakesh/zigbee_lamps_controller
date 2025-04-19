#include "light.h"
#include "esp_log.h"

#define TAG "Light"

void Light::update_curr_state_to(const LightState& new_state) {
  curr_state = new_state;
  ESP_LOGI(TAG, "%s: Updated State: { .power=%d, .color_temperature=%d, .brightness=%d }",
           __FUNCTION__, curr_state.power_state, curr_state.color_temperature,
           curr_state.brightness);
}

void Light::transition_to_target_state() {
  LightState new_state;
  {
    RTOSMutex mutex(mutex_handle);
    new_state = target_state;
  }
  if (new_state == curr_state) {
    // Trivial case. Do nothing.
    return;
  }

  update_curr_state_to(new_state);
}

void Light::update_target_power_state(Power on_off) {
  RTOSMutex mutex(mutex_handle);
  modified_count++;
  if (target_state.power_state == on_off) {
    return;
  }

  target_state.power_state = on_off;
}

void Light::update_target_brightness(Brightness brightness) {
  RTOSMutex mutex(mutex_handle);
  modified_count++;
  if (target_state.brightness == brightness) {
    return;
  }

  target_state.brightness = brightness;
}

void Light::update_target_temperature(Temperature color_temperature) {
  RTOSMutex mutex(mutex_handle);
  modified_count++;
  if (target_state.color_temperature == color_temperature) {
    return;
  }

  target_state.color_temperature = color_temperature;
}

std::optional<std::pair<uint8_t, LightState>> Light::get_state_and_modified_count_if_newer(
    uint8_t prev_modified_count) {
  RTOSMutex mutex(mutex_handle);
  if (modified_count == prev_modified_count) {
    return std::nullopt;
  }
  return std::make_optional(std::make_pair(modified_count, target_state));
}
