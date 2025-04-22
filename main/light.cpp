#include "light.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"

#define TAG "Light"

void Light::update_curr_state_to(const LightState& new_state) {
  ESP_LOGD(TAG, "%s: Updating  State: { .power=%d, .color_temperature=%d, .brightness=%d }",
           __FUNCTION__, new_state.power_state, new_state.color_temperature, new_state.brightness);

  if (curr_state.power_state != new_state.power_state) {
    ESP_LOGD(TAG, "%s: Pressing Power. New power state: %d", __FUNCTION__, new_state.power_state);
    press_power();
    curr_state.power_state = new_state.power_state;
  }

  if (curr_state.power_state == POWER_OFF) {
    // Light is off. Other states cannot be changed at this time.
    // The states will be updated when the light is turned on.
    ESP_LOGD(TAG, "%s: Light is off. Ignoring any other udate.", __FUNCTION__);
    return;
  }

  if (curr_state.color_temperature != new_state.color_temperature) {
    ESP_LOGD(TAG, "%s: Changing Color Temperature. New color temperature: %d", __FUNCTION__,
             new_state.color_temperature);
    transition_to_color_temperature(curr_state.color_temperature, new_state.color_temperature);
    curr_state.color_temperature = new_state.color_temperature;
  }

  if (curr_state.brightness != new_state.brightness) {
    ESP_LOGD(TAG, "%s: Changing Brightness. New brightness: %d", __FUNCTION__,
             new_state.brightness);
    transition_to_brightness_level(curr_state.brightness, new_state.brightness);
    curr_state.brightness = new_state.brightness;
  }
}

int32_t Light::color_temperature_position(ColorTemperature color_temperature) {
  switch (color_temperature) {
    case ColorTemperature::TEMPERATURE_3000K_MIRED:
      return 0;
      break;
    case ColorTemperature::TEMPERATURE_4000K_MIRED:
      return 1;
      break;
    case ColorTemperature::TEMPERATURE_5000K_MIRED:
      return 2;
      break;
  }
  return 0;
}

int32_t Light::number_of_steps_to_target_color_temp(ColorTemperature from_temp,
                                                    ColorTemperature to_temp) {
  int32_t from_idx = color_temperature_position(from_temp);
  int32_t to_idx = color_temperature_position(to_temp);

  int32_t diff = to_idx - from_idx;
  if (diff < 0) {
    // This is the equivalent of diff % 3 (postitive only).
    diff = 3 + diff;
  }

  return diff;
}

void Light::transition_to_color_temperature(ColorTemperature from_color_temp,
                                            ColorTemperature to_color_temp) {
  int32_t steps = number_of_steps_to_target_color_temp(from_color_temp, to_color_temp);
  for (int32_t i = 0; i < steps; ++i) {
    press_color_temperature();
  }
}

void Light::transition_to_brightness_level(Brightness from_brightness, Brightness to_brightness) {
  if (from_brightness < to_brightness) {
    press_brightness_up(BRIGHTNESS_RELAY_PRESS_TIME_MS);
  } else {
    press_brightness_down(BRIGHTNESS_RELAY_PRESS_TIME_MS);
  }
}

void Light::press_power() {
  gpio_set_level(control_pins.power, 0);
  vTaskDelay(pdMS_TO_TICKS(RELAY_SIMPLE_PRESS_TIME_MS));
  gpio_set_level(control_pins.power, 1);
  vTaskDelay(pdMS_TO_TICKS(RELAY_CLEAR_TIME_MS));
}

void Light::press_color_temperature() {
  gpio_set_level(control_pins.color_temperature, 0);
  vTaskDelay(pdMS_TO_TICKS(RELAY_SIMPLE_PRESS_TIME_MS));
  gpio_set_level(control_pins.color_temperature, 1);
  vTaskDelay(pdMS_TO_TICKS(RELAY_CLEAR_TIME_MS));
}

void Light::press_brightness_up(uint64_t duration_ms) {
  gpio_set_level(control_pins.brightness_up, 0);
  vTaskDelay(pdMS_TO_TICKS(duration_ms));
  gpio_set_level(control_pins.brightness_up, 1);
  vTaskDelay(pdMS_TO_TICKS(RELAY_CLEAR_TIME_MS));
}

void Light::press_brightness_down(uint64_t duration_ms) {
  gpio_set_level(control_pins.brightness_down, 0);
  vTaskDelay(pdMS_TO_TICKS(duration_ms));
  gpio_set_level(control_pins.brightness_down, 1);
  vTaskDelay(pdMS_TO_TICKS(RELAY_CLEAR_TIME_MS));
}

esp_err_t Light::initialize_gpio() {
  gpio_config_t cfg = {
      .pin_bit_mask = (1ULL << control_pins.power) | (1ULL << control_pins.color_temperature) |
                      (1ULL << control_pins.brightness_up) | (1ULL << control_pins.brightness_down),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };

  ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "Failed to configure GPIO pins");
  gpio_set_level(control_pins.power, 1);
  gpio_set_level(control_pins.color_temperature, 1);
  gpio_set_level(control_pins.brightness_up, 1);
  gpio_set_level(control_pins.brightness_down, 1);
  return ESP_OK;
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

void Light::update_target_color_temperature(ColorTemperature color_temperature) {
  RTOSMutex mutex(mutex_handle);
  modified_count++;
  if (target_state.color_temperature == color_temperature) {
    return;
  }

  target_state.color_temperature = color_temperature;
}

std::optional<std::pair<uint8_t, LightState>> Light::get_modified_count_and_state_if_newer(
    uint8_t prev_modified_count) {
  RTOSMutex mutex(mutex_handle);
  if (modified_count == prev_modified_count) {
    return std::nullopt;
  }
  return std::make_optional(std::make_pair(modified_count, target_state));
}
