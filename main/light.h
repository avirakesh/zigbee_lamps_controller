#ifndef __ZIGBEE_LAMPS_CONTROLLER_LIGHT_H__
#define __ZIGBEE_LAMPS_CONTROLLER_LIGHT_H__

#include <inttypes.h>
#include <optional>
#include <utility>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define ZIGBEE_LAMPS_CONTROLLER_NUM_LIGHTS 3
#define NUM_COLOR_TEMPERATURES 3

#define RELAY_SIMPLE_PRESS_TIME_MS 300
#define RELAY_CLEAR_TIME_MS 150

#define BRIGHTNESS_RELAY_PRESS_TIME_MS 5000

typedef enum { POWER_OFF, POWER_ON } Power;

/**
 * Mired is defined as 1'000'000 / color_temp_in_kelvin
 */
typedef enum {
  TEMPERATURE_3000K_MIRED = 333,
  TEMPERATURE_4000K_MIRED = 267,
  TEMPERATURE_5000K_MIRED = 200
} ColorTemperature;

typedef enum {
  BRIGHTNESS_STATE_DIM = 10,
  BRIGHTNESS_STATE_BRIGHT = 254,
} Brightness;

typedef struct LightState {
  Power power_state;
  ColorTemperature color_temperature;
  Brightness brightness;

  bool operator==(const struct LightState& other) const {
    return power_state == other.power_state && color_temperature == other.color_temperature &&
           brightness == other.brightness;
  }
} LightState;

typedef struct {
  gpio_num_t power;
  gpio_num_t color_temperature;
  gpio_num_t brightness_up;
  gpio_num_t brightness_down;
} ControlPins;

const ControlPins LIGHT_PINS[ZIGBEE_LAMPS_CONTROLLER_NUM_LIGHTS] = {
    {
        .power = GPIO_NUM_4,
        .color_temperature = GPIO_NUM_5,
        .brightness_up = GPIO_NUM_6,
        .brightness_down = GPIO_NUM_7,
    },
    {
        .power = GPIO_NUM_10,
        .color_temperature = GPIO_NUM_11,
        .brightness_up = GPIO_NUM_2,
        .brightness_down = GPIO_NUM_3,
    },
    {
        .power = GPIO_NUM_23,
        .color_temperature = GPIO_NUM_22,
        .brightness_up = GPIO_NUM_21,
        .brightness_down = GPIO_NUM_20,
    }};

/**
 * This class contains information and logic around controlling a single light. There are two tasks
 * that access a light: (1) zigbee task to update the target state, and (2) lights controller tasks
 * to actually change physical state of the light. So care must be taken when accessing data
 * structures.
 */
class Light {
 private:
  ControlPins control_pins;  // readonly

  LightState curr_state;  // The actual current physical state of the light -- only accessed by
                          // LightsControllerTask

  /**
   * Updates the physical state of the light (tracked by curr_state) to match new_state.
   */
  void update_curr_state_to(const LightState& new_state);

  LightState target_state;  // The state that zigbee wants the light to be in. Accesed by both: the
                            // zigbee task, and the LightsController Task. Protected by mutex_handle
  uint8_t modified_count = 0;  // Incremented whenever there is an attempt to modify target_state
                               // Overflow is fine as only equality is checked. Protected by
                               // mutex_handle

  int32_t color_temperature_position(ColorTemperature color_temperature);

  int32_t number_of_steps_to_target_color_temp(ColorTemperature from_temp,
                                               ColorTemperature to_temp);

  void transition_to_color_temperature(ColorTemperature from_color_temp,
                                       ColorTemperature to_color_temp);

  void transition_to_brightness_level(Brightness from_brightness, Brightness to_brightness);

  // "Presses" the power button, by toggling GPIO state for a short duration.
  void press_power();
  // "Presses" the color temperature button, by toggling GPIO state for a short duration.
  void press_color_temperature();
  // "Presses" the brightness up button, by toggling GPIO state for the specified duration.
  void press_brightness_up(uint64_t duration_ms);
  // "Presses" the brightness down button, by toggling GPIO state for the specified duration.
  void press_brightness_down(uint64_t duration_ms);

 public:
  SemaphoreHandle_t mutex_handle;  // Mutex protects read/write of target_state and modified_count.

  Light(ControlPins pins) : control_pins(pins) {
    curr_state = {.power_state = POWER_ON,
                  .color_temperature = TEMPERATURE_3000K_MIRED,
                  .brightness = BRIGHTNESS_STATE_BRIGHT};

    mutex_handle = xSemaphoreCreateMutex();
    target_state = {.power_state = POWER_ON,
                    .color_temperature = TEMPERATURE_3000K_MIRED,
                    .brightness = BRIGHTNESS_STATE_BRIGHT};
  }

  /**
   * Called to set up the GPIO pins for the light.
   * @return ESP_OK if successful, otherwise an error code.
   */
  esp_err_t initialize_gpio();

  /**
   * Changes the physical state of the light to match the tracked "target_state". This function
   * should only be called by the lights controller task when it wants to change the physical state
   * of the light.
   */
  void transition_to_target_state();

  /**
   * Updates the target power state of the light. Note that this does not change the physical
   * state of the light. Physical state will be updated when transition_to_target_state function is
   * called.
   *
   * Must only be called by the zigbee task.
   */
  void update_target_power_state(Power on_off);

  /**
   * Updates the target brighness state of the light. Note that this does not change the physical
   * state of the light. Physical state will be updated when transition_to_target_state function is
   * called.
   *
   * Must only be called by the zigbee task.
   */
  void update_target_brightness(Brightness brightness);

  /**
   * Updates the target color temperature of the light. Note that this does not change the physical
   * state of the light. Physical state will be updated when transition_to_target_state function is
   * called.
   *
   * Must only be called by the zigbee task.
   */
  void update_target_color_temperature(ColorTemperature color_temperature);

  /*
   * Returns the target state and modified_count if the modified_count is not equal to
   * prev_modified_count. The caller must save the returned modified count after every call and use
   * it for the next call. This ensures that we don't attempt to report states that have no changes.
   */
  std::optional<std::pair<uint8_t, LightState>> get_modified_count_and_state_if_newer(
      uint8_t prev_modified_count);
};

/* Simple RAII style access to RTOS mutexes. */
class RTOSMutex {
 public:
  SemaphoreHandle_t mutex_handle;
  RTOSMutex(SemaphoreHandle_t mutex_handle) : mutex_handle(mutex_handle) {
    xSemaphoreTake(mutex_handle, portMAX_DELAY);
  }
  ~RTOSMutex() { xSemaphoreGive(mutex_handle); }
};

#endif  // __ZIGBEE_LAMPS_CONTROLLER_LIGHT_H__
