#ifndef __ZIGBEE_LAMPS_CONTROLLER_LIGHT_H__
#define __ZIGBEE_LAMPS_CONTROLLER_LIGHT_H__

#include <inttypes.h>
#include <optional>
#include <utility>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define ZIGBEE_LAMPS_CONTROLLER_NUM_LIGHTS 3

typedef enum { POWER_OFF, POWER_ON } Power;
typedef enum {
  TEMPERATURE_3000K_MIRED = 333,
  TEMPERATURE_4000K_MIRED = 267,
  TEMPERATURE_5000K_MIRED = 200
} Temperature;

typedef enum {
  BRIGHTNESS_STATE_DIM = 10,
  BRIGHTNESS_STATE_BRIGHT = 255,
} Brightness;

typedef struct LightState {
  Power power_state;
  Temperature color_temperature;
  Brightness brightness;

  bool operator==(const struct LightState& other) const {
    return power_state == other.power_state && color_temperature == other.color_temperature &&
           brightness == other.brightness;
  }
} LightState;

typedef struct {
  uint8_t power;
  uint8_t color_temperature;
  uint8_t brightness_up;
  uint8_t brightness_down;
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

class Light {
 private:
  ControlPins control_pins;
  LightState curr_state;  // only accessed by LightsControllerTask

  void update_curr_state_to(const LightState& new_state);

  // Accesed by zigbee task and LightsControllerTask. Protected by mutex_handle
  LightState target_state;
  uint8_t modified_count = 0;  // Incremented whenever there is an attempt to modify target_state
                               // Overflow is fine as only equality is checked.

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
  void transition_to_target_state();

  void update_target_power_state(Power on_off);
  void update_target_brightness(Brightness brightness);
  void update_target_temperature(Temperature color_temperature);

  /*
   * Returns the target state and modified_count if the modified_count has changed since the last
   * call.
   */
  std::optional<std::pair<uint8_t, LightState>> get_state_and_modified_count_if_newer(
      uint8_t prev_modified_count);
};

/* Provides RAII style access to RTOS mutexes. */
class RTOSMutex {
 public:
  SemaphoreHandle_t mutex_handle;
  RTOSMutex(SemaphoreHandle_t mutex_handle) : mutex_handle(mutex_handle) {
    xSemaphoreTake(mutex_handle, portMAX_DELAY);
  }
  ~RTOSMutex() { xSemaphoreGive(mutex_handle); }
};

#endif  // __ZIGBEE_LAMPS_CONTROLLER_LIGHT_H__
