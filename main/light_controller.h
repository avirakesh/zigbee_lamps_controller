#ifndef __ZIGBEE_LAMPS_CONTROLLER_LIGHT_CONTROLLER_H__
#define __ZIGBEE_LAMPS_CONTROLLER_LIGHT_CONTROLLER_H__

#include <inttypes.h>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "light.h"

class LightsController {
 public:
  LightsController() {};

  esp_err_t initialize(const std::array<uint8_t, ZIGBEE_LAMPS_CONTROLLER_NUM_LIGHTS>& endpoints);
  /**
   * Entry point for the lights controller task. This function never returns.
   */
  void lights_task_main_loop();

  /*
   * This will update the on off state of the light.
   */
  void update_on_off_state(uint8_t ep, bool on_off);
  /*
   * This will update the brightness to the closest value that is within the range of supported
   * brightnesses.
   *
   * get_state_to_report() must be called to get the brightness that was set.
   */
  void update_brightness(uint8_t ep, uint8_t brightness);
  /*
   * This will update the color temperature to the closest value that is within the range
   * of supported temperatures.
   *
   * get_state_to_report() must be called to get the color temperature that was set.
   */
  void update_color_temperature(uint8_t ep, uint16_t color_temperature);

  /**
   * Returns the state that should be reported by the zigbee stack. Returns std::nullopt if there
   * have been no attempts to change the target state since previous call.
   */
  std::optional<LightState> get_state_to_report(uint8_t ep);

 private:
  bool is_initialized = false;
  std::unordered_map<uint8_t, std::shared_ptr<Light>> ep_to_light;

  // Tracks the modified_count value of the last reported state for each endpoint.
  std::unordered_map<uint8_t, uint8_t> ep_to_reported_modified_count;

  // Semaphore to signal the lights controller task that there may be a change in state.
  // We're functionally using a binary semaphore as a std::condition_variable.
  // The zigbee task signals this handle whenever there is an attempt to change the light state and
  // the lights controller tasks waits on this handle to wake up and update the light state.
  SemaphoreHandle_t signal_sem_handle;
};

#endif  // __ZIGBEE_LAMPS_CONTROLLER_LIGHT_CONTROLLER_H__
