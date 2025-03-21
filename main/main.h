#ifndef __ZIGBEE_LAMPS_CONTROLLER_MAIN_H__
#define __ZIGBEE_LAMPS_CONTROLLER_MAIN_H__

#include <stdio.h>
#include "esp_log.h"
#include "esp_system.h"

#define _MAIN_H_TAG_ "main"

typedef enum { CONTINUE, TERMINATE, RESTART } LoopResult;

// Gives us an Arduino like framework where we can use setup and loop functions
// Both function returns true to continue, and false to stop.
bool setup();
LoopResult loop();

/**
 * Main application entry point, simply delegates to setup and loop functions.
 * @return void
 */
void app_main(void) {
  ESP_LOGI(_MAIN_H_TAG_, "Calling setup function");

  if (!setup()) {
    ESP_LOGI(_MAIN_H_TAG_, "Setup failed, Exiting.");
    return;
  }

  ESP_LOGI(_MAIN_H_TAG_, "Setup successful, calling loop function");
  LoopResult ret;

  do {
    ret = loop();
  } while (ret == CONTINUE);

  if (ret == TERMINATE) {
    ESP_LOGI(_MAIN_H_TAG_, "Loop signalled exit. Exiting.");
  } else {
    ESP_LOGI(_MAIN_H_TAG_, "Loop Signalled restart. Restarting.");
    esp_restart();
  }
}

#undef _MAIN_H_TAG_
#endif  // __ZIGBEE_LAMPS_CONTROLLER_MAIN_H__
