#ifndef __ZIGBEE_LAMPS_CONTROLLER_MAIN_H__
#define __ZIGBEE_LAMPS_CONTROLLER_MAIN_H__

#include <stdio.h>
#include "esp_system.h"

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
  printf("Calling setup function\n");

  if (!setup()) {
    printf("Setup failed, Exiting.\n");
    return;
  }

  printf("Setup successful, calling loop function\n");
  LoopResult ret;

  do {
    ret = loop();
  } while (ret == CONTINUE);

  if (ret == TERMINATE) {
    printf("Loop signalled exit. Exiting.\n");
  } else {
    printf("Loop Signalled restart. Restarting\n");
    esp_restart();
  }
}

#endif  // __ZIGBEE_LAMPS_CONTROLLER_MAIN_H__
