/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "main.h"
#include <inttypes.h>
#include <stdio.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "sdkconfig.h"

#define LED_PIN GPIO_NUM_8
#define LED_COUNT 1
#define BLINK_SPEED_MS 100

#define MAX_RUNTIME_MS 10000

static const TickType_t MAX_RUNTIME_TICKS = pdMS_TO_TICKS(MAX_RUNTIME_MS);

#define TAG "blink"

static led_strip_handle_t kLedStrip;
static bool kIsLedOn = false;

static TickType_t kStartTick;

bool setup(void) {
  ESP_LOGI(TAG, "%s: Setting up LED", __FUNCTION__);

  led_strip_config_t stripConfig = {.strip_gpio_num = LED_PIN,
                                    .max_leds = LED_COUNT};

  led_strip_spi_config_t spiConfig = {.spi_bus = SPI2_HOST,
                                      .flags.with_dma = true};

  ESP_ERROR_CHECK(
      led_strip_new_spi_device(&stripConfig, &spiConfig, &kLedStrip));

  ESP_LOGI(TAG, "%s: Sucessfully setup LED strip", __FUNCTION__);

  led_strip_clear(kLedStrip);

  kStartTick = xTaskGetTickCount();

  return true;
}

LoopResult loop(void) {
  if (kIsLedOn) {
    // Turn off LED
    led_strip_clear(kLedStrip);
  } else {
    led_strip_set_pixel(kLedStrip, 0, 16, 0, 16);  // Red color
    led_strip_refresh(kLedStrip);
  }

  kIsLedOn = !kIsLedOn;
  vTaskDelay(pdMS_TO_TICKS(BLINK_SPEED_MS));  // Delay for 1 second

  TickType_t currTick = xTaskGetTickCount();
  if (currTick - kStartTick > MAX_RUNTIME_TICKS) {
    ESP_LOGI(TAG, "%s: Max runtime reached (%d ms). Exiting loop.",
             __FUNCTION__, MAX_RUNTIME_MS);
    led_strip_clear(kLedStrip);
    return TERMINATE;
  }

  return CONTINUE;
}
