//! @file
//!
//! Copyright (c) Memfault, Inc.
//! See LICENSE for details
//!
//! Deep sleep handling

#include "deep_sleep.h"

#include <inttypes.h>
#include <sys/time.h>

#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "memfault/components.h"
#include "memfault/esp_port/deep_sleep.h"
#include "sdkconfig.h"

static const char *TAG = "deep_sleep";

esp_err_t deep_sleep_start(uint32_t sleep_seconds) {
  esp_err_t err = esp_sleep_enable_timer_wakeup(sleep_seconds * 1000 * 1000);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable timer wakeup: %s", esp_err_to_name(err));
    return ESP_FAIL;
  }

#if defined(CONFIG_MEMFAULT_DEEP_SLEEP_SUPPORT)
  memfault_platform_deep_sleep_save_state();
#endif

  ESP_LOGI(TAG, "🛌 Entering deep sleep for %" PRIu32 " seconds", sleep_seconds);
  esp_deep_sleep_start();

  // This code should never run, but if it does, return an error
  ESP_LOGE(TAG, "Failed to enter deep sleep");
  return ESP_FAIL;
}

#if defined(CONFIG_MEMFAULT_APP_AUTO_DEEP_SLEEP)
static void prv_auto_deep_sleep_start_cb(MEMFAULT_UNUSED TimerHandle_t handle) {
  ESP_LOGI(TAG, "⏰ Deep sleep timer expired");

  (void)deep_sleep_start(CONFIG_MEMFAULT_APP_AUTO_DEEP_SLEEP_PERIOD_SECONDS);
}
#endif

void deep_sleep_wakeup(void) {
#if defined(CONFIG_MEMFAULT_DEEP_SLEEP_SUPPORT)
  memfault_platform_deep_sleep_restore_state();
#endif  // CONFIG_MEMFAULT_DEEP_SLEEP_SUPPORT

#if defined(CONFIG_MEMFAULT_APP_AUTO_DEEP_SLEEP)
  // start a FreeRTOS timer to trigger deep sleep
  const TickType_t deep_sleep_interval =
    pdMS_TO_TICKS(CONFIG_MEMFAULT_APP_AUTO_DEEP_SLEEP_PERIOD_SECONDS * 1000);
  TimerHandle_t timer =
    xTimerCreate("AutoDeepSleep", deep_sleep_interval, pdFALSE, NULL, prv_auto_deep_sleep_start_cb);

  if (timer == NULL) {
    ESP_LOGE(TAG, "Failed to create auto deep sleep timer");
  } else {
    // start the timer
    ESP_LOGI(TAG, "⏱️ Starting auto deep sleep timer for %d seconds",
             CONFIG_MEMFAULT_APP_AUTO_DEEP_SLEEP_PERIOD_SECONDS);
    xTimerStart(timer, 0);
  }
#endif
}
