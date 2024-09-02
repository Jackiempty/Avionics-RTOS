#include <esp_log.h>
#include <esp_vfs.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <inttypes.h>
#include <stdio.h>

#include "bsp.h"
#include "driver/gpio.h"
#include "hal/gpio_types.h"
#include "ra01s.h"
#include "recv.h"
#include "sdkconfig.h"
#include "slave.h"
#include "task/fsm.h"
#include "task/logger.h"
#include "task/sensors.h"
#include "task/wdt.h"

static int log_vprintf(const char *fmt, va_list arguments) {
  static FILE *f = NULL;
  static int ret;
  f = storage_fetch();
  if (f != NULL)
    ret = vfprintf(f, fmt, arguments);
  else
    ret = vprintf(fmt, arguments);
  if (ret < 0)
    printf("Logging error: %d\n", ret);
  storage_flush();
  return ret;
}

void app_main() {
  gpio_init();
  i2c_init();
  uart_init();
  spi_init(LORA_SPI_HOST, CONFIG_LORA_MOSI_GPIO, CONFIG_LORA_MISO_GPIO,
           CONFIG_LORA_SCK_GPIO);
  spi_init(SD_SPI_HOST, CONFIG_SD_MOSI_GPIO, CONFIG_SD_MISO_GPIO,
           CONFIG_SD_SCK_GPIO);
  lora_init();
  // slave_reset(); // This makes sd_init malfunction

  if (sd_init() == ESP_OK) {
    printf("I am an on-board avionics board!\n");
    printf("stop_0\n");
    storage_init(NULL);
    printf("stop_1\n");
    esp_log_set_vprintf(log_vprintf);
    printf("stop_2\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("stop_3\n");
    xTaskCreatePinnedToCore(fsm_task, "fsm_task", 4096, NULL, 5, NULL, 1);
    printf("stop_4\n");
    // xTaskCreatePinnedToCore(sensors_task, "sensors_task", 8192, NULL, 4,
    // NULL, 1);
    printf("stop_5\n");
    // xTaskCreatePinnedToCore(logger_task, "logger_task", 4096, NULL, 3, NULL,
    // 0);
    printf("stop_6\n");
    // xTaskCreatePinnedToCore(wdt_task, "wdt_task", 2048, NULL, 1, NULL, 1);
  } else {
    printf("I am a ground receiver board!\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    xTaskCreatePinnedToCore(recv_task, "recv_task", 8192, NULL, 5, NULL, 1);
  }

  printf("Minimum free heap size: %" PRIu32 " bytes\n",
         esp_get_minimum_free_heap_size());
}
