#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "freertos/semphr.h"

#include "wifi.h"
#include "http.h"
#include "mqtt.h"
#include "comms.h"

static const char *TAG = "COMMS";

SemaphoreHandle_t xJWTAccess;

static char jwt[512];

static void _commsTask(void *pvParameters)
{
  // comms_init();
  while (1)
  {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void comms_init(const TCommsSetup *const commsSetup)
{

  ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
  wifi_init_sta(commsSetup->wifiSsid, commsSetup->wifiPass);

  ESP_LOGI(TAG, "ESP_HTTP_MODE_TCP");
  http_init();
  http_get_jwt(commsSetup->userEmail, commsSetup->userPass, commsSetup->userId, jwt);

  if (*(commsSetup->deviceId) == NULL)
  {
    // register device
    http_reg_device(jwt, commsSetup->deviceId);
  }
  // http_get_device(device, "5f4f554538c3786e9c36c899", jwt);

  ESP_LOGI(TAG, "ESP_MQTT_MODE_TCP");
  mqtt_init(jwt, *(commsSetup->userId), *(commsSetup->deviceId));

  xTaskCreate(&_commsTask, "Comms_Task", 2048, NULL, 5, NULL);
}