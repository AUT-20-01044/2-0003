#include "stdio.h"
#include "string.h"

#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"

#include "board.h"
#include "config.h"
#include "Motion.h"
#include "SPI.h"
#include "comms.h"

esp_err_t err;

char *wifiSsid, *wifiPass;
char *userEmail, *userPass, *userId;
char *deviceId;

esp_err_t
event_handler(void *ctx, system_event_t *event)
{
  return ESP_OK;
}
static const char *TAG = "MAIN";

#define GPIO_OUTPUT_PIN_SEL ((1ULL << GPIO_LED_ERR) | (1ULL << GPIO_LED_STS))
#define GPIO_INPUT_PIN_SEL ((1ULL << GPIO_BTN_RST))

static void nvsString(nvs_handle_t handle, char *key, char **val)
{
  size_t reqSize;
  err = nvs_get_str(handle, key, NULL, &reqSize);
  if (err != ESP_OK)
  {
    // *val = NULL;
  }
  else
  {
    *val = malloc(reqSize);
    nvs_get_str(handle, key, *val, &reqSize);
  }
}

static void nvsStringProv(nvs_handle_t handle, char *key, char **val)
{
  size_t reqSize;
  err = nvs_get_str(handle, key, NULL, &reqSize);
  if (err != ESP_OK)
  {
    char line[128];

    int count = 0;
    printf("Please enter default value for %s\n", key);
    while (count < 128)
    {
      int c = fgetc(stdin);
      if (c == '\n')
      {
        line[count] = '\0';
        break;
      }
      else if (c > 0 && c < 127)
      {
        line[count] = c;
        ++count;
      }
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    err = nvs_set_str(handle, key, line);
    *val = malloc(sizeof(line));
    strcpy(*val, line);
    printf("%s : %s\n", key, line);
  }
  else
  {
    *val = malloc(reqSize);
    nvs_get_str(handle, key, *val, &reqSize);
  }
}

static esp_err_t initNvs(void)
{
  //Initialize NVS
  printf("btn lvl: %d\n", gpio_get_level(GPIO_BTN_RST));
  if (gpio_get_level(GPIO_BTN_RST))
  {
    nvs_flash_erase();
    gpio_set_level(GPIO_LED_STS, true);
  }
  err = nvs_flash_init();
  if (err != ESP_OK)
  {
    printf("Error %s\n", esp_err_to_name(err));
  }

  err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    // NVS partition was truncated and needs to be erased
    // Retry nvs_flash_init
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  // get handle
  nvs_handle_t my_handle;
  err = nvs_open(NVS_PART_KEY, NVS_READWRITE, &my_handle); // currently using the default partition
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "Error (%s) opening NVS handle\n", esp_err_to_name(err));
    return ESP_FAIL;
  }
  else
  {
    // Read
    // Wifi SSID
    nvsStringProv(my_handle, NVS_KEY_WIFI_SSID, &wifiSsid);

    // Wifi Password
    nvsStringProv(my_handle, NVS_KEY_WIFI_PASS, &wifiPass);

    // User Email
    nvsStringProv(my_handle, NVS_KEY_USER_EMAIL, &userEmail);

    // User Password
    nvsStringProv(my_handle, NVS_KEY_USER_PASS, &userPass);

    // User Id
    nvsString(my_handle, NVS_KEY_USER_ID, &userId);

    // device Id
    nvsString(my_handle, NVS_KEY_DEVICE_ID, &deviceId);

    err = nvs_commit(my_handle);
    if (err == ESP_OK)
    {
      ESP_LOGI(TAG, "NVS Success\n");
    }
    else
    {
      ESP_LOGE(TAG, "NVS Failed\n");
    }

    // Close
    nvs_close(my_handle);
    gpio_set_level(GPIO_LED_STS, false);
  }
  nvs_stats_t nvs_stats;
  nvs_get_stats(NULL, &nvs_stats);
  printf("Count: UsedEntries = (%d), FreeEntries = (%d), AllEntries = (%d)\n",
         nvs_stats.used_entries, nvs_stats.free_entries, nvs_stats.total_entries);

  return ESP_OK;
}

static void _gpioInit()
{

  // LED INIT
  gpio_config_t io_conf;
  //disable interrupt
  io_conf.intr_type = GPIO_INTR_DISABLE;
  //set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  //bit mask of the pins that you want to set,e.g.GPIO18/19
  io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
  //disable pull-down mode
  io_conf.pull_down_en = 0;
  //disable pull-up mode
  io_conf.pull_up_en = 0;
  //configure GPIO with the given settings
  gpio_config(&io_conf);

  //interrupt of rising edge
  // io_conf.intr_type = GPIO_INTR_POSEDGE;
  //bit mask of the pins, use GPIO4/5 here
  io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
  //set as input mode
  io_conf.mode = GPIO_MODE_INPUT;
  gpio_config(&io_conf);
}

void app_main(void)
{
  _gpioInit();

  ESP_ERROR_CHECK(initNvs());

  // Initialise SPI interface for main SPI bus
  spi_init();
  // Initiliase Motion module
  motion_init();

  TCommsSetup commsSetup;
  commsSetup.wifiSsid = wifiSsid;
  commsSetup.wifiPass = wifiPass;
  commsSetup.userEmail = userEmail;
  commsSetup.userPass = userPass;
  commsSetup.userId = &userId;
  commsSetup.deviceId = &deviceId;

  comms_init(&commsSetup);

  printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

  // int cnt = 0;
  // while (1)
  // {
  //   printf("LED cnt: %d\n", cnt++);
  //   vTaskDelay(1000 / portTICK_RATE_MS);
  //   gpio_set_level(GPIO_LED_ERR, cnt % 2);
  //   gpio_set_level(GPIO_LED_STS, cnt % 2);
  // }
}
