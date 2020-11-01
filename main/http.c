
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_tls.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "esp_http_client.h"
#include "cJSON.h"

#include "http.h"
#include "config.h"

static const char *TAG = "HTTP_CLIENT";

static SemaphoreHandle_t _xResponse;

#define WEB_SERVER "10.1.5.39"
#define WEB_PORT "3600"
#define WEB_PATH "/"

static uint32_t total_len = 0;
char *response;

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
  switch (evt->event_id)
  {
  case HTTP_EVENT_ERROR:
    ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
    break;
  case HTTP_EVENT_ON_CONNECTED:
    ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
    break;
  case HTTP_EVENT_HEADER_SENT:
    ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
    break;
  case HTTP_EVENT_ON_HEADER:
    ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
    // printf("%.*s", evt->data_len, (char *)evt->data);
    break;
  case HTTP_EVENT_ON_DATA:
    ESP_LOGI(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
    if (!esp_http_client_is_chunked_response(evt->client))
    {
      // Write out data
      memcpy(response + total_len, (char *)evt->data, evt->data_len);
      total_len += evt->data_len;
    }

    break;
  case HTTP_EVENT_ON_FINISH:
    ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
    break;
  case HTTP_EVENT_DISCONNECTED:
    ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
    int mbedtls_err = 0;
    esp_err_t err = esp_tls_get_and_clear_last_error(evt->data, &mbedtls_err, NULL);
    if (err != 0)
    {
      ESP_LOGI(TAG, "Last esp error code: 0x%x", err);
      ESP_LOGI(TAG, "Last mbedtls failure: 0x%x", mbedtls_err);
    }
    break;
  }
  return ESP_OK;
}

static void _storeString(char *val, char *key)
{
  nvs_handle_t my_handle;
  nvs_open(NVS_PART_KEY, NVS_READWRITE, &my_handle);
  nvs_set_str(my_handle, key, val);
  nvs_close(my_handle);
  ESP_LOGI(TAG, "%s stored\n", key);
}

static void _chkUserId(char **userId, char *newUserId)
{
  if (*userId == NULL && newUserId != NULL)
  {
    *userId = malloc(strlen(newUserId) + 1);
    strcpy(*userId, newUserId);
    _storeString(*userId, NVS_KEY_USER_ID);
  }
}

esp_err_t http_get_jwt(const char *usr, const char *pass, char **userId, char *jwt)
{
  xSemaphoreTake(_xResponse, (TickType_t)10);
  total_len = 0;
  // can probs reduce this size
  response = malloc(1024);

  esp_http_client_config_t config = {
      .url = "http://" WEB_SERVER ":" WEB_PORT "/auth",
      .event_handler = _http_event_handler,
      .method = HTTP_METHOD_POST,
  };
  esp_http_client_handle_t client = esp_http_client_init(&config);

  // POST
  cJSON *post_data = cJSON_CreateObject();
  cJSON_AddStringToObject(post_data, "email", usr);
  cJSON_AddStringToObject(post_data, "password", pass);

  esp_http_client_set_post_field(client, cJSON_Print(post_data), strlen(cJSON_Print(post_data)));
  esp_http_client_set_header(client, "Content-Type", "application/json");
  esp_err_t err = esp_http_client_perform(client);
  if (err == ESP_OK)
  {
    ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d",
             esp_http_client_get_status_code(client),
             esp_http_client_get_content_length(client));
  }
  else
  {
    ESP_LOGE(TAG, "HTTP POST request failed: %s\n", esp_err_to_name(err));
    esp_http_client_cleanup(client);
    cJSON_Delete(post_data);
    free(response);
    xSemaphoreGive(_xResponse);
    return ESP_FAIL;
  }

  cJSON *root = cJSON_Parse(response);

  printf("response %s\n", cJSON_Print(root));

  strcat(jwt, cJSON_GetObjectItem(root, "tokenType")->valuestring);
  strcat(jwt, " ");
  strcat(jwt, cJSON_GetObjectItem(root, "accessToken")->valuestring);

  _chkUserId(userId, cJSON_GetObjectItem(root, "userId")->valuestring);

  printf("UserId %s\n", *userId);

  esp_http_client_cleanup(client);
  cJSON_Delete(post_data);
  cJSON_Delete(root);
  free(response);
  xSemaphoreGive(_xResponse);
  return ESP_OK;
}

esp_err_t http_reg_device(const char *jwt, char **deviceId)
{
  xSemaphoreTake(_xResponse, (TickType_t)10);
  total_len = 0;
  // can probs reduce this size
  response = malloc(1024);

  esp_http_client_config_t config = {
      .url = "http://" WEB_SERVER ":" WEB_PORT "/device",
      .event_handler = _http_event_handler,
      .method = HTTP_METHOD_POST,
  };

  esp_http_client_handle_t client = esp_http_client_init(&config);

  cJSON *post_data = cJSON_CreateObject();
  cJSON_AddStringToObject(post_data, "type", "CellStretcher");
  char id[15];
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);

  sprintf(id, "%s_%02x%02X%02X", "CS", mac[3], mac[4], mac[5]);
  cJSON_AddStringToObject(post_data, "name", id);

  sprintf(id, "%s_%02x%02X%02X", "ESP32", mac[3], mac[4], mac[5]);
  cJSON_AddStringToObject(post_data, "clientId", id);

  printf("POST DATA %s", cJSON_Print(post_data));

  esp_http_client_set_post_field(client, cJSON_Print(post_data), strlen(cJSON_Print(post_data)));
  esp_http_client_set_header(client, "Content-Type", "application/json");
  esp_http_client_set_header(client, "Authorization", jwt);

  esp_err_t err = esp_http_client_perform(client);
  if (err == ESP_OK)
  {
    ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d",
             esp_http_client_get_status_code(client),
             esp_http_client_get_content_length(client));
  }
  else
  {
    ESP_LOGE(TAG, "HTTP POST request failed: %s\n", esp_err_to_name(err));
    esp_http_client_cleanup(client);
    free(response);
    xSemaphoreGive(_xResponse);
    return ESP_FAIL;
  }

  cJSON *device = cJSON_Parse(response);

  char *newDevID = cJSON_GetObjectItem(device, "id")->valuestring;

  *deviceId = malloc(strlen(newDevID) + 1);
  strcpy(*deviceId, newDevID);
  _storeString(*deviceId, NVS_KEY_DEVICE_ID);

  ESP_LOGE(TAG, "Device Registered: %s\n", *deviceId);

  esp_http_client_cleanup(client);
  free(response);
  xSemaphoreGive(_xResponse);
  return ESP_OK;
}

esp_err_t http_get_device(const char *deviceId, const char *jwt)
{
  xSemaphoreTake(_xResponse, (TickType_t)10);
  total_len = 0;
  // can probs reduce this size
  response = malloc(1024);

  char url[100];
  strcpy(url, "http://" WEB_SERVER ":" WEB_PORT "/device/");
  strcat(url, deviceId);

  esp_http_client_config_t config = {
      .url = url,
      .event_handler = _http_event_handler,
      .method = HTTP_METHOD_GET,
  };
  esp_http_client_handle_t client = esp_http_client_init(&config);

  esp_http_client_set_url(client, url);
  esp_http_client_set_header(client, "Content-Type", "application/json");
  esp_http_client_set_header(client, "Authorization", jwt);
  esp_err_t err = esp_http_client_perform(client);
  if (err == ESP_OK)
  {
    ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d",
             esp_http_client_get_status_code(client),
             esp_http_client_get_content_length(client));
  }
  else
  {
    ESP_LOGE(TAG, "HTTP POST request failed: %s\n", esp_err_to_name(err));
    esp_http_client_cleanup(client);
    free(response);
    xSemaphoreGive(_xResponse);
    return ESP_FAIL;
  }

  cJSON *device = cJSON_Parse(response);

  printf("device %s\n", cJSON_Print(device));

  esp_http_client_cleanup(client);
  free(response);
  xSemaphoreGive(_xResponse);
  return ESP_OK;
}

void http_init(void)
{
  _xResponse = xSemaphoreCreateMutex();
}