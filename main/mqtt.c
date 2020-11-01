
#include "esp_log.h"
#include "mqtt_client.h"

#include "mqtt.h"
#include "cJSON.h"
#include "Motion.h"

static const char *TAG = "MQTT";

static bool MQTTConnected = false;
esp_mqtt_client_handle_t Client;

static char UserId[40];
static char DeviceId[40];

static char *Jwt;

static void _mkTopic(char *custLevel, char *topic)
{
  strcpy(topic, UserId);
  strcat(topic, "/");
  strcat(topic, DeviceId);
  strcat(topic, "/");
  strcat(topic, custLevel);
}

enum cmd
{
  CMD_MANUAL,
  CMD_HOME
};

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
  esp_mqtt_client_handle_t client = event->client;
  char topic[100];
  int msgId;
  // your_context_t *context = event->context;
  switch (event->event_id)
  {
  case MQTT_EVENT_CONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
    MQTTConnected = true;
    _mkTopic("General", topic);
    msgId = esp_mqtt_client_subscribe(client, topic, 0);
    ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msgId);
    break;
  case MQTT_EVENT_DISCONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
    MQTTConnected = false;
    break;
  case MQTT_EVENT_ERROR:
    ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
    break;
  case MQTT_EVENT_DATA:
    printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
    printf("DATA=%.*s\r\n", event->data_len, event->data);
    cJSON *root = cJSON_Parse(event->data);
    int cmd = cJSON_GetObjectItem(root, "cmd")->valueint;
    if (cmd == CMD_MANUAL)
    {
      cJSON *data = cJSON_GetObjectItem(root, "data");
      // printf("M1_RUN: %d, M1_DIR: %d\n", cJSON_GetObjectItem(data, "M1_RUN")->valueint, cJSON_GetObjectItem(data, "M1_DIR")->valueint);
      motion_event_t motionData = {
          .motor1.motorDir = cJSON_GetObjectItem(data, "M1_DIR")->valueint,
          .motor1.motorRun = cJSON_GetObjectItem(data, "M1_RUN")->valueint,
          .motor1.motorFreq = cJSON_GetObjectItem(data, "M1_FREQ")->valueint,
          .motor2.motorDir = cJSON_GetObjectItem(data, "M2_DIR")->valueint,
          .motor2.motorRun = cJSON_GetObjectItem(data, "M2_RUN")->valueint,
      };

      ESP_ERROR_CHECK(esp_event_post_to(MOTION_TASK, MOTION_EVENTS, MOTION_CMD_EVENT, &motionData, sizeof(motionData), portMAX_DELAY));
    }
    else if (cmd == CMD_HOME)
    {
      cJSON *data = cJSON_GetObjectItem(root, "data");
      int motor = cJSON_GetObjectItem(data, "motor")->valueint;
      printf("posted\n");

      ESP_ERROR_CHECK(esp_event_post_to(MOTION_TASK, MOTION_EVENTS, MOTION_HOME_EVENT, &motor, sizeof(motor), portMAX_DELAY));
    }
    else
    {
      ESP_LOGI(TAG, "Other CMD id:%d", cmd);
    }

    cJSON_Delete(root);
    break;
  default:
    ESP_LOGI(TAG, "Other event id:%d", event->event_id);
    break;
  }
  return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
  ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
  mqtt_event_handler_cb(event_data);
}

static void _task_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
  motor_t *Motor_Status = (motor_t *)event_data;
  int drv;
  char topic[100];
  cJSON *payload = cJSON_CreateObject();
  cJSON_AddStringToObject(payload, "jwt", Jwt);
  cJSON *data = cJSON_CreateArray();
  cJSON_AddItemToObject(payload, "data", data);
  cJSON *object;

  switch (id)
  {
  case MOTION_STATUS_EVENT:
    _mkTopic("status", topic);
    // data =
    // object = cJSON_CreateObject();

    for (drv = 0; drv < 2; drv++)
    {
      object = cJSON_CreateObject();
      cJSON_AddNumberToObject(object, "error", Motor_Status[drv].error);
      cJSON_AddNumberToObject(object, "dir", Motor_Status[drv].motorDir);
      cJSON_AddNumberToObject(object, "run", Motor_Status[drv].motorRun);
      cJSON_AddNumberToObject(object, "homed", Motor_Status[drv].homed);
      cJSON_AddNumberToObject(object, "freq", Motor_Status[drv].motorFreq);
      cJSON_AddItemToArray(data, object);
    }
    printf("DATA: %s\n", cJSON_Print(payload));
    esp_mqtt_client_publish(Client, topic, cJSON_Print(payload), 0, 1, 0);

    break;
  default:
    ESP_LOGI(TAG, "Other event id:%d", id);
    break;
  }
  cJSON_Delete(payload); // Deletes the array and its contents
}

void mqtt_init(const char *jwt, const char *userId, const char *deviceId)
{
  strcpy(UserId, userId);
  strcpy(DeviceId, deviceId);

  Jwt = jwt;

  esp_mqtt_client_config_t mqtt_cnfg = {
      .uri = "mqtt://10.1.5.39",
      .port = 8883,
      .username = "rct@test.com",
      .password = jwt,
  };

  Client = esp_mqtt_client_init(&mqtt_cnfg);
  esp_mqtt_client_register_event(Client, ESP_EVENT_ANY_ID, mqtt_event_handler, Client);
  esp_mqtt_client_start(Client);
  ESP_ERROR_CHECK(esp_event_handler_register_with(MOTION_TASK, MOTION_EVENTS, MOTION_STATUS_EVENT, _task_handler, NULL));
}