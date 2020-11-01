/*
 * Motion.c
 *
 *  Created on: 4 Jun 2020
 *      Author: rob_c
 */
#include "Motion.h"
#include "tmc/ic/TMC2130/TMC2130.h"
#include "SPI.h"
#include "string.h"
#include "driver/ledc.h" // PWM Controls
#include <freertos/task.h>
#include "board.h"
#include "esp_log.h"
// #include "soc.h"

/* Event source task related definitions */
ESP_EVENT_DEFINE_BASE(MOTION_EVENTS);

#define LEDC_HS_TIMER LEDC_TIMER_0
#define LEDC_HS_MODE LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_CHANNEL LEDC_CHANNEL_0
#define PWM_RUN_DUTY (32) // 50% at 6-bit resolution
#define PWM_CH_NUM 2

#define MOTION_DRV_NUM 2
#define MOTION_DRV_ID_1 0
#define MOTION_DRV_ID_2 1
#define MOTION_EXPAND 1
#define MOTION_CONTRACT 0
#define MOTION_RUN 1
#define MOTION_STOP 0
#define MOTION_DEFAULT_FREQ 100000

#define HOME_THRES 50

static const char *TAG = "Motion";

//bit mask of the pins that you want to set
#define GPIO_DIR_PIN_SEL ((1ULL << GPIO_DRV_DIR_1) | (1ULL << GPIO_DRV_DIR_2))

static ledc_channel_config_t pwm_channels[PWM_CH_NUM] = {
    {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = GPIO_DRV_STP_1,
        .speed_mode = LEDC_HS_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_HS_TIMER,
    },
    {
        .channel = LEDC_CHANNEL_1,
        .duty = 0,
        .gpio_num = GPIO_DRV_STP_2,
        .speed_mode = LEDC_HS_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_HS_TIMER,
    },
};

// Array for driver spi interfaces
spi_device_handle_t Driver_Interface[2];
TMC2130TypeDef Driver_Setup[2];
ConfigurationTypeDef Driver_Config[2];
motor_t Motor_Status[2];

// => SPI wrapper
// Send [length] bytes stored in the [data] array over SPI and overwrite [data]
// with the reply. The first byte sent/received is data[0].
void tmc2130_readWriteArray(uint8_t channel, uint8_t *data, size_t length)
{
  esp_err_t ret;

  // allocate memory on heap for local globals
  uint8_t txdata[length];
  uint8_t rxdata[length];

  spi_transaction_t trans;
  memset(&trans, 0, sizeof(spi_transaction_t));
  //	trans.addr = data[0];
  trans.length = 8 * length;
  trans.rxlength = 8 * length;
  trans.tx_buffer = txdata;
  trans.rx_buffer = rxdata;
  //	trans.flags=SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;

  memcpy(txdata, data, length);

  ret = spi_device_transmit(Driver_Interface[channel], &trans);
  ESP_ERROR_CHECK(ret);

  memcpy(data, rxdata, length);
}

void _updateStatus()
{
  // int drv;
  // for (drv = 0; drv < MOTION_DRV_NUM; drv++)
  // {
  //   printf("Motor %d status:\n", drv + 1);
  //   printf("Error %d; ", Motor_Status[drv].error);
  //   printf("DIR %d; ", Motor_Status[drv].motorDir);
  //   printf("RUN %d; ", Motor_Status[drv].motorRun);
  //   printf("HOMED %d; ", Motor_Status[drv].homed);
  //   printf("FREQ %d;\n", Motor_Status[drv].motorFreq);
  // }

  ESP_ERROR_CHECK(esp_event_post_to(MOTION_TASK, MOTION_EVENTS, MOTION_STATUS_EVENT, Motor_Status, 2 * sizeof(Motor_Status), portMAX_DELAY));
}

void _initDriver(spi_device_interface_config_t devcfg, int drvId)
{
  esp_err_t ret;

  //Attach the Driver IC to the SPI bus
  ret = spi_bus_add_device(VSPI_HOST, &devcfg, &Driver_Interface[drvId]);
  ESP_ERROR_CHECK(ret);

  // Initialise drivers
  tmc2130_init(&Driver_Setup[drvId], drvId, &Driver_Config[drvId], tmc2130_defaultRegisterResetState);

  // The below register settings were generated using the TMC-IDE 3.0
  tmc2130_writeInt(&Driver_Setup[drvId], 0x00, 0x00000004); // writing value 0x00000000 = 0 = 0.0 to address 0 = 0x00(GCONF)
  tmc2130_writeInt(&Driver_Setup[drvId], 0x10, 0x00000402); // writing value 0x00071703 = 464643 = 0.0 to address 1 = 0x10(IHOLD_IRUN)
  tmc2130_writeInt(&Driver_Setup[drvId], 0x11, 0x00000000); // writing value 0x00000000 = 0 = 0.0 to address 2 = 0x11(TPOWERDOWN)
  tmc2130_writeInt(&Driver_Setup[drvId], 0x13, 0x00000000); // writing value 0x00000000 = 0 = 0.0 to address 3 = 0x13(TPWMTHRS)
  tmc2130_writeInt(&Driver_Setup[drvId], 0x14, 0x00000010); // writing value 0x00000010 = 16 = 0.0 to address 4 = 0x14(TCOOLTHRS)
  tmc2130_writeInt(&Driver_Setup[drvId], 0x15, 0x00000000); // writing value 0x00000000 = 0 = 0.0 to address 5 = 0x15(THIGH)
  tmc2130_writeInt(&Driver_Setup[drvId], 0x2D, 0x00000000); // writing value 0x00000000 = 0 = 0.0 to address 6 = 0x2D(XDIRECT)
  tmc2130_writeInt(&Driver_Setup[drvId], 0x33, 0x00000000); // writing value 0x00000000 = 0 = 0.0 to address 7 = 0x33(VDCMIN)
  tmc2130_writeInt(&Driver_Setup[drvId], 0x60, 0xAAAAB554); // writing value 0xAAAAB554 = 0 = 0.0 to address 8 = 0x60(MSLUT[0])
  tmc2130_writeInt(&Driver_Setup[drvId], 0x61, 0x4A9554AA); // writing value 0x4A9554AA = 1251300522 = 0.0 to address 9 = 0x61(MSLUT[1])
  tmc2130_writeInt(&Driver_Setup[drvId], 0x62, 0x24492929); // writing value 0x24492929 = 608774441 = 0.0 to address 10 = 0x62(MSLUT[2])
  tmc2130_writeInt(&Driver_Setup[drvId], 0x63, 0x10104222); // writing value 0x10104222 = 269500962 = 0.0 to address 11 = 0x63(MSLUT[3])
  tmc2130_writeInt(&Driver_Setup[drvId], 0x64, 0xFBFFFFFF); // writing value 0xFBFFFFFF = 0 = 0.0 to address 12 = 0x64(MSLUT[4])
  tmc2130_writeInt(&Driver_Setup[drvId], 0x65, 0xB5BB777D); // writing value 0xB5BB777D = 0 = 0.0 to address 13 = 0x65(MSLUT[5])
  tmc2130_writeInt(&Driver_Setup[drvId], 0x66, 0x49295556); // writing value 0x49295556 = 1227445590 = 0.0 to address 14 = 0x66(MSLUT[6])
  tmc2130_writeInt(&Driver_Setup[drvId], 0x67, 0x00404222); // writing value 0x00404222 = 4211234 = 0.0 to address 15 = 0x67(MSLUT[7])
  tmc2130_writeInt(&Driver_Setup[drvId], 0x68, 0xFFFF8056); // writing value 0xFFFF8056 = 0 = 0.0 to address 16 = 0x68(MSLUTSEL)
  tmc2130_writeInt(&Driver_Setup[drvId], 0x69, 0x00F70000); // writing value 0x00F70000 = 16187392 = 0.0 to address 17 = 0x69(MSLUTSTART)
  tmc2130_writeInt(&Driver_Setup[drvId], 0x6C, 0x010101D5); // writing value 0x000101D5 = 66005 = 0.0 to address 18 = 0x6C(CHOPCONF)
  tmc2130_writeInt(&Driver_Setup[drvId], 0x6D, 0x01002041); // writing value 0x01002041 = 16785473 = 0.0 to address 19 = 0x6D(COOLCONF)
  tmc2130_writeInt(&Driver_Setup[drvId], 0x6E, 0x00000000); // writing value 0x00000000 = 0 = 0.0 to address 20 = 0x6E(DCCTRL)
  tmc2130_writeInt(&Driver_Setup[drvId], 0x70, 0x000504C8); // writing value 0x000504C8 = 328904 = 0.0 to address 21 = 0x70(PWMCONF)
  tmc2130_writeInt(&Driver_Setup[drvId], 0x72, 0x00000000); // writing value 0x00000000 = 0 = 0.0 to address 22 = 0x72(ENCM_CTRL)

  Motor_Status[drvId].error = false;
  Motor_Status[drvId].homed = false;
  Motor_Status[drvId].motorDir = false;
  Motor_Status[drvId].motorRun = false;
  Motor_Status[drvId].motorFreq = MOTION_DEFAULT_FREQ;
}

void _set_freq(uint32_t freq)
{
  // ledc_set_freq(LEDC_HS_MODE, LEDC_HS_TIMER, freq);
  // ledc_channel_config(&pwm_channel);

  // ledc_timer_pause(LEDC_HS_MODE, LEDC_HS_TIMER);

  ledc_timer_config_t pwm_timer = {
      .duty_resolution = LEDC_TIMER_6_BIT, // resolution of PWM duty
      .freq_hz = freq,                     // frequency of PWM signal
      .speed_mode = LEDC_HS_MODE,          // timer mode
      .timer_num = LEDC_HS_TIMER,          // timer index
      .clk_cfg = LEDC_AUTO_CLK,            // Auto select the source clock
  };

  int drv;
  for (drv = 0; drv < MOTION_DRV_NUM; drv++)
  {
    Motor_Status[drv].motorFreq = freq;
  }

  // Set configuration of timer0 for high speed channels
  ledc_timer_config(&pwm_timer);
}

// true to resume stepping false to stop
void _run_motion(bool run, int drvId)
{
  uint32_t duty;

  duty = run ? PWM_RUN_DUTY : 0;

  Motor_Status[drvId].motorRun = run;

  ledc_set_duty(pwm_channels[drvId].speed_mode, pwm_channels[drvId].channel, duty);
  ledc_update_duty(pwm_channels[drvId].speed_mode, pwm_channels[drvId].channel);
}

// true = clockwise, false = anti clockwise
void _set_dir(bool clockwise, int drvId)
{
  if (drvId == MOTION_DRV_ID_1)
  {
    gpio_set_level(GPIO_DRV_DIR_1, clockwise);
    Motor_Status[drvId].motorDir = clockwise;
  }
  else if (drvId == MOTION_DRV_ID_2)
  {
    gpio_set_level(GPIO_DRV_DIR_2, !clockwise);
    Motor_Status[drvId].motorDir = !clockwise;
  }
}

void _home_motor(int drvId)
{
  ESP_LOGI(TAG, "Homing MOTOR %d", drvId + 1);
  uint32_t gconf, drvStatus, sgRes, sgResOrig, sgDiff, curFreq;
  bool home = true;

  curFreq = Motor_Status[drvId].motorFreq;

  _run_motion(MOTION_RUN, drvId);
  _set_dir(MOTION_EXPAND, drvId);
  _set_freq(MOTION_DEFAULT_FREQ);

  gconf = tmc2130_readInt(&Driver_Setup[drvId], TMC2130_GCONF);
  gconf &= ~TMC2130_EN_PWM_MODE_MASK; // disable stealChop
  tmc2130_writeInt(&Driver_Setup[drvId], TMC2130_GCONF, gconf);

  _run_motion(true, drvId);

  vTaskDelay(1000 / portTICK_PERIOD_MS);

  drvStatus = tmc2130_readInt(&Driver_Setup[drvId], TMC2130_DRV_STATUS);
  sgResOrig = TMC2130_SG_RESULT_MASK & drvStatus;

  while (home)
  {
    // sgResOld = sgRes;
    drvStatus = tmc2130_readInt(&Driver_Setup[drvId], TMC2130_DRV_STATUS);
    sgRes = TMC2130_SG_RESULT_MASK & drvStatus;

    if (sgResOrig > sgRes)
    {
      sgDiff = sgResOrig - sgRes;
      // printf("SGRES: %d, SGRESOLD: %d, Diff: %d\n", sgRes, sgResOrig, sgDiff);

      if (sgDiff > HOME_THRES)
      {
        _run_motion(MOTION_STOP, drvId);
        home = false;
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }

  gconf = tmc2130_readInt(&Driver_Setup[drvId], TMC2130_GCONF);
  gconf |= TMC2130_EN_PWM_MODE_MASK; // enable stealChop
  tmc2130_writeInt(&Driver_Setup[drvId], TMC2130_GCONF, gconf);

  ESP_LOGI(TAG, "MOTOR %d is homed", drvId + 1);
  Motor_Status[drvId].homed = true;

  //store postion as zero

  // move away slightly
  _set_dir(MOTION_CONTRACT, drvId);
  _run_motion(MOTION_RUN, drvId);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  _run_motion(MOTION_STOP, drvId);
  _set_freq(curFreq);
}

// initilises with 0% duty
void _init_PWM()
{
  /*
	 * Prepare and set configuration of timers
	 * that will be used by LED Controller
	 */
  ledc_timer_config_t pwm_timer = {
      .duty_resolution = LEDC_TIMER_6_BIT, // resolution of PWM duty
      .freq_hz = MOTION_DEFAULT_FREQ,      // frequency of PWM signal
      .speed_mode = LEDC_HS_MODE,          // timer mode
      .timer_num = LEDC_HS_TIMER,          // timer index
      .clk_cfg = LEDC_AUTO_CLK,            // Auto select the source clock
  };

  // Set configuration of timer0 for high speed channels
  ledc_timer_config(&pwm_timer);

  // Set LED Controller with previously prepared configuration
  int ch;
  for (ch = 0; ch < PWM_CH_NUM; ch++)
  {
    ledc_channel_config(&pwm_channels[ch]);
  }
}

void _initGPIO()
{
  // Config Direction pin
  gpio_config_t io_conf;
  //disable interrupt
  io_conf.intr_type = GPIO_INTR_DISABLE;
  //set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  //bit mask of the pins that you want to set
  io_conf.pin_bit_mask = GPIO_DIR_PIN_SEL;
  //disable pull-down mode
  io_conf.pull_down_en = true;
  //disable pull-up mode
  io_conf.pull_up_en = false;
  //configure GPIO with the given settings
  gpio_config(&io_conf);
}

void getDriverStat()
{
  int32_t DRVSTAT = tmc2130_readInt(&Driver_Setup[0], TMC2130_DRV_STATUS);
  printf("DRVSTAT = 0x%x\n", DRVSTAT);
}

static void _task_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
  motion_event_t *data;
  int *drvId;
  printf("ID %d\n", id);
  switch (id)
  {
  case MOTION_CMD_EVENT:
    data = (motion_event_t *)event_data;
    _run_motion(data->motor1.motorRun, MOTION_DRV_ID_1);
    _run_motion(data->motor2.motorRun, MOTION_DRV_ID_2);
    _set_dir(data->motor1.motorDir, MOTION_DRV_ID_1);
    _set_dir(data->motor2.motorDir, MOTION_DRV_ID_2);
    _set_freq(data->motor1.motorFreq);

    ESP_LOGI(TAG, "Motor Dir: %d", data->motor1.motorDir);
    ESP_LOGI(TAG, "Motor Run: %d", data->motor1.motorRun);
    break;
  case MOTION_HOME_EVENT:

    drvId = (int *)event_data;
    printf("here drvID %d\n", *drvId);
    _home_motor(*drvId);
    break;
  default:
    ESP_LOGI(TAG, "Other event id:%d", id);
    break;
  }
  _updateStatus();
}

void vStatusInitTask(void *pvParameters)
{
  vTaskDelay(10000 / portTICK_PERIOD_MS);
  _updateStatus();
  vTaskDelete(NULL);
}

void motion_init(void)
{

  spi_device_interface_config_t devcfg = {
      .clock_speed_hz = 4 * 1000 * 1000, //Clock out at 4 MHz
      .mode = 3,                         //SPI mode 3
      .spics_io_num = GPIO_CS_1,         //CS pin
      .queue_size = 7,                   //We want to be able to queue 7 transactions at a time
                                         //			.address_bits=8
  };

  _initDriver(devcfg, MOTION_DRV_ID_1);

  devcfg.spics_io_num = GPIO_CS_2;
  _initDriver(devcfg, MOTION_DRV_ID_2);

  esp_event_loop_args_t motion_task_args = {
      .queue_size = 5,
      .task_name = "motion_task", // task will be created
      .task_priority = 5,
      .task_stack_size = 3072,
      .task_core_id = 1,
  };

  ESP_ERROR_CHECK(esp_event_loop_create(&motion_task_args, &MOTION_TASK));

  // Register the handler for task iteration event. Notice that the same handler is used for handling event on different loops.
  // The loop handle is provided as an argument in order for this example to display the loop the handler is being run on.
  ESP_ERROR_CHECK(esp_event_handler_register_with(MOTION_TASK, MOTION_EVENTS, MOTION_CMD_EVENT, _task_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register_with(MOTION_TASK, MOTION_EVENTS, MOTION_HOME_EVENT, _task_handler, NULL));

  _init_PWM();
  _initGPIO();
  // _updateStatus(); event doesn't have a handler registered yet so doesn't work
  // _home_motor(MOTION_DRV_ID_1);

  // xTaskCreate(&vMotionTask, "Motion_task_man", 2048, NULL, 5, NULL);
  // Task to send current status to server on boot
  xTaskCreate(&vStatusInitTask, "status_task_init", 1024, NULL, 4, NULL);
}

void vMotionTask(void *pvParameters)
{

  // int cnt = 1;
  while (1)
  {

    getDriverStat();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}