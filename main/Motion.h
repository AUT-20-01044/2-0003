/*
 * Motion.h
 *
 *  Created on: 4 Jun 2020
 *      Author: rob_c
 */

#ifndef MAIN_MOTION_H_
#define MAIN_MOTION_H_

#include "esp_event.h"
// #include "event_source.h"
#include "esp_event_base.h"

esp_event_loop_handle_t MOTION_TASK;

ESP_EVENT_DECLARE_BASE(MOTION_EVENTS); // declaration of the task events family

enum
{
  MOTION_CMD_EVENT, // raised during an iteration of the loop within the task
  MOTION_HOME_EVENT,
  MOTION_STATUS_EVENT,
};

typedef struct
{
  int motorDir;
  int motorRun;
  uint32_t motorFreq;
  int homed;
  int error;
} motor_t;
typedef struct
{
  motor_t motor1;
  motor_t motor2;
} motion_event_t;

void motion_init(void);

void vMotionTask(void *pvParameters);

#endif /* MAIN_MOTION_H_ */
