/*
 * comms.h
 *
 *  Created on: 12 Sep 2020
 *      Author: rob_c
 */

#ifndef MAIN_COMMS_H_
#define MAIN_COMMS_H_
typedef struct
{
  char *wifiSsid;
  char *wifiPass;
  char *userEmail;
  char *userPass;
  char **userId;
  char **deviceId;
} TCommsSetup;

void comms_task(void *pvParameters);
void comms_init(const TCommsSetup *const commsSetup);

#endif