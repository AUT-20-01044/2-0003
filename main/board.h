/*
 * board.h
 *
 *  Created on: 19 May 2020
 *      Author: rob_c
 */

#ifndef MAIN_BOARD_H_
#define MAIN_BOARD_H_

// SPI pins (from ESP perspective)
#define GPIO_SPI_MOSI 23
#define GPIO_SPI_MISO 19
#define GPIO_SPI_SCK 18
#define GPIO_CS_1 5 // Stepper Driver 1 CS
#define GPIO_CS_2 4 // Stepper Driver 2 CS
#define GPIO_CS_3 2 // SD Card CS

// Stepper Driver 1 Pins
#define GPIO_DRV_STP_1 25  // Step Control
#define GPIO_DRV_DIR_1 33  // Direction Control
#define GPIO_DRV_DIAG0_1 7 // Diagnostic pin
#define GPIO_DRV_DIAG1_1 6 // Diagnostic pin

// Stepper Driver 2 Pins
#define GPIO_DRV_STP_2 17   // Step Control
#define GPIO_DRV_DIR_2 16   // Direction Control
#define GPIO_DRV_DIAG0_2 27 // Diagnostic pin
#define GPIO_DRV_DIAG1_2 26 // Diagnostic pin

// SD Card Pins
#define GPIO_SD_CD 4 // Card detect pin for SD Card

// Status LED PINS (High = On)
#define GPIO_LED_ERR 21
#define GPIO_LED_STS 22

// Buttons
#define GPIO_BTN_RST 39 // Pull low when pressed

#endif /* MAIN_BOARD_H_ */
