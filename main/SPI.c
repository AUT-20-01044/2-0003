/*
 * SPI.c
 *
 *  Created on: 19 May 2020
 *      Author: rob_c
 */

#include "SPI.h"

void spi_init(void)
{
	esp_err_t ret;

	spi_bus_config_t buscfg={
	        .miso_io_num=GPIO_SPI_MISO,
	        .mosi_io_num=GPIO_SPI_MOSI,
	        .sclk_io_num=GPIO_SPI_SCK,
	        .quadwp_io_num=-1,
	        .quadhd_io_num=-1,
	        .max_transfer_sz=40 						// Max transfer Size in bytes
	    };
	//Initialize the SPI bus on SPI3_HOST
	ret=spi_bus_initialize(VSPI_HOST, &buscfg, 1); // initilised with DMA channel 1
	ESP_ERROR_CHECK(ret);
}

