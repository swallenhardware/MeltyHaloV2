/*
 * lidar.h
 *
 *  Created on: Mar 22, 2019
 *      Author: swallen
 */

#ifndef LIDAR_H_
#define LIDAR_H_

#include "stm32f4xx_hal.h"

#define TDC7200_CONFIG1 0x00
#define TDC7200_CONFIG2 0x01
#define TDC7200_INT_STATUS 0x02
#define TDC7200_INT_MASK 0x03
#define TDC7200_COURSE_CNTR_OVF_H 0x04
#define TDC7200_COARSE_CNTR_OVF_L 0x05
#define TDC7200_CLOCK_CNTR_OVF_H 0x06
#define TDC7200_CLOCK_CNTR_OVF_L 0x07
#define TDC7200_CLOCK_CNTR_STOP_MASK_H 0x08
#define TDC7200_CLOCK_CNTR_STOP_MASK_L 0x09
#define TDC7200_TIME1 0x10
#define TDC7200_CLOCK_COUNT1 0x11
#define TDC7200_TIME2 0x12
#define TDC7200_CLOCK_COUNT2 0x13
#define TDC7200_TIME3 0x14
#define TDC7200_CLOCK_COUNT3 0x15
#define TDC7200_TIME4 0x16
#define TDC7200_CLOCK_COUNT4 0x17
#define TDC7200_TIME5 0x18
#define TDC7200_CLOCK_COUNT5 0x19
#define TDC7200_TIME6 0x1A
#define TDC7200_CALIBRATION1 0x1B
#define TDC7200_CALIBRATION2 0x1C

#define TDC7200_SPI_WRITE 0x40
#define TDC7200_SPI_READ 0x00
#define TDC7200_AUTOINC 0x80

void initLidar(void);

uint8_t readSPIReg8Blocking(SPI_HandleTypeDef spi, uint8_t addr);

void turnSourceOn(void);
void turnSourceOff(void);

#endif /* LIDAR_H_ */
