/*
 * serial.h
 *
 *  Created on: Mar 8, 2019
 *      Author: swallen
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include "stm32f4xx_hal.h"
#include <string.h>
#include "time.h"
#include "math.h"

void initSerial(void);

void periodicReturnU16(uint16_t value);
void sendBulkU16(uint16_t *data, uint16_t size, uint16_t *dataSent);
void periodicReturnU32(uint32_t value);
void periodicReturnCalibration(uint32_t time, uint16_t accel);

void receiveSerial(void);

void sendSerial(char * buf, uint8_t size);

void DMA1_Stream1_IRQHandler(void);
void USART2_IRQHandler(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

_Bool isControlled(void);
_Bool isEnabled(void);
uint16_t commandedThrottle(void);

uint16_t getMeltyThrottle(void);
int16_t getMeltyAngle(void);

int16_t getThumbX(void);
int16_t getThumbY(void);

uint8_t getDirSwitch(void);
uint8_t getModeSwitch(void);
uint8_t getStickSwitch(void);

//this is used to calculate meltythrottle
uint32_t SquareRootRounded(uint32_t a_nInput);
unsigned int_pow(int n, unsigned x);

#endif /* SERIAL_H_ */
