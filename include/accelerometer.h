/*
 * accelerometer.h
 *
 *  Created on: Mar 10, 2019
 *      Author: swallen
 */

#ifndef ACCELEROMETER_H_
#define ACCELEROMETER_H_

#include "stm32f4xx_hal.h"
#include "math.h"
#include "time.h"
#include "serial.h"
#include "adcs.h"

#define I2C_ADDRESS (0x18<<1)

#define WHO_AM_I 0x0F
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
#define HP_FILTER_RESET 0x25
#define REFERENCE 0x26
#define STATUS_REG 0x27
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D
#define INT1_CFG 0x30
#define INT1_SRC 0x31
#define INT1_THS 0x32
#define INT1_DURATION 0x33
#define INT2_CFG 0x34
#define INT2_SRC 0x35
#define INT2_THS 0x36
#define INT2_DURATION 0x37

void initAccelerometer(void);

void runAccelerometer(void);
int16_t getAccelAngle(void);

void runHybridSensing(void);
int16_t getHybridAngle(void);

_Bool upToSpeed(void);

uint8_t writeI2CReg8Blocking(I2C_HandleTypeDef *i2c, uint8_t addr, uint8_t subaddr, uint8_t data, uint32_t timeout);
uint8_t readI2CReg16Blocking(I2C_HandleTypeDef *i2c, uint8_t addr, uint8_t subaddr, uint16_t *data, uint32_t timeout);

void I2C_ClearBusyFlagErratum(void);

#endif /* ACCELEROMETER_H_ */
