/*
 * photodiode.h
 *
 *  Created on: Mar 8, 2019
 *      Author: swallen
 */

#ifndef PHOTODIODE_H_
#define PHOTODIODE_H_

#include "stm32f4xx_hal.h"
#include "time.h"

void initADCs(void);

uint16_t getVBatt(void);
uint16_t getPhotoDiode(void);
uint16_t getBandPass(void);

void runBeacon(void);
int16_t getBeaconAngle();
uint32_t getUsPerDeg(void);

#endif /* PHOTODIODE_H_ */
