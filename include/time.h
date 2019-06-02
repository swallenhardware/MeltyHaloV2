/*
 * time.h
 *
 *  Created on: Apr 16, 2019
 *      Author: swallen
 */

#ifndef TIME_H_
#define TIME_H_

#include "stm32f4xx_hal.h"
#include "params.h"

void initTimers(void);

uint32_t getMicroseconds(void);
uint32_t getMilliseconds(void);

void resetTimers(void);

#endif /* TIME_H_ */
