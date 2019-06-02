/*
 * watchdog.h
 *
 *  Created on: May 19, 2019
 *      Author: swallen
 */

#ifndef WATCHDOG_H_
#define WATCHDOG_H_

#include "stm32f4xx_hal.h"

void initWatchdog(void);

void feedWatchdog(void);


#endif /* WATCHDOG_H_ */
