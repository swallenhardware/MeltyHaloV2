/*
 * topLoop.h
 *
 *  Created on: Mar 8, 2019
 *      Author: swallen
 */

#ifndef TOPLOOP_H_
#define TOPLOOP_H_

#include "params.h"
#include "watchdog.h"
#include "animations.h"
#include "serial.h"
#include "adcs.h"
#include "motors.h"
#include "accelerometer.h"
#include "lidar.h"
#include "time.h"

void loop(void);

void setState(uint8_t newState);

#endif /* TOPLOOP_H_ */
