/*
 * watchdog.c
 *
 *  Created on: May 19, 2019
 *      Author: swallen
 */

#include "watchdog.h"

IWDG_HandleTypeDef hiwdg;

void initWatchdog() {
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
	//set for 500ms timeout. LSI=32000Hz, prescaler=32, LSI*(time, s)/prescaler=500
	hiwdg.Init.Reload = 500;

	HAL_IWDG_Init(&hiwdg);
}

void feedWatchdog() {
	HAL_IWDG_Refresh(&hiwdg);
}
