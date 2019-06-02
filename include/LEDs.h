/*
 * LEDs.h
 *
 *  Created on: Feb 24, 2019
 *      Author: swallen
 */

#ifndef LEDS_H_
#define LEDS_H_

#include <stdlib.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

typedef struct __Frame {
	uint16_t duration;//in half-degrees or milliseconds
	struct {
		uint8_t red;
		uint8_t green;
		uint8_t blue;
	} led[7];
	struct __Frame *next;
} *Frame_P, Frame;

extern void LED_init(void);
extern void updateLEDs(Frame_P f);

#endif /* LEDS_H_ */
