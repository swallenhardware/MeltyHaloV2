/*
 * initializeAnimations.h
 *
 *  Created on: Feb 24, 2019
 *      Author: swallen
 */

#ifndef INITIALIZEANIMATIONS_H_
#define INITIALIZEANIMATIONS_H_

#include "stm32f4xx_hal.h"
#include "LEDs.h"
#include "time.h"
#include "params.h"

Frame_P createBlankFrame(uint16_t duration);
Frame_P createMonocolorFrame(uint8_t leds, uint8_t red, uint8_t green, uint8_t blue, uint16_t duration);
Frame_P addBlankFrame(Frame_P tail, uint16_t duration);
Frame_P addMonocolorFrame(Frame_P tail, uint8_t leds, uint8_t red, uint8_t green, uint8_t blue, uint16_t duration);
Frame_P addForegroundBackgroundFrame(Frame_P tail, uint8_t leds, uint8_t fr, uint8_t fg, uint8_t fb, uint16_t duration,
		uint8_t br, uint8_t bg, uint8_t bb);

void initAnimations(void);

void setAnimation(uint8_t state);

void runTimeAnimation(void);
void runPOVAnimation(int16_t heading);

void initSafeAnimation(void);
void initIdleAnimation(void);
void initPreJukeAnimation(void);
void initJukeAnimation(void);
void initDriveAnimation(void);
void initSpinAnimation(void);


#endif /* INITIALIZEANIMATIONS_H_ */
