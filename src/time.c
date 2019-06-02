/*
 * time.c
 *
 *  Created on: Apr 16, 2019
 *      Author: swallen
 */

#include "time.h"

TIM_TypeDef * microTimInst = TIM5;
TIM_TypeDef * milliTimInst = TIM2;

void initTimers() {
	//SETUP THE MICROSECONDS TIMER
	__HAL_RCC_TIM5_CLK_ENABLE();
	//config timer 5 to be our microseconds count
	microTimInst->CR1 = 0;//make sure the timer is disabled

	microTimInst->PSC = (MAIN_CLOCK/2)-1;//gives 1MHz, or 1us period
	//tim2Inst->ARR = 0xFFFF;
	microTimInst->CNT = 0;

	//SETUP THE MILLISECONDS TIMER
	__HAL_RCC_TIM5_CLK_ENABLE();
	//config timer 5 to be our microseconds count
	milliTimInst->CR1 = 0;//make sure the timer is disabled

	milliTimInst->PSC = (MAIN_CLOCK*1000/2)-1;//gives 1KHz, or 1us period
	//tim2Inst->ARR = 0xFFFF;
	milliTimInst->CNT = 0;

	//ENABLE THE TIMERS
	milliTimInst->CR1 = 1;
	microTimInst->CR1 = 1;

	//manually generate an update event so the prescaler loads into the shadow registers
	//this took a ridiculous two hours to figure out. Just so you know
	milliTimInst->EGR |= 0x0001;
	microTimInst->EGR |= 0x0001;
	resetTimers();
}

uint32_t getMicroseconds() {
	return microTimInst->CNT;
}

uint32_t getMilliseconds() {
	return milliTimInst->CNT;
}

void resetTimers() {
	microTimInst->CNT = 0;
	milliTimInst->CNT = 0;
}

