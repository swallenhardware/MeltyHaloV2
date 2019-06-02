/*
 * photodiode.c
 *
 *  Created on: Mar 8, 2019
 *      Author: swallen
 */

#include "adcs.h"

ADC_HandleTypeDef adc1Handle = {.Instance = ADC1};
ADC_HandleTypeDef adc2Handle = {.Instance = ADC2};
ADC_HandleTypeDef adc3Handle = {.Instance = ADC3};

//this just initializes things not initialized by the BubeMX code
void initADCs() {
	HAL_ADC_Start(&adc1Handle);
	HAL_ADC_Start(&adc2Handle);
	HAL_ADC_Start(&adc3Handle);
}

uint16_t getVBatt() {
	return (uint16_t) ADC2->DR;
}

uint16_t getPhotoDiode() {
	return (uint16_t) ADC1->DR;
}

uint16_t getBandPass() {
	return (uint16_t) ADC3->DR;
}

#define BEACON_STATE_WAIT 0
#define BEACON_STATE_EDGE 1
#define BEACON_STATE_RECIEVED 2
uint8_t beaconState = BEACON_STATE_WAIT;

uint32_t edgeTime = 0;
uint32_t beaconLastTime = 0;
uint32_t usPerDeg = 0;

#define MAX_USPERDEG 600000UL
#define MIN_BEACON_THRESH 400

void runBeacon() {
	uint16_t beaconMeas = getBandPass();

	switch(beaconState) {
	case BEACON_STATE_WAIT :
		//check for new readings
		if(beaconMeas > MIN_BEACON_THRESH) {
			beaconState = BEACON_STATE_EDGE;
			edgeTime = getMicroseconds();
		}
		break;
	case BEACON_STATE_EDGE:
		//double check the packet
		if(beaconMeas > MIN_BEACON_THRESH) { //if the beacon is still high
			if(getMicroseconds() - edgeTime > 2000UL) {//and 0.2ms has passed
				usPerDeg = (edgeTime - beaconLastTime) / 360UL;//we report a real edge
				beaconLastTime = edgeTime;
				if(usPerDeg >= MAX_USPERDEG) usPerDeg = 0;
				beaconState = BEACON_STATE_RECIEVED;
			}
		} else {
			beaconState = BEACON_STATE_WAIT;
		}
		break;

	case BEACON_STATE_RECIEVED :
		//check for the current packet to end. Wait at least the amount of time for the bot to complete a half revolution at 1000RPM
		if(beaconMeas < MIN_BEACON_THRESH/2) {
			beaconState = BEACON_STATE_WAIT;
		}
		break;
	default :
		beaconState = BEACON_STATE_WAIT;
		break;
	}

	//reset the algorthim if it's been too long since we've seen an edge
	if(getMicroseconds() - beaconLastTime > 500000UL) usPerDeg = 0;
}

int16_t getBeaconAngle() {
	//return (HAL_GetTick()/100) % 360;
	if(usPerDeg != 0) {
		return (int16_t) ((((getMicroseconds() - beaconLastTime) / usPerDeg) + 180) % 360);
	}
	return 0;
}

uint32_t getUsPerDeg() {
	return usPerDeg;
}
