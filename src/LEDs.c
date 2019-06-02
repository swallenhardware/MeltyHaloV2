/*
 * LEDs.c
 *
 *  Created on: Feb 23, 2019
 *      Author: swallen
 */

#include "LEDs.h"

static SPI_HandleTypeDef spi = {.Instance = SPI3 };

#define SPI_FRAME_SIZE 33
uint8_t ledTransmitBuffer[SPI_FRAME_SIZE];

void LED_init() {

	//set up the transmit buffer
	for(int i=0;i<SPI_FRAME_SIZE-1;i++) ledTransmitBuffer[i] = 0;//first four bytes are start frame, clear the rest of the frames too
	for(int i=0;i<7;i++) ledTransmitBuffer[i*4+4] = 0xFF;//set the three required bits for each led frame, plus the global brightness
	for(int i=7*4+4;i<SPI_FRAME_SIZE;i++) ledTransmitBuffer[i] = 0xFF;//set the end frames

}

void updateLEDs(Frame_P f) {
	//map the frame data into the buffer
	for(int i=0;i<7;i++) {
		ledTransmitBuffer[i*4+5] = f->led[i].blue;
		ledTransmitBuffer[i*4+6] = f->led[i].green;
		ledTransmitBuffer[i*4+7] = f->led[i].red;
	}
	//send out the buffer
	HAL_SPI_Transmit(&spi, &ledTransmitBuffer[0], SPI_FRAME_SIZE, HAL_MAX_DELAY);
}
