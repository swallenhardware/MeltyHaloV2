/*
 * serial.c
 *
 *  Created on: Mar 8, 2019
 *      Author: swallen
 */

#include "serial.h"


UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

#define SERIAL_RX_SIZE 9
uint8_t serialRxBuf[SERIAL_RX_SIZE];
uint8_t packet[9];

//how long we will wait before we go into safe mode
#define RECEIVE_TIMEOUT 1000
uint32_t lastReceived = 0;

#define SERIAL_WAIT 0x01
#define SERIAL_PACKETSTART 0x02
uint8_t serialState = SERIAL_WAIT;
uint8_t bytesRead = 0;

_Bool packetReceived;

uint8_t status = 0;
int16_t thumbX = 0;
int16_t thumbY = 0;
uint16_t throt = 0;
uint8_t enable = 0;

uint16_t meltyThrottle;
int16_t meltyAngle;

void initSerial() {

	huart3.Instance = USART3;
	huart3.Init.BaudRate = 57600;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&huart3);

	HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART3_IRQn);
}

uint32_t lastSend = 0;
char sendBuf[11];
void periodicReturnU16(uint16_t value) {
	if(HAL_GetTick() - lastSend > 100) {
		lastSend = HAL_GetTick();
		//sprintf, snprintf kept giving me hardfaults, so I said fuck it and wrote my own int to string converter
		sendBuf[0] = (value / 1000) + 48;
		sendBuf[1] = ((value % 1000) / 100) + 48;
		sendBuf[2] = ((value % 100) / 10) + 48;
		sendBuf[3] = (value % 10) + 48;
		sendBuf[4] = '\n';

		//turn higher power 0's into spaces
		for(uint8_t i=0; i<3; i++) {
			if(sendBuf[i] == 48) {
				sendBuf[i] = 32;
			} else {
				break;
			}
		}
		sendSerial(sendBuf, 5);
	}
}

void sendBulkU16(uint16_t *data, uint16_t size, uint16_t *dataSent) {
	uint16_t value = data[*dataSent];
	if(HAL_GetTick() - lastSend > 10 && (*dataSent) < size) {
		lastSend = HAL_GetTick();
		//a header to help align the bytes
		sendBuf[0] = 0x7E;

		//the time
		sendBuf[1] = (uint8_t) ((value >> 8) & 0x00FF);
		sendBuf[2] = (uint8_t) (value & 0x00FF);

		sendSerial(sendBuf, 5);
		(*dataSent)++;
	}
}

void periodicReturnU32(uint32_t value) {
	if(HAL_GetTick() - lastSend > 100) {
		lastSend = HAL_GetTick();
		//sprintf, snprintf kept giving me hardfaults, so I said fuck it and wrote my own int to string converter
		sendBuf[0] = (value / 1000000000U) + 48;
		for(uint32_t i=1; i<10; i++) {
			sendBuf[10-i] = ((value % int_pow(10,i)) / int_pow(10,i-1)) + 48;
		}
		sendBuf[10] = '\n';

		//turn higher power 0's into spaces
		for(uint8_t i=0; i<9; i++) {
			if(sendBuf[i] == 48) {
				sendBuf[i] = 32;
			} else {
				break;
			}
		}
		sendSerial(sendBuf, 11);
	}
}

//this function is designed to periodically send a packet for calibrating the accelerometer
void periodicReturnCalibration(uint32_t time, uint16_t accel){
	if(HAL_GetTick() - lastSend > 100) {
		lastSend = HAL_GetTick();
		//a header to help align the bytes
		sendBuf[0] = 0x7E;

		//the time
		sendBuf[1] = (uint8_t) ((time >> 24) & 0x000000FF);
		sendBuf[2] = (uint8_t) ((time >> 16) & 0x000000FF);
		sendBuf[3] = (uint8_t) ((time >> 8) & 0x000000FF);
		sendBuf[4] = (uint8_t) (time & 0x000000FF);

		//the acceleration
		sendBuf[5] = (uint8_t) ((accel >> 8) & 0x00FF);
		sendBuf[6] = (uint8_t) (accel & 0x00FF);

		sendSerial(sendBuf, 7);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	UNUSED(huart);

	//trace_printf("TC");
	for(int i=0; i<9; i++) {
		if(serialState == SERIAL_WAIT) {
			if(serialRxBuf[i] == 0x7E) {
			serialState = SERIAL_PACKETSTART;
			bytesRead = 0;
			}

			continue;
	    }
	    packet[bytesRead] = serialRxBuf[i];
	    bytesRead++;

	    if(bytesRead >= 8) {
	    	lastReceived = HAL_GetTick();
	    	packetReceived = 1;

			serialState = SERIAL_WAIT;
	    }
	}
}

void sendSerial(char * buf, uint8_t size) {
	HAL_UART_Transmit(&huart3, buf, size, 100);
}

void receiveSerial() {
	//this sets us up to receive serial data. If we're already set up and waiting, this does nothing
	HAL_UART_Receive_IT(&huart3, serialRxBuf, SERIAL_RX_SIZE);

	//if we received a new packet, crunch the numbers
	if(packetReceived) {
		packetReceived = 0;

    	//unpack the received data into the correct variables
		status = packet[0];
		thumbX = 512 - ((((int16_t) packet[1]) << 8) | ((int16_t) packet[2]));
		thumbY = ((((int16_t) packet[3]) << 8) | ((int16_t) packet[4])) - 512;
		throt = (((uint16_t) packet[5]) << 8) | ((uint16_t) packet[6]);
		enable = packet[7];

		uint32_t s32X = (int32_t) thumbX;
		uint32_t s32Y = (int32_t) thumbY;

		//calculate meltybrain commands
		meltyThrottle = (uint16_t) SquareRootRounded(s32X*s32X + s32Y*s32Y)/2;

		meltyAngle = (int16_t) (atan2((double) thumbY, (double) thumbX)*180.0/M_PI);
		if(meltyAngle < 0) meltyAngle += 360;
	}
}

void DMA1_Stream1_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
}

void USART3_IRQHandler(void) {
  HAL_UART_IRQHandler(&huart3);
}

//helper functions
_Bool isControlled() {
	return (((HAL_GetTick() - lastReceived) < RECEIVE_TIMEOUT) && (lastReceived != 0));
}

_Bool isEnabled() {
	return enable == 0xAA;
}

uint16_t commandedThrottle() {
	return throt;
}

uint16_t getMeltyThrottle() {
	return meltyThrottle;
}

int16_t getMeltyAngle() {
	return meltyAngle;
}

int16_t getThumbX() {
	return thumbX;
}

int16_t getThumbY() {
	return thumbY;
}

uint8_t getDirSwitch() {
	return (status & 0x01) > 0;
}

uint8_t getModeSwitch() {
	return (status & 0x02) > 0;
}

uint8_t getStickSwitch() {
	return (status & 0x04) > 0;
}

//an integer square root function shamelessly stolen from
//https://stackoverflow.com/questions/1100090/looking-for-an-efficient-integer-square-root-algorithm-for-arm-thumb2
uint32_t SquareRootRounded(uint32_t a_nInput)
{
    uint32_t op  = a_nInput;
    uint32_t res = 0;
    uint32_t one = 1uL << 30; // The second-to-top bit is set: use 1u << 14 for uint16_t type; use 1uL<<30 for uint32_t type


    // "one" starts at the highest power of four <= than the argument.
    while (one > op)
    {
        one >>= 2;
    }

    while (one != 0)
    {
        if (op >= res + one)
        {
            op = op - (res + one);
            res = res +  2 * one;
        }
        res >>= 1;
        one >>= 2;
    }

    /* Do arithmetic rounding to nearest integer */
    if (op > res)
    {
        res++;
    }

    return res;
}

//an integer exponent function shamelessly stolen from
//https://stackoverflow.com/questions/15394216/integer-power-in-c
unsigned int_pow(int n, unsigned x)
{
    unsigned i, a = n;
    switch (x)
    {
      case 0:
        return 1;
      case 1:
        return n;
      default:
        for (i = 1; i < x; ++i) {
             a *= n;
        }
        break;
    }
    return a;
}
