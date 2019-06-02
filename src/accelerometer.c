/*
 * accelerometer.c
 *
 *  Created on: Mar 10, 2019
 *      Author: swallen
 */

#include "accelerometer.h"

I2C_HandleTypeDef hi2c3;

uint16_t accelZ;

//the minimum accelerometer reading before we attempt to run meltybrain and POV display
#define MELTY_MIN_ACCEL 400

uint32_t lastMeasTime = 0;

uint16_t robotPeriod[2];//measured in microseconds per degree, with some memory for discrete integration

//this is the times we measured the accelerometer at. We keep some history for extrapolation
unsigned long accelMeasTime[2];

#define ACCEL_MEASUREMENT_PERIOD 10//in ms

//this angle (degrees) is calculated only using the accelerometer. We keep it separate to keep our discrete integration algorithms operating smoothly
//the beacon sets our heading to 0, which would mess up the discrete integration if allowed to affect this variable directly
//instead we utilize a trim variable. the beacon controls trim
int16_t accelAngle = 0;

//variables for hybrid mode
uint16_t highestBeaconReading = 0;
int16_t highestBeaconAngle = 0;

int16_t accelTrim = 0;
uint32_t lastTrimTime = 0;

uint16_t angleAtLastMeasurement;

//for sampling the beacon for debugging the filtering algorithm
#define BEACON_SAMPLE_COUNT 10000
uint16_t beaconSampleIndex = 0;
uint16_t beaconSamples[BEACON_SAMPLE_COUNT];

#define BEACON_DEBUG_ACQUIRE 0
#define BEACON_DEBUG_SEND 1
uint8_t beacon_debug_mode = BEACON_DEBUG_ACQUIRE;

void initAccelerometer() {
    __HAL_RCC_I2C3_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = {0};
    /**I2C3 GPIO Configuration
    PC9     ------> I2C3_SDA
    PA8     ------> I2C3_SCL
    */

	hi2c3.Instance = I2C3;
	hi2c3.Init.ClockSpeed = 400000;
	hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c3.Init.OwnAddress1 = 0;
	hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c3.Init.OwnAddress2 = 0;
	hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    HAL_I2C_Init(&hi2c3);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	I2C_ClearBusyFlagErratum();

    //now initialize the device registers
    writeI2CReg8Blocking(&hi2c3, I2C_ADDRESS, CTRL_REG1, 0x2E, 1000);

	writeI2CReg8Blocking(&hi2c3, I2C_ADDRESS, CTRL_REG4, 0x30, 1000);
}

void runAccelerometer() {
	//make sure we aren't reading too quickly, the accelerometer only runs at 100Hz
	if(HAL_GetTick() - lastMeasTime > ACCEL_MEASUREMENT_PERIOD) {
		lastMeasTime = HAL_GetTick();

		//TODO: this is a blocking operation. We could gain some clock cycles back by making this interrupt driven
		uint8_t i2c_stat = readI2CReg16Blocking(&hi2c3, I2C_ADDRESS, OUT_Z_L, &accelZ, 10);

	    //shift all of the old values down
	    for(int i=1; i>0; i--) {
	      accelMeasTime[i] = accelMeasTime[i-1];
	    }
	    //put in the new value
	    accelMeasTime[0] = getMicroseconds();

		if(i2c_stat != 0) {//If we get here, the accelerometer suddenly doesn't work
			//try clearing the busy erratum and reading again next loop
			I2C_ClearBusyFlagErratum();
			lastMeasTime -= ACCEL_MEASUREMENT_PERIOD;
		} else {
			//accellZ has successfully updated with the new accelerometer raw value

		    //give up if the bot is moving too slowly
		    if(accelZ < MELTY_MIN_ACCEL) return;

		    //this equation has been carefully calibrated for this bot. See here for explanation:
		    //https://www.swallenhardware.io/battlebots/2018/8/12/halo-pt-9-accelerometer-calibration
		    robotPeriod[1] = robotPeriod[0];
		    robotPeriod[0] = (uint32_t) (727 / sqrt((double) (accelZ-225)/522));

		    //find the new angle
		    //TRIANGULAR INTEGRATION
		    uint32_t deltaT = accelMeasTime[0] - accelMeasTime[1];
		    angleAtLastMeasurement = (angleAtLastMeasurement + (deltaT/robotPeriod[0] + deltaT/robotPeriod[1])/2) % 360;

		    accelAngle = angleAtLastMeasurement;
		}
	} else {//if it isn't time to check the accelerometer, predict our current heading
		//predict the current velocity by extrapolating old data
		uint32_t newTime = getMicroseconds();
		uint32_t periodPredicted = robotPeriod[1] + (newTime - accelMeasTime[1]) * (robotPeriod[0] - robotPeriod[1]) / (accelMeasTime[0] - accelMeasTime[1]);

		//predict the current robot heading by triangular integration up to the extrapolated point
		uint32_t deltaT = newTime - accelMeasTime[0];
		accelAngle = (angleAtLastMeasurement + (deltaT/periodPredicted + deltaT/robotPeriod[0])/2) % 360;
	}
}

//this is the angle as kept track solely by the accelerometer integration algorithm. THIS VALUE DRIFTS!
int16_t getAccelAngle() {
	return (int16_t) accelAngle;
}

#define BANDPASS_SAMPLES 20
uint16_t bandPassReadings[BANDPASS_SAMPLES];

void runHybridSensing() {
	runAccelerometer();

	if(upToSpeed()) {
		uint16_t reading = getBandPass();
		if(reading < 1500) {
			for(int i=BANDPASS_SAMPLES-2; i>=0; i--) {
				bandPassReadings[i+1] = bandPassReadings[i];
			}
			bandPassReadings[0] = reading;

			/*/code for dumping beacon measurements over serial
			if(beacon_debug_mode == BEACON_DEBUG_ACQUIRE) {
				if(beaconSampleIndex < BEACON_SAMPLE_COUNT) {
					beaconSamples[beaconSampleIndex++] = bandPassReadings[0];
				} else {
					beacon_debug_mode = BEACON_DEBUG_SEND;
					beaconSampleIndex = 0;
				}
			} else {
				sendBulkU16(beaconSamples, BEACON_SAMPLE_COUNT, &beaconSampleIndex);
				if(beaconSampleIndex >= BEACON_SAMPLE_COUNT) {
					beaconSampleIndex = 0;
					beacon_debug_mode = BEACON_DEBUG_ACQUIRE;
				}
			}//*/

			uint32_t sum = 0;
			for(int i=0; i<BANDPASS_SAMPLES; i++) {
				sum += (uint32_t) bandPassReadings[i];
			}

			sum /= BANDPASS_SAMPLES;

			if(sum > highestBeaconReading && sum > 100) {
				highestBeaconReading = sum;
				highestBeaconAngle = getAccelAngle();
			}
		}

		//periodically trim the accelerometer and reset the beacon reading
		if(getMicroseconds() - lastTrimTime > 100000) {
			lastTrimTime = getMicroseconds();
			accelTrim = highestBeaconAngle;
			highestBeaconReading = 0;
		}
	}
}

int16_t getHybridAngle() {
	int16_t angle = getAccelAngle() - accelTrim;
	if(angle < 0) angle += 360;
	angle = (angle + 180) % 360;
	return angle;
}

_Bool upToSpeed() {
	return accelZ > MELTY_MIN_ACCEL;
}

uint8_t writeI2CReg8Blocking(I2C_HandleTypeDef *i2c, uint8_t addr, uint8_t subaddr, uint8_t data, uint32_t timeout) {
	uint32_t startTime = HAL_GetTick();
	//make sure the stop bit is cleared
	while(i2c->Instance->CR1 & 0x0200) {
		if(startTime + timeout < HAL_GetTick()) return 1;
	}

	//send the start bit
	i2c->Instance->CR1 |= 0x0100;
	//wait for the start bit to complete
	while(!(i2c->Instance->SR1 & 0x0001)) {
		if(startTime + timeout < HAL_GetTick()) return 2;
	}

	//load the slave address into the peripheral
	i2c->Instance->DR = addr;
	//wait for the address to be sent
	while(!(i2c->Instance->SR1 & 0x0002)) {
		if(startTime + timeout < HAL_GetTick()) return 3;
	}
	//I2C peripheral needs this status register to be read to continue, but we don't really care about anything in it
	i2c->Instance->SR2;

	//load the subaddress byte into the peripheral
	i2c->Instance->DR = subaddr;
	//wait for the byte to be sent
	while(!(i2c->Instance->SR1 & 0x0080)) {
		if(startTime + timeout < HAL_GetTick()) return 4;
	}

	//load the data byte into the peripheral
	i2c->Instance->DR = data;
	//wait for the byte to be sent
	while(!(i2c->Instance->SR1 & 0x0080)) {
		if(startTime + timeout < HAL_GetTick()) return 5;
	}

	//send the stop bit
	i2c->Instance->CR1 |= 0x0200;

	return 0;
}

uint8_t readI2CReg16Blocking(I2C_HandleTypeDef *i2c,
		uint8_t addr,
		uint8_t subaddr,
		uint16_t *data,
		uint32_t timeout) {
	uint32_t startTime = HAL_GetTick();
	//make sure the stop bit is cleared
	while(i2c->Instance->CR1 & 0x0200) {
		if(startTime + timeout < HAL_GetTick()) return 1;
	}

	//send the start bit
	i2c->Instance->CR1 |= 0x0100;
	//wait for the start bit to complete
	while(!(i2c->Instance->SR1 & 0x0001)) {
		if(startTime + timeout < HAL_GetTick()) return 2;
	}

	//load the slave address into the peripheral
	i2c->Instance->DR = addr;
	//wait for the address to be sent
	while(!(i2c->Instance->SR1 & 0x0002)) {
		if(startTime + timeout < HAL_GetTick()) return 3;
	}
	//I2C peripheral needs this status register to be read to continue, but we don't really care about anything in it
	if(i2c->Instance->SR2){};

	//load the subaddress byte into the peripheral. We assert the MSB to indicate multi-byte transfer
	i2c->Instance->DR = subaddr | 0x80;
	//wait for the byte to be sent
	while(!(i2c->Instance->SR1 & 0x0080)) {
		if(startTime + timeout < HAL_GetTick()) return 4;
	}

	//send a repeated start bit
	i2c->Instance->CR1 |= 0x0100;
	//wait for the start bit to complete
	while(!(i2c->Instance->SR1 & 0x0001)) {
		if(startTime + timeout < HAL_GetTick()) return 5;
	}

	//load the slave address into the peripheral. Assert LSB to indicate read operation
	i2c->Instance->DR = addr | 0x01;
	//wait for the address to be sent
	while(!(i2c->Instance->SR1 & 0x0002)) {
		if(startTime + timeout < HAL_GetTick()) return 6;
	}

	//procedure for 2-byte reads from reference manual, RM0390 Pg 770
	//optimized i2c example:
	//https://www.st.com/content/ccc/resource/technical/document/application_note/5d/ae/a3/6f/08/69/4e/9b/CD00209826.pdf/files/CD00209826.pdf/jcr:content/translations/en.CD00209826.pdf
	//"STM32F10xxx I2C optimized examples"

	i2c->Instance->CR1 |= 0x0800;//Set POS
	__disable_irq();//disable interrupts
	i2c->Instance->SR2;//clear ADDR by reading SR2
	i2c->Instance->CR1 &= ~(0x0400);//clear ACK
	__enable_irq();//enable interrupts

	//wait for the bytes to come in (BTF==1)
	while(!(i2c->Instance->SR1 & 0x0004)) {
		if(startTime + timeout < HAL_GetTick()) return 7;
	}

	__disable_irq();
	i2c->Instance->CR1 |= 0x0200;//set STOP
	uint16_t dataL = i2c->Instance->DR;//read the lower data byte
	__enable_irq();
	uint16_t dataH = i2c->Instance->DR;//read upper data byte

	while(!(i2c->Instance->CR1 & 0x0200)) {//wait until STOP is cleared by hardware
		if(startTime + timeout < HAL_GetTick()) return 8;
	}

	i2c->Instance->CR1 &= ~(0x0800);//clear POS
	i2c->Instance->CR1 |= 0x0400;//Set ACK

	*data = (dataH << 8) | dataL;

	return 0;
}

//shamelessly stolen from https://electronics.stackexchange.com/questions/267972/i2c-busy-flag-strange-behaviour
//to fix a permanent busy flag in the i2c peripheral
void I2C_ClearBusyFlagErratum()
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // 1. Clear PE bit.
  hi2c3.Instance->CR1 &= ~(0x0001);

  //HAL_I2C_DeInit(&hi2c3);

  //  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
  GPIO_InitStructure.Mode         = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStructure.Alternate    = GPIO_AF4_I2C3;
  GPIO_InitStructure.Pull         = GPIO_PULLUP;
  GPIO_InitStructure.Speed        = GPIO_SPEED_FREQ_HIGH;

  GPIO_InitStructure.Pin          = GPIO_PIN_9;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);

  GPIO_InitStructure.Pin          = GPIO_PIN_8;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

  // 3. Check SCL and SDA High level in GPIOx_IDR.
  while (GPIO_PIN_SET != HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9))
  {
    asm("nop");
  }

  while (GPIO_PIN_SET != HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8))
  {
    asm("nop");
  }

  // 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

  //  5. Check SDA Low level in GPIOx_IDR.
  while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9))
  {
    asm("nop");
  }

  // 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  //  7. Check SCL Low level in GPIOx_IDR.
  while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8))
  {
    asm("nop");
  }

  // 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

  // 9. Check SCL High level in GPIOx_IDR.
  while (GPIO_PIN_SET != HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8))
  {
    asm("nop");
  }

  // 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);

  // 11. Check SDA High level in GPIOx_IDR.
  while (GPIO_PIN_SET != HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9))
  {
    asm("nop");
  }

  // 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
  GPIO_InitStructure.Mode         = GPIO_MODE_AF_OD;
  GPIO_InitStructure.Alternate    = GPIO_AF4_I2C3;

  GPIO_InitStructure.Pin          = GPIO_PIN_8;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.Pin          = GPIO_PIN_9;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

  // 13. Set SWRST bit in I2Cx_CR1 register.
  hi2c3.Instance->CR1 |= 0x8000;

  asm("nop");

  // 14. Clear SWRST bit in I2Cx_CR1 register.
  hi2c3.Instance->CR1 &= ~0x8000;

  asm("nop");

  // 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register
  hi2c3.Instance->CR1 |= 0x0001;

  // Call initialization function.
  HAL_I2C_Init(&hi2c3);
}

