/*
 * motors.h
 *
 *  Created on: Mar 10, 2019
 *      Author: swallen
 */

#ifndef MOTORS_H_
#define MOTORS_H_

#include "stm32f4xx_hal.h"
#include "params.h"

//#define MC_DRV8320
#define MC_DSHOT
//#define MC_ONESHOT125

#define MOTOR1 1
#define MOTOR2 2

#define MOTOR_DIR_CW 0
#define MOTOR_DIR_CCW 1
#define MOTOR_BRAKE_ON 0
#define MOTOR_BRAKE_OFF 1

#define MOTOR_MAX_SPEED 512

#define MOTOR_DEADZONE 100//the minimum speed value before the motors are engaged

void initMotors(void);

void setMotorSpeed(uint8_t motor, uint16_t speed, uint8_t dir);
void setMotorSpeed2(uint8_t motor, int16_t speed);

#ifdef MC_DSHOT
//all of these values are tuned for a base peripheral clock of 40,000,000Hz
	//#define DSHOT300
	//#define DSHOT600
	#define DSHOT1200

	#ifdef DSHOT300
		#define DSHOT_PERIOD 100//80MHz: 132
		#define DSHOT_1_TIME 75//80MHz: 104
		#define DSHOT_0_TIME 37//80MHz: 52
		#define MIN_MOTOR_TIME 400UL
	#endif

	#ifdef DSHOT600
		#define DSHOT_PERIOD 50 //80MHz: 66
		#define DSHOT_1_TIME 38 //80MHz: 52 75% of 600 baud, tuned for 1250ns
		#define DSHOT_0_TIME 19 //80MHz: 26 37% of 600 baud, tuned for 625ns
		#define MIN_MOTOR_TIME 200UL//in us
	#endif

	#ifdef DSHOT1200
		#define DSHOT_PERIOD 25 //80MHz: 33
		#define DSHOT_1_TIME 19 //80MHz: 26
		#define DSHOT_0_TIME 9 //80MHz: 13
		#define MIN_MOTOR_TIME 100UL
	#endif

//this typedef taken from src\main\drivers\pwm_output.h in the betaflight github page
typedef enum {
    DSHOT_CMD_MOTOR_STOP = 0,
    DSHOT_CMD_BEACON1,
    DSHOT_CMD_BEACON2,
    DSHOT_CMD_BEACON3,
    DSHOT_CMD_BEACON4,
    DSHOT_CMD_BEACON5,
    DSHOT_CMD_ESC_INFO, // V2 includes settings
    DSHOT_CMD_SPIN_DIRECTION_1,
    DSHOT_CMD_SPIN_DIRECTION_2,
    DSHOT_CMD_3D_MODE_OFF,
    DSHOT_CMD_3D_MODE_ON,
    DSHOT_CMD_SETTINGS_REQUEST, // Currently not implemented
    DSHOT_CMD_SAVE_SETTINGS,
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
    DSHOT_CMD_LED0_ON, // BLHeli32 only
    DSHOT_CMD_LED1_ON, // BLHeli32 only
    DSHOT_CMD_LED2_ON, // BLHeli32 only
    DSHOT_CMD_LED3_ON, // BLHeli32 only
    DSHOT_CMD_LED0_OFF, // BLHeli32 only
    DSHOT_CMD_LED1_OFF, // BLHeli32 only
    DSHOT_CMD_LED2_OFF, // BLHeli32 only
    DSHOT_CMD_LED3_OFF, // BLHeli32 only
    DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF = 30, // KISS audio Stream mode on/Off
    DSHOT_CMD_SILENT_MODE_ON_OFF = 31, // KISS silent Mode on/Off
    DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE = 32,
    DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY = 33,
    DSHOT_CMD_MAX = 47
} dshotCommands_e;

void issueDshotCommand(uint8_t motor, uint16_t command);
void armMotors(void);
#endif

#ifdef MC_ONESHOT125
#define ONESHOT_PERIOD 10000
void disableMotors(void);
void enableMotors(void);
#endif

#ifdef MC_DRV8320
void enableMotors(void);
void disableMotors(void);
void setMotorBrake(uint8_t motor, uint8_t setting);
#endif


#endif /* MOTORS_H_ */
