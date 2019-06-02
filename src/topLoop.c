/*
 * topLoop.c
 *
 * this contains the main state machine logic for the bot
 *
 *  Created on: Mar 8, 2019
 *      Author: swallen
 */

#include "topLoop.h"

uint8_t mainState = 0;//we set to an invalid state so the loop is forced to properly set up in safe state

uint32_t jukeStartTime = 0;

uint32_t loopTime = 0;//in us
uint32_t lastMotorSendTime = 0;

uint8_t stickSwitchDebounce = 0;

void loop() {

	//USER INITIALIZATION FUNCTIONS
	initTimers();
	initAnimations();
	setAnimation(STATE_SAFE);
	initADCs();
	initSerial();
	initWatchdog();
	initMotors();
	initAccelerometer();
	//initLidar();

	//turnSourceOn();

	//INFINITE LOOP
	while(1) {
		feedWatchdog();
		loopTime = getMicroseconds();

		receiveSerial();

		//make sure we are still connected to the controller
		if(!isControlled() && mainState != STATE_SAFE){
			setState(STATE_SAFE);
		}

		//switch debouncing functions
		if(stickSwitchDebounce && !getStickSwitch()) stickSwitchDebounce = 0;

		switch(mainState) {
		case(STATE_SAFE):
			//This state is for fault conditions, and for when we don't have communication with the controller.
			if(isControlled()) setState(STATE_IDLE);
			break;

		case(STATE_IDLE):
			//this state is when we have communications with the controller, but it's telling us to stay disabled
			if(isEnabled()) {
				if(commandedThrottle() > 10) {
					setState(STATE_SPIN);
				} else {
					setState(STATE_DRIVE);
				}
			}

			if(getStickSwitch() && !stickSwitchDebounce) {
				stickSwitchDebounce = 1;
				setState(STATE_PREJUKE);
			}

#ifdef MC_DSHOT
			//make sure we don't go *too* fast so the motor controllers don't freak out
			if(loopTime - lastMotorSendTime < MIN_MOTOR_TIME) break;
			lastMotorSendTime = loopTime;

			setMotorSpeed2(MOTOR1, 0);
			setMotorSpeed2(MOTOR2, 0);
#endif
			break;

		case(STATE_PREJUKE):
			//this state is like idle, but we are going to do a juke maneuver if we go to spin mode next
			if(isEnabled()) {
				if(commandedThrottle() > 10) {
					jukeStartTime = getMicroseconds();
					setState(STATE_JUKE);
				} else {
					setState(STATE_DRIVE);
				}
			}

			if(getStickSwitch() && !stickSwitchDebounce) {
				stickSwitchDebounce = 1;
				setState(STATE_IDLE);
			}

#ifdef MC_DSHOT
			//make sure we don't go *too* fast so the motor controllers don't freak out
			if(loopTime - lastMotorSendTime < MIN_MOTOR_TIME) break;
			lastMotorSendTime = loopTime;

			setMotorSpeed2(MOTOR1, 0);
			setMotorSpeed2(MOTOR2, 0);
#endif
			break;

		case(STATE_JUKE):
			if(getMicroseconds() - jukeStartTime > 500000) {
				setState(STATE_SPIN);
			}

#ifdef MC_DSHOT
			//make sure we don't go *too* fast so the motor controllers don't freak out
			if(loopTime - lastMotorSendTime < MIN_MOTOR_TIME) break;
			lastMotorSendTime = loopTime;

			setMotorSpeed(MOTOR1, 400, MOTOR_DIR_CW);
			setMotorSpeed(MOTOR2, 400, MOTOR_DIR_CCW);
#endif
			break;

		case(STATE_DRIVE):
			//this state drives the robot in standard arcade mode
			if(!isEnabled()) setState(STATE_IDLE);
			if(commandedThrottle() > 10) {
				setState(STATE_SPIN);
				continue;
			}


#ifdef MC_DSHOT
			//make sure we don't go *too* fast so the motor controllers don't freak out
			if(loopTime - lastMotorSendTime < MIN_MOTOR_TIME) break;
			lastMotorSendTime = loopTime;
#endif

			setMotorSpeed2(MOTOR1, -1*(getThumbX() - getThumbY()/2));
			setMotorSpeed2(MOTOR2, getThumbX() + getThumbY()/2);

			break;

		case(STATE_SPIN):
			//this state drives the robot in meltybrain mode
			if(!isEnabled()) setState(STATE_IDLE);
			if(commandedThrottle() <= 10) {
				setState(STATE_DRIVE);
				continue;
			}

			runHybridSensing();
			//runBeacon();

			//this is the master heading variable. If we want to change what sensor combinations are feeding us
			//our heading, we do that in the following line
			int16_t heading = getHybridAngle();
			//int16_t heading = getAccelAngle();
			//int16_t heading = getBeaconAngle();

			runPOVAnimation(heading);

			//MOTOR COMMAND
#ifdef MC_DSHOT
			//make sure we don't go *too* fast so the motor controllers don't freak out
			if(loopTime - lastMotorSendTime < MIN_MOTOR_TIME) break;
			lastMotorSendTime = loopTime;
#endif

			uint16_t speed = (commandedThrottle()/2)*(getDirSwitch() ? -1 : 1);

			//first check if the melty throttle is high enough for translation
			if (getMeltyThrottle() > 10) {

				//calculate the distance between the current heading and the commanded direction
				int16_t diff = 180 - abs(abs(getMeltyAngle() - heading) - 180);

				//now check if we are pointed more towards the commanded direction or the opposite
				if(diff < 90) {
				  //we are pointing towards the commanded heading, forward pulse
				  setMotorSpeed2(MOTOR1, speed-40);
				  setMotorSpeed2(MOTOR2, speed+40);
				} else {
				  //we are pointing opposite the commanded heading, reverse pulse
				  setMotorSpeed2(MOTOR1, speed+40);
				  setMotorSpeed2(MOTOR2, speed-40);
				}
			} else {
				//if we aren't translating, just run the motors at the throttle speed
				setMotorSpeed2(MOTOR1, speed);
				setMotorSpeed2(MOTOR2, speed);
			}

			break;

		default:
			setState(STATE_SAFE);
			break;
		}

		if(mainState != STATE_SPIN) {
			runTimeAnimation();
		} else {
			//if(upToSpeed()) runPOVAnimation(heading);
		}
	}
}

/* setState()
 * arguments:
 * 	newState: the new state that you wish to transition the bot into
 * returns:
 *  nothing
 *
 *  Use this to change what state the bot is in. This function handles all relevant re-initialization
 *  for every state.
 */

void setState(uint8_t newState) {
	switch(newState) {
	default:


	case(STATE_SAFE):
		mainState = newState;
#if defined(MC_DRV8320) || defined(MC_ONESHOT125)
		disableMotors();
#endif
		break;


	case(STATE_IDLE):
		mainState = newState;
#ifdef MC_ONESHOT125
		setMotorSpeed2(MOTOR1, 0);
		setMotorSpeed2(MOTOR2, 0);
#endif

#ifdef MC_DRV8320
		setMotorBrake(MOTOR1, MOTOR_BRAKE_ON);
		setMotorBrake(MOTOR2, MOTOR_BRAKE_ON);
#endif
		break;


	case(STATE_PREJUKE):
		mainState = newState;
		break;


	case(STATE_JUKE):
		mainState = newState;
#ifdef MC_DRV8320
		setMotorBrake(MOTOR1, MOTOR_BRAKE_OFF);
		setMotorBrake(MOTOR2, MOTOR_BRAKE_OFF);
#endif

#if defined(MC_DRV8320) || defined(MC_ONESHOT125)
		enableMotors();
#endif
		break;


	case(STATE_DRIVE):
		mainState = newState;
#ifdef MC_DRV8320
		setMotorBrake(MOTOR1, MOTOR_BRAKE_OFF);
		setMotorBrake(MOTOR2, MOTOR_BRAKE_OFF);
#endif

#if defined(MC_DRV8320) || defined(MC_ONESHOT125)
		enableMotors();
#endif
		break;


	case(STATE_SPIN):
		mainState = newState;
#ifdef MC_DRV8320
		setMotorBrake(MOTOR1, MOTOR_BRAKE_OFF);
		setMotorBrake(MOTOR2, MOTOR_BRAKE_OFF);
#endif

#if defined(MC_DRV8320) || defined(MC_ONESHOT125)
		enableMotors();
#endif
		break;


	}

	//assuming the state change was successful, change the animation pattern
	if(newState == mainState) setAnimation(mainState);
}
