/*
 * params.h
 *
 *  Created on: Feb 24, 2019
 *      Author: swallen
 */

#ifndef PARAMS_H_
#define PARAMS_H_

#define MAIN_CLOCK 60//In MHz

//defines for the main state machine
#define STATE_SAFE 1
#define STATE_IDLE 2
#define STATE_PREJUKE 3
#define STATE_JUKE 4
#define STATE_DRIVE 5
#define STATE_SPIN 6

#define SENSE_BEACON 0
//#define SENSE_ACCEL_BEACON 1
//#define SENSE_ACCEL_LIDAR 2

#endif /* PARAMS_H_ */
