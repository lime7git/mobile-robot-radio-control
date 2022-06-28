#ifndef __ROBOT_H__
#define __ROBOT_H__

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "motor.h"

typedef struct {
	
	float front_velocity;
	float direction_velocity;
	
	sRadioFrame *radio_frame;
	
	sMotor	*motor_left;
	sMotor 	*motor_right;
	
} sRobot;

void ROBOT_INIT(sRobot *robot, sMotor *motor_left, sMotor *motor_right, sRadioFrame *radio_frame);
void ROBOT_UPDATE_VELOCITY(sRobot *robot);

#endif
