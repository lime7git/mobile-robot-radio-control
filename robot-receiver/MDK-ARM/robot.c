#include "robot.h"

void ROBOT_INIT(sRobot *robot, sMotor *motor_left, sMotor *motor_right, sRadioFrame *radio_frame)
{
	robot->front_velocity = 0.0f;
	robot->direction_velocity = 0.0f;
	
	robot->motor_left = motor_left;
	robot->motor_right = motor_right;
	
	robot->radio_frame = radio_frame;
}
void ROBOT_UPDATE_VELOCITY(sRobot *robot)
{
	robot->front_velocity 		= ((float)robot->radio_frame->velocity_front / 34.1333f) - 60.0f;
	robot->direction_velocity = ((float)robot->radio_frame->velocity_direction / 34.1333f) - 60.0f;
	
	MOTOR_SET_VELOCITY(robot->motor_left, 	robot->front_velocity - robot->direction_velocity);
	MOTOR_SET_VELOCITY(robot->motor_right, 	robot->front_velocity + robot->direction_velocity);
}
