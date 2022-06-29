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
	float front_velocity_radio = ((float)robot->radio_frame->velocity_front / 16.384f) - 125.0f;
	float direction_velocity_radio = ((float)robot->radio_frame->velocity_direction / 16.384f) - 125.0f;
	
	if(fabs(front_velocity_radio) > 5.0f)
	{
		robot->front_velocity = front_velocity_radio;
	}
	else 
	{
		robot->front_velocity = 0.0f;
	}
	
	if(fabs(direction_velocity_radio) > 5.0f)
	{
		robot->direction_velocity = direction_velocity_radio;
	}
	else 
	{
		robot->direction_velocity = 0.0f;
	}
	
	MOTOR_SET_VELOCITY(robot->motor_left, 	robot->front_velocity - robot->direction_velocity);
	MOTOR_SET_VELOCITY(robot->motor_right, 	robot->front_velocity + robot->direction_velocity);
}
