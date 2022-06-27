#include "pid.h"

void PID_INIT(sPid *pid, float kp, float ki, float kd, float anti_windup_limit)
{
	pid->set_value 			= 0.0f;
	pid->current_value 	= 0.0f;
	pid->error 					= 0.0f;
	pid->previous_error = 0.0f;
	pid->total_error		= 0.0f;
	pid->out 						= 0.0f;
	
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->anti_windup_limit = anti_windup_limit;
}
void PID_CALCULATE(sPid *pid)
{
	pid->previous_error = pid->error;
	pid->error = pid->set_value - pid->current_value;
	pid->total_error += pid->error;
	
	if(pid->total_error > pid->anti_windup_limit)
	{
		pid->total_error = pid->anti_windup_limit;
	}
		else if(pid->total_error < -(pid->anti_windup_limit))
		{
			pid->total_error = -(pid->anti_windup_limit);
		}
	
	pid->out = (pid->kp * pid->error) + (pid->ki * pid->total_error * TIME_STAMP) + (pid->kd * (pid->error - pid->previous_error) / TIME_STAMP);
		
	if(pid->out > 1.0f)
	{
		pid->out = 1.0f;
	}
		else if(pid->out < -1.0f)
		{
			pid->out = -1.0f;
		}	
}
