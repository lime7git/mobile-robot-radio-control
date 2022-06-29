#ifndef __PID_H__
#define __PID_H__

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define TIME_STAMP 0.01f

typedef struct {
	
	float set_value;
	float current_value;
	float	error;
	float	previous_error;
	float	total_error;
	
	float kp;
	float ki;
	float kd;
	
	float anti_windup_limit;
	
	float out;
	
} sPid;

void PID_INIT(sPid *pid, float kp, float ki, float kd, float anti_windup_limit);
void PID_CALCULATE(sPid *pid);
void PID_RESET(sPid *pid);

#endif
