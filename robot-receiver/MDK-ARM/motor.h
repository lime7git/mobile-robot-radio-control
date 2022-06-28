#ifndef __MOTOR_H__
#define __MOTOR_H__

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "encoder.h"
#include "pid.h"
#include <math.h>
#include <stdlib.h>

typedef struct {
	
	TIM_HandleTypeDef *pwm_timer;
	uint16_t		  		pwm_timer_channel;
	
	GPIO_TypeDef 			*in_a_gpio_port;
	uint16_t	 				in_a_gpio_pin;
	
	GPIO_TypeDef 			*in_b_gpio_port;
	uint16_t	 				in_b_gpio_pin;
	
	sEncoder 					*encoder;
	sPid							*pid;
	
	float							velocity_rpm;
	
	float 						current_velocity_rpm;
	float							previous_velocity_rpm;
	float							current_velocity_rpm_filtered;

	
} sMotor;

void MOTOR_INIT(sMotor *motor, TIM_HandleTypeDef *pwm_timer, uint16_t pwm_timer_channel, GPIO_TypeDef *in_a_gpio_port, uint16_t in_a_gpio_pin, GPIO_TypeDef *in_b_gpio_port, uint16_t in_b_gpio_pin, sEncoder *encoder, sPid *pid);
void MOTOR_SET_VELOCITY(sMotor *motor, float velocity);
void MOTOR_UPDATE_VELOCITY(sMotor *motor);
void MOTOR_UPDATE_PID(sMotor *motor);
void MOTOR_UPDATE_PWM(sMotor *motor);
void MOTOR_STOP(sMotor *motor);

#endif
