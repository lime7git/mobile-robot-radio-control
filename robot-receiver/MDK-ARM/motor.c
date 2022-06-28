#include "motor.h"

void MOTOR_INIT(sMotor *motor, TIM_HandleTypeDef *pwm_timer, uint16_t pwm_timer_channel, GPIO_TypeDef *in_a_gpio_port, uint16_t in_a_gpio_pin, GPIO_TypeDef *in_b_gpio_port, uint16_t in_b_gpio_pin, sEncoder *encoder, sPid *pid)
{
	motor->pwm_timer = pwm_timer;
	motor->pwm_timer_channel = pwm_timer_channel;
	motor->in_a_gpio_port = in_a_gpio_port;
	motor->in_a_gpio_pin = in_a_gpio_pin;
	motor->in_b_gpio_port = in_b_gpio_port;
	motor->in_b_gpio_pin = in_b_gpio_pin;
	motor->encoder = encoder;
	motor->pid = pid;
	
	motor->velocity_rpm = 0.0f;
	motor->current_velocity_rpm = 0.0f;
	motor->previous_velocity_rpm = 0.0f;
	motor->current_velocity_rpm_filtered = 0.0f;
}
void MOTOR_SET_VELOCITY(sMotor *motor, float velocity)
{
	motor->velocity_rpm = velocity;
}
void MOTOR_UPDATE_VELOCITY(sMotor *motor)
{
	motor->encoder->previous_value = motor->encoder->value;
	motor->encoder->value = ENCODER_GET_VALUE(motor->encoder);
	motor->encoder->difference = motor->encoder->value - motor->encoder->previous_value;
	
	if(motor->encoder->difference > 32768)  motor->encoder->difference -= 65536;
	if(motor->encoder->difference < -32768) motor->encoder->difference =  65536 - abs(motor->encoder->difference);
	
	motor->current_velocity_rpm = ((float)motor->encoder->difference / TIME_STAMP * 60.0f) / ENCODER_IMPULSES_PER_ROTATE;
	motor->current_velocity_rpm_filtered = 0.854f * motor->current_velocity_rpm_filtered + 0.0728f * motor->current_velocity_rpm + 0.0728f * motor->previous_velocity_rpm;
	motor->previous_velocity_rpm = motor->current_velocity_rpm;
}
void MOTOR_UPDATE_PID(sMotor *motor)
{
	motor->pid->set_value = motor->velocity_rpm;
	motor->pid->current_value = motor->current_velocity_rpm_filtered;
}
void MOTOR_UPDATE_PWM(sMotor *motor)
{
	if(motor->pid->out > 0.01f)
	{
		HAL_GPIO_WritePin(motor->in_a_gpio_port, motor->in_a_gpio_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor->in_b_gpio_port, motor->in_b_gpio_pin, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_timer_channel, (uint16_t)(fabs(motor->pid->out) * 999.0f));
	}
		else if(motor->pid->out < -0.01f)
		{
			HAL_GPIO_WritePin(motor->in_a_gpio_port, motor->in_a_gpio_pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(motor->in_b_gpio_port, motor->in_b_gpio_pin, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_timer_channel, (uint16_t)(fabs(motor->pid->out) * 999.0f));
		}
			else
			{
				HAL_GPIO_WritePin(motor->in_a_gpio_port, motor->in_a_gpio_pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(motor->in_b_gpio_port, motor->in_b_gpio_pin, GPIO_PIN_RESET);
				__HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_timer_channel, 1000);
			}		
}
void MOTOR_STOP(sMotor *motor)
{
	HAL_GPIO_WritePin(motor->in_a_gpio_port, motor->in_a_gpio_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor->in_b_gpio_port, motor->in_b_gpio_pin, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_timer_channel, 1000);
}
