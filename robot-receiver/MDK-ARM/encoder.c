#include "encoder.h"

void ENCODER_INIT(sEncoder *encoder, TIM_HandleTypeDef *encoder_timer)
{
	encoder->encoder_timer = encoder_timer;
	encoder->value = 0;
	encoder->previous_value = 0;
	encoder->difference = 0;
}
int32_t ENCODER_GET_VALUE(sEncoder *encoder)
{
	return encoder->value = encoder->encoder_timer->Instance->CNT;
}
