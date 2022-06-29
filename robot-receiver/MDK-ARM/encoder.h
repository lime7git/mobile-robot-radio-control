#ifndef __ENCODER_H__
#define __ENCODER_H__

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define ENCODER_IMPULSES_PER_ROTATE 1920.0f

typedef struct {
	
	TIM_HandleTypeDef *encoder_timer;
	
	int32_t	value;
	int32_t previous_value;
	int32_t difference;
	
} sEncoder;

void ENCODER_INIT(sEncoder *encoder, TIM_HandleTypeDef *encoder_timer);
int32_t ENCODER_GET_VALUE(sEncoder *encoder);

#endif
