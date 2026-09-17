#ifndef _Encode_h
#define _Encode_h
#include "stm32f1xx_it.h"

int Read_Speed(TIM_HandleTypeDef *htim);
void Encoder_Clear(TIM_HandleTypeDef *htim);
void Encoder_ClearAll(void);

#endif
