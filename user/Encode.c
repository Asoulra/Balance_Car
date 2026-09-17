#include "head.h"

int Read_Speed(TIM_HandleTypeDef *htim)
{
    int temp;
    temp=(short)__HAL_TIM_GetCounter(htim);
    __HAL_TIM_SetCounter(htim,0);
    return temp;
}

/**
 * @brief Clear one encoder timer and discard counts left before control starts.
 */
void Encoder_Clear(TIM_HandleTypeDef *htim)
{
    if (htim == NULL)
        return;

    __HAL_TIM_SetCounter(htim, 0);
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
}

/**
 * @brief Clear both wheel encoders used by the balance car.
 */
void Encoder_ClearAll(void)
{
    Encoder_Clear(&htim2);
    Encoder_Clear(&htim4);
}
