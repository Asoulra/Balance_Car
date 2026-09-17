#include "head.h"

#define PWM_MAX 7199 // TIM1 ARR = 7200-1
#define PWM_Dead 100
#define PWM_Dead_Comp 500 // 489
uint32_t last_tick = 0;
static uint8_t motor_output_enabled = 0;

/* Immediately remove PWM and put all TB6612 direction inputs low. */
static void Motor_ClearHardwareOutput(void)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_13 |
                                 GPIO_PIN_14 | GPIO_PIN_15,
                      GPIO_PIN_RESET);
}

void Motor_OutputDisable(void)
{
    motor_output_enabled = 0;
    Motor_ClearHardwareOutput();
}

void Motor_OutputEnable(void)
{
    /* Always begin from zero; the next Load() supplies the first valid command. */
    Motor_ClearHardwareOutput();
    motor_output_enabled = 1;
    last_tick = HAL_GetTick();
}

void Limit(int *motoA, int *motoB)
{
    if (*motoA > PWM_MAX)
        *motoA = PWM_MAX;
    if (*motoA < -PWM_MAX)
        *motoA = -PWM_MAX;
    if (*motoB > PWM_MAX)
        *motoB = PWM_MAX;
    if (*motoB < -PWM_MAX)
        *motoB = -PWM_MAX;

    if (*motoA > PWM_Dead)
        *motoA = *motoA + PWM_Dead_Comp;
    if (*motoA < -PWM_Dead)
        *motoA = *motoA - PWM_Dead_Comp;
    if (*motoB > PWM_Dead)
        *motoB = *motoB + PWM_Dead_Comp;
    if (*motoB < -PWM_Dead)
        *motoB = *motoB - PWM_Dead_Comp;
}

int abs(int p)
{
    if (p > 0)
        return p;
    else
        return -p;
}

void Load(int moto1, int moto2)
{
    /* Power-on safety gate: accept calls, but do not drive TB6612 until armed. */
    if (motor_output_enabled == 0)
    {
        Motor_ClearHardwareOutput();
        return;
    }

    if (HAL_GetTick() - last_tick < 2)
        return;
    if (moto1 < 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    }
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, abs(moto1));

    if (moto2 < 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    }
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, abs(moto2));
    last_tick = HAL_GetTick();
}
