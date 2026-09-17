#ifndef _motor_h
#define _motor_h
#include "stm32f1xx_it.h"
extern uint32_t last_tick;
void Load(int moto1, int moto2);
void Limit(int *motoA, int *motoB);
void Motor_OutputDisable(void);
void Motor_OutputEnable(void);

#endif
