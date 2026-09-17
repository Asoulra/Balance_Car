#ifndef _vofa_h
#define _vofa_h
#include "stm32f1xx_hal.h"
#include <stdint.h>

/*  VOFA+ JustFloat 波形通道数 */
#define VOFA_CH_NUM 6

/* ============ 输出模式切换（改这里） ============
 * VOFA_MODE_TEXT : ASCII 文本，串口助手(XCOM/SSCOM)可读，波特率115200
 * VOFA_MODE_WAVE : VOFA+ JustFloat 波形，波特率115200，协议选 JustFloat
 */
#define VOFA_MODE_TEXT 1
#define VOFA_MODE_WAVE 0
#define VOFA_MODE VOFA_MODE_WAVE

void VOFA_Init(void);
void VOFA_Printf(const char *fmt, ...);
void VOFA_SendJustFloat(float *data, uint8_t ch_num);
/* 周期输出任务：主循环无条件调用，内部自动节流
 * TEXT: 每200ms一行状态；WAVE: 每10ms一帧(100Hz) */
void VOFA_PeriodicTask(void);

#endif
