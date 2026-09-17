#include "vofa.h"
#include "usart.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "pid.h"

/*  小车控制数据（定义在 pid.c） */
extern int Encoder_Left, Encoder_Right;
extern float pitch, roll, yaw;
extern int MOTO1, MOTO2;
extern uint8_t dmp_get_ret;

/* JustFloat 帧尾: 恒为 {0x00, 0x00, 0x80, 0x7F} */
static const uint8_t justfloat_tail[4] = {0x00, 0x00, 0x80, 0x7F};
/* printf 缓冲 */
static char printf_buf[128];
/* 周期任务节流时间戳 */
static uint32_t send_tick = 0;

/**
 * @brief 初始化 VOFA: 确认 UART 就绪
 *        注意：本工程实际配置的是 USART2（PA2=TX, PA3=RX, 115200）
 */
void VOFA_Init(void)
{
    uint8_t dummy = 0xFF;
    HAL_UART_Transmit(&huart2, &dummy, 1, 2);
}

/**
 * @brief printf 重定向到 USART2
 */
void VOFA_Printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(printf_buf, sizeof(printf_buf), fmt, args);
    va_end(args);
    if (len > 0)
    {
        HAL_UART_Transmit(&huart2, (uint8_t *)printf_buf, len, 100);
    }
}

/**
 * @brief 按 JustFloat 协议发送 float 数组
 *        data: float 数组, ch_num: 实际通道数
 */
void VOFA_SendJustFloat(float *data, uint8_t ch_num)
{
    uint8_t buf[VOFA_CH_NUM * 4 + 4]; /* 数据 + 帧尾 */
    uint8_t *p = buf;

    for (uint8_t i = 0; i < ch_num; i++)
    {
        uint32_t raw;
        memcpy(&raw, &data[i], 4);
        *p++ = (uint8_t)(raw);
        *p++ = (uint8_t)(raw >> 8);
        *p++ = (uint8_t)(raw >> 16);
        *p++ = (uint8_t)(raw >> 24);
    }

    memcpy(p, justfloat_tail, 4);
    p += 4;

    HAL_UART_Transmit(&huart2, buf, p - buf, 20);
}

/**
 * @brief 周期输出任务：主循环无条件调用，内部按模式自动节流
 *        WAVE: 每20ms一帧(50Hz)，通道: pitch roll yaw EN_L EN_R MOTO1 MOTO2
 *        TEXT: 每200ms一行状态，串口助手可读
 */
// extern short gyrox, gyroy, gyroz;
void VOFA_PeriodicTask(void)
{
#if VOFA_MODE == VOFA_MODE_WAVE
    if (HAL_GetTick() - send_tick < 20)
        return;
    send_tick = HAL_GetTick();

    float data[VOFA_CH_NUM];
    data[0] = pitch;
    data[1] = roll;
    data[2] = yaw;
    data[3] = (float)gyrox;
    data[4] = (float)gyroy;
    data[5] = (float)gyroz;

    VOFA_SendJustFloat(data, VOFA_CH_NUM);
#else
    if (HAL_GetTick() - send_tick < 200)
        return;
    send_tick = HAL_GetTick();

    VOFA_Printf("alive: pitch=%7.2f roll=%7.2f yaw=%7.2f EN_L=%6d EN_R=%6d MOTO=%6d %6d dmp_ret=%d\r\n",
                (double)pitch, (double)roll, (double)yaw,
                Encoder_Left, Encoder_Right, MOTO1, MOTO2, dmp_get_ret);
#endif
}
