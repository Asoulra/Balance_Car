#ifndef _pid_h
#define _pid_h
#include "stm32f1xx_it.h"

//  PID 变量
extern float Vertical_Kp, Vertical_Kd;
extern float Velocity_Kp, Velocity_Ki;
extern float Turn_Kp, Turn_Kd;
extern float Med_Angle;
extern int Target_Speed, Target_turn;
extern uint8_t stop;
extern uint8_t pid_run;   // 按键启动 PID 控制标志，0=停止，1=运行
extern short gyrox, gyroy, gyroz;
// 各环输出 & 传感器，供 OLED / 串口调试显示
extern int Encoder_Left, Encoder_Right;
extern float pitch, roll, yaw;
extern int Vertical_out, Velocity_out, Turn_out;
extern volatile int Vertical_P_out, Vertical_D_out;
extern volatile int Velocity_P_out, Velocity_I_out;
extern int MOTO1, MOTO2;
// PID 函数
int Vertical(float Med, float Angle, float gyo_y);
int Velocity(int Target, int encoder_L, int encoder_R);
int Turn(float gyro_Z, int Target_turn);
void Control(void);

#endif
