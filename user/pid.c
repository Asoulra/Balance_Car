#include "head.h"
#include <math.h>
// 传感器的变量
int Encoder_Left, Encoder_Right;
float pitch, roll, yaw;
short gyrox, gyroy, gyroz;
short aacx, aacy, aacz;

// 当前 pitch/roll 已互换，因此直立环使用 X 轴角速度；若恢复原映射改为 0。
#define BALANCE_GYRO_USE_X 1

// 闭环控制
int Vertical_out, Velocity_out, Turn_out, Target_Speed, Target_turn, MOTO1, MOTO2;
volatile int Vertical_P_out, Vertical_D_out; // Vertical-loop P/D terms for OLED debug
volatile int Velocity_P_out, Velocity_I_out; // Velocity-loop P/I terms for OLED debug
// 平衡倾角偏置，单位必须与 pitch(度) 一致。原 0.03 是弧度≈1.72度，现统一用度。
float Med_Angle = -7.87f;                               //-6.23f
float Vertical_Kp = 450, Vertical_Kd = 0.52435f;        // 430 0.87391 490 0.9 0.52435
float Velocity_Kp = -1.687f, Velocity_Ki = -0.0084381f; // 20ms累计编码器计数；Ki=Kp/200 -1.687
float Turn_Kp = 0, Turn_Kd = 0;
uint8_t stop;
uint8_t pid_run = 1; // 上电直接启动 PID（调试用，专注排查为何车不动）

// 直立环pid控制器
// 期望角度，真实角度，角速度
int Vertical(float Med, float Angle, float gyo_y)
{
    float p_term = Vertical_Kp * (Angle - Med);
    float d_term = Vertical_Kd * gyo_y;

    Vertical_P_out = (int)p_term;
    Vertical_D_out = (int)d_term;
    return (int)(p_term + d_term);
}

// 速度环pid控制器
// 期望速度，真实速度（左右编码器）
int Velocity(int Target, int encoder_L, int encoder_R)
{
    // 算偏差值
    int Err, Err_Lowout, temp;
    Err = (encoder_L + encoder_R) - Target;
    // 滤波
    static int Err_LowOut_last, Encoder_S;
    static float a = 0.7;
    Err_Lowout = (1 - a) * Err + a * Err_LowOut_last;
    Err_LowOut_last = Err_Lowout;
    // 积分
    Encoder_S += Err_Lowout;
    // 积分限幅
    Encoder_S = Encoder_S > 20000 ? 20000 : (Encoder_S < (-20000) ? (-20000) : Encoder_S);
    if (stop == 1)
        Encoder_S = 0, stop = 0;
    // 速度环计算
    float p_term = Velocity_Kp * Err_Lowout;
    float i_term = Velocity_Ki * Encoder_S;

    Velocity_P_out = (int)p_term;
    Velocity_I_out = (int)i_term;
    temp = (int)(p_term + i_term);
    return temp;
}

// 转向环pid控制器
// 输入：角速度，角速值
int Turn(float gyro_Z, int Target_turn)
{
    int temp;
    temp = Turn_Kp * Target_turn + Turn_Kd * gyro_Z;
    return temp;
}

void Control(void)
{
    int PWM_out;
    static uint8_t startup_data_ready = 0;

    // 未按键启动 PID 时，电机保持停转
    if (pid_run == 0)
    {
        startup_data_ready = 0;
        MOTO1 = 0;
        MOTO2 = 0;
        Vertical_out = 0;
        Velocity_out = 0;
        Turn_out = 0;
        Encoder_Left = 0;
        Encoder_Right = 0;
        Encoder_ClearAll();
        Motor_OutputDisable();
        return;
    }

    /*
     * 上电保护：清掉编码器旧计数和全部电机命令。在收到第一帧有效的
     * MPU/DMP 数据以前，Load() 即使被调用也会被输出门控拦住，TB6612
     * 的 PWM 保持为 0，不会响应旧的控制量。
     */
    if (startup_data_ready == 0)
    {
        Encoder_ClearAll();
        Encoder_Left = 0;
        Encoder_Right = 0;
        MOTO1 = 0;
        MOTO2 = 0;
        Vertical_out = 0;
        Velocity_out = 0;
        Turn_out = 0;
        Motor_OutputDisable();

        if (mpu_dmp_get_data(&pitch, &roll, &yaw) != 0)
            return;

        startup_data_ready = 1;
        Motor_OutputEnable();
        return;
    }

    Encoder_Left = Read_Speed(&htim2);
    Encoder_Right = -Read_Speed(&htim4);
    // 每次控制都尝试读取 FIFO；没有新数据时保持上一帧姿态和角速度。
    if (mpu_dmp_get_data(&pitch, &roll, &yaw) != 0)
    {
        /* FIFO 暂时无新数据时保持上一帧姿态和角速度。 */
    }
    // 与当前互换后的 pitch 对应，使用 X 轴角速度作为直立环微分项。
    short balance_gyro = BALANCE_GYRO_USE_X ? gyrox : gyroy;
    Vertical_out = Vertical(Velocity_out + Med_Angle, pitch, balance_gyro);
    Turn_out = Turn(gyroz, Target_turn);
    PWM_out = -Vertical_out;
    MOTO1 = PWM_out - Turn_out;
    MOTO2 = PWM_out + Turn_out;
    Limit(&MOTO1, &MOTO2);
    Load(-MOTO1, MOTO2);
}
