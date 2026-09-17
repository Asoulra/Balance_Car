/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main  program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "head.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MOTOR_TEST_WITHOUT_MPU 0 // 1: 上电自检电机 1 秒（无 MPU/DMP 时可用）；0: 不自检，直接进入 PID 控制
#define BALANCE_LOOP_ONLY 1      // 1:关闭速度环     0: 开启速度环
#define ENABLE_UPRIGHT_CONTROL 1 // 1: 允许PID 控制；0: PID 控制（仅用于调试）
#define CONTROL_DEBUG_OLED 0     // 1=MPU initialization diagnostics, 0=PID component-ratio view
#define OLED_REFRESH_MS 100U     // OLED refresh period (ms)
#define OLED_PID_VIEW 1          // 0=速度环 P/I/D, 1=直立环 loop P/I/D
#define SPEED_DIRECTION_OLED 0   // 1=OLED显示100ms编码器累计值，用于检查速度环方向
#define OLED_ENABLE 1            // 0=完全关闭OLED初始化与刷新，用于排查串口

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// int Encoder_left,Encoder_Right;
// float pitch,roll,yaw;
uint32_t sys_tick;
uint8_t flag_10s = 0; // TIM3 中断计数（5ms/次）
uint8_t mpu_ok = 0;   // DMP 初始化成功标志，供 TIM3 中断内控制使用
uint8_t mpu_ack = 1;
uint8_t mpu_who = 0;
uint8_t mpu_addr = 0;
uint8_t i2c_bus_ok = 1;
extern float roll, pitch, yaw;
extern int Encoder_Left, Encoder_Right;
extern float distance;
extern int MOTO1, MOTO2;
static volatile int32_t encoder_left_sum = 0;
static volatile int32_t encoder_right_sum = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// 閺勫墽銇?
// void R65ad(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// OLED 显示带符号整数：固定符号位 + len 位数值（前导零填充空格）
static void OLED_ShowSignedNum(uint8_t x, uint8_t y, int num, uint8_t len, uint8_t size2)
{
  if (num < 0)
  {
    OLED_ShowChar(x, y, '-', size2);
    num = -num;
  }
  else
  {
    OLED_ShowChar(x, y, '+', size2);
  }
  OLED_ShowNum(x + size2 / 2, y, (unsigned int)num, len, size2);
}

// Integer absolute value used for PID component percentages.
static int AbsInt(int value)
{
  return (value < 0) ? -value : value;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint8_t dmp_ret = 0; // DMP 初始化返回值（全局，供主循环显示）
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // // 直接操作寄存器释�? PB3(JTDO)/PB4(JNTRST)，仅保留 SWD(PA13/PA14)
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
  AFIO->MAPR = (AFIO->MAPR & ~0x07000000) | 0x02000000; // SWJ_CFG=010: JTAG off, SWD on
  for (volatile int _i = 0; _i < 100; _i++)
    ; // �? AFIO 生效
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  VOFA_Init();
  HAL_Delay(500);
  i2c_bus_ok = (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(0x3C << 1), 3, 100) == HAL_OK) ? 0 : 1;
  mpu_ack = MPU_CheckDevice();
  mpu_addr = MPU_GetAddress();
  mpu_who = MPU_Read_Byte(MPU_DEVICE_ID_REG);

  dmp_ret = mpu_dmp_init();
  if (dmp_ret == 0)
  {
    mpu_ok = 1;
  }
  else
  {
  }

  /* 电机 PWM/编码器与 MPU/DMP 解耦，先初始化硬件。 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  Load(0, 0);

#if MOTOR_TEST_WITHOUT_MPU
  /* 无 MPU 时执行 1 秒电机自检；确认正常后可将宏改为 0。 */
  Load(3600, 3600);
  HAL_Delay(1000);
  Load(0, 0);
#endif

  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE BEGIN 2 */
#if OLED_ENABLE
  /* OLED 显示初始化，用于显示 PID 状态/按键自检（无需串口） */
  OLED_Init();
  OLED_Clear();
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#if OLED_ENABLE
    static uint32_t oled_refresh_tick = 0;
#endif
    /* 控制已在 TIM3 中断内以固定 10ms 硬实时节拍执行，不受串口/延时阻塞干扰 */
    /* 串口输出：文本/波形模式在 vofa.h 里切换，此函数内部自动节流 */
    VOFA_PeriodicTask();

#if OLED_ENABLE
    /* OLED PID component view: each P/I/D term is shown as a percentage of
       the sum of absolute component outputs. Select speed or balance loop with
       OLED_PID_VIEW at the top of this file. */
#if SPEED_DIRECTION_OLED
    if ((HAL_GetTick() - oled_refresh_tick) >= OLED_REFRESH_MS)
    {
      int32_t encoder_left_display;
      int32_t encoder_right_display;
      int32_t encoder_sum_display;

      oled_refresh_tick = HAL_GetTick();

      /* 与TIM3中断交换累计值，避免显示刷新期间丢失编码器脉冲。 */
      __disable_irq();
      encoder_left_display = encoder_left_sum;
      encoder_right_display = encoder_right_sum;
      encoder_left_sum = 0;
      encoder_right_sum = 0;
      __enable_irq();

      encoder_sum_display = encoder_left_display + encoder_right_display;

      OLED_Clear();
      OLED_ShowString(0, 0, (uint8_t *)"EL:", 16);
      OLED_ShowSignedNum(32, 0, (int)encoder_left_display, 5, 16);
      OLED_ShowString(0, 2, (uint8_t *)"ER:", 16);
      OLED_ShowSignedNum(32, 2, (int)encoder_right_display, 5, 16);
      OLED_ShowString(0, 4, (uint8_t *)"SUM:", 16);
      OLED_ShowSignedNum(40, 4, (int)encoder_sum_display, 5, 16);
      OLED_ShowString(0, 6, (uint8_t *)"FWD KP:", 16);
      if (encoder_sum_display > 2)
        OLED_ShowChar(64, 6, '-', 16);
      else if (encoder_sum_display < -2)
        OLED_ShowChar(64, 6, '+', 16);
      else
        OLED_ShowChar(64, 6, '?', 16);
    }
#elif CONTROL_DEBUG_OLED
    int pitch_x10 = (int)(pitch * 10.0f);
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"A:", 16);
    OLED_ShowSignedNum(16, 0, mpu_ack, 1, 16);
    OLED_ShowString(40, 0, (uint8_t *)"B:", 16);
    OLED_ShowSignedNum(56, 0, i2c_bus_ok, 1, 16);
    OLED_ShowString(80, 0, (uint8_t *)"E:", 16);
    OLED_ShowSignedNum(96, 0, dmp_ret, 2, 16);

    OLED_ShowString(0, 2, (uint8_t *)"ADDR:", 16);
    OLED_ShowSignedNum(56, 2, mpu_addr, 3, 16);

    OLED_ShowString(0, 4, (uint8_t *)"WHO:", 16);
    OLED_ShowSignedNum(40, 4, mpu_who, 3, 16);

    OLED_ShowString(0, 6, (uint8_t *)"M1:", 16);
    OLED_ShowSignedNum(32, 6, MOTO1, 5, 16);
#else
    if ((HAL_GetTick() - oled_refresh_tick) >= OLED_REFRESH_MS)
    {
      int p_out, i_out, d_out;
      int p_pct, i_pct, d_pct;
      int total_out;

      oled_refresh_tick = HAL_GetTick();

#if OLED_PID_VIEW == 1
      OLED_ShowString(0, 0, (uint8_t *)"PID: Stand", 16);
      p_out = Vertical_P_out;
      i_out = 0;
      d_out = Vertical_D_out;
#else
      OLED_ShowString(0, 0, (uint8_t *)"PID: Speed", 16);
      p_out = Velocity_P_out;
      i_out = Velocity_I_out;
      d_out = 0;
#endif

      total_out = AbsInt(p_out) + AbsInt(i_out) + AbsInt(d_out);
      p_pct = (total_out != 0) ? (int)((long)p_out * 100 / total_out) : 0;
      i_pct = (total_out != 0) ? (int)((long)i_out * 100 / total_out) : 0;
      d_pct = (total_out != 0) ? (int)((long)d_out * 100 / total_out) : 0;

      OLED_Clear();
      if (total_out == 0)
      {
        OLED_ShowString(0, 2, (uint8_t *)"PID output = 0", 16);
      }
      else
      {
        OLED_ShowString(0, 2, (uint8_t *)"P:", 16);
        OLED_ShowSignedNum(32, 2, p_pct, 3, 16);
        OLED_ShowChar(64, 2, '%', 16);

        OLED_ShowString(0, 4, (uint8_t *)"I:", 16);
        OLED_ShowSignedNum(32, 4, i_pct, 3, 16);
        OLED_ShowChar(64, 4, '%', 16);

        OLED_ShowString(0, 6, (uint8_t *)"D:", 16);
        OLED_ShowSignedNum(32, 6, d_pct, 3, 16);
        OLED_ShowChar(64, 6, '%', 16);
      }
    }
#endif
#endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static uint8_t tick20ms = 0;
  if (htim->Instance == TIM3)
  {
#if SPEED_DIRECTION_OLED
    /* 方向测试仅采集编码器，禁止电机输出，便于安全地手推车轮。 */
    Encoder_Left = Read_Speed(&htim2);
    Encoder_Right = -Read_Speed(&htim4);
    encoder_left_sum += Encoder_Left;
    encoder_right_sum += Encoder_Right;
    Load(0, 0);
#else
    /* 固定 2ms 节拍*/
#if ENABLE_UPRIGHT_CONTROL
    if (mpu_ok)
    {
      Control();
      encoder_left_sum += Encoder_Left;
      encoder_right_sum += Encoder_Right;

      /* 速度环每20ms运行一次，使用期间内全部10次编码器计数。 */
      if (++tick20ms >= 10)
      {
        tick20ms = 0;
#if BALANCE_LOOP_ONLY
        Velocity_out = 0;
#else
        Velocity_out = Velocity(Target_Speed, (int)encoder_left_sum, (int)encoder_right_sum);
#endif
        encoder_left_sum = 0;
        encoder_right_sum = 0;
      }
    }
#else
    Load(0, 0);
#endif
#endif
  }
}
// void Read(void){
//   if(uwTick-sys_tick<10)
//     return;
//   sys_tick=uwTick;
//   Encoder_Left=Read_Speed(&htim2);
//   Encoder_Right=-Read_Speed(&htim4);
// }
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
