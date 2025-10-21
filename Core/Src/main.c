/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "EventRecorder.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// 烙铁控制状态
typedef enum {
    STATE_HEATING = 0,      // 加热并等待测温间隔
    STATE_PWM_OFF_WAIT_STABLE, // 关闭PWM等待温度稳定
    STATE_MEASURE_TEMP,     // 读取ADC温度
    STATE_CALCULATE_PWM,    // 计算新的PWM
    STATE_UPDATE_DISPLAY    // 更新OLED显示
} T12_Control_State_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// PWM配置
#define PWM_PERIOD          2000 // TIM3 ARR+1, 例如 ARR=1999
#define PWM_MAX_DUTY        (PWM_PERIOD - 1)

// PID 控制参数
#define PID_KP              8.0f
#define PID_KI              0.0f
#define PID_KD              0.0f
#define PID_INTEGRAL_MAX    1999.0f
#define PID_INTEGRAL_MIN    0.0f

#define NUM_CALIBRATION_POINTS (sizeof(temp_calibration_table) / sizeof(TempCalibrationPoint))
#define DEFAULT_TARGET_TEMP 100
#define BANG_BANG_THRESHOLD 20 // 切换PID控制的温差阈值
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// 温度校准点 (ADC原始值 -> 实际温度 °C)
// 结构体定义
typedef struct {
    uint16_t adc_val;
    uint16_t temp_c;
} TempCalibrationPoint;

// 校准数据表
const TempCalibrationPoint temp_calibration_table[] = {
    {0,    25},       // ADC为0时，近似室温
    {1212, 216},      // 校准点1
    {1696, 308},      // 校准点2
    {2181, 390},      // 校准点3
    // 可以根据需要添加更多点
};

PID_Controller t12_pid = {
    .Kp = PID_KP,
    .Ki = PID_KI,
    .Kd = PID_KD,
    .integral_sum = 0,
    .last_error = 0,
    .last_measurement = 0 // 为改进的D项新增
};

T12_Control_State_t t12_state = STATE_PWM_OFF_WAIT_STABLE;
uint32_t uWTick = 0;   // 自芯片启动以来的ms
uint32_t last_time_check = 0;   // 上次执行时间
const uint32_t ADC_STABLE_WAIT_MS = 1; // 测温稳定等待时间 (替代 LL_mDelay(1))
const uint32_t CONTROL_PERIOD_MS = 20; // 主控制周期 (替代 LL_mDelay(20))

// 存储编码器计数值（旋转步数）
volatile int16_t rotaryCount = 0; 
// 存储上一次读取到的 A相/CLK 的状态
volatile uint8_t lastCLKState = 0;

char sprintf_tmp[16];
static u8g2_t u8g2;
uint16_t t12_now = 0, t12_pwm = 0, temp_target = DEFAULT_TARGET_TEMP;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  EventRecorderInitialize(EventRecordAll, 1U);
  EventRecorderStart();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 3);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  printf("CPU @ %d Hz\n", SystemCoreClock);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  Activate_ADC();
  u8g2_Setup_ssd1306_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_4wire_hw_spi, u8x8_stm32_gpio_and_delay);
  u8g2_InitDisplay(&u8g2);
  u8g2_SetPowerSave(&u8g2, 0);
  
  LL_TIM_EnableAllOutputs(TIM3);
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_SetCompareCH1(TIM3, 0);
  LL_TIM_EnableCounter(TIM3);
  LL_TIM_EnableCounter(TIM14);
  LL_TIM_EnableIT_CC1(TIM14);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		uint32_t current_time = uWTick;

		// =========================================================================
		// 状态机控制逻辑
		// =========================================================================

		switch (t12_state)
			{
        case STATE_HEATING:
            // 状态 0: 加热中，等待下一个控制周期
            if (current_time - last_time_check >= CONTROL_PERIOD_MS)
            {
                // 周期时间到，准备进入测温阶段
                LL_TIM_OC_SetCompareCH1(TIM3, 0); // 关闭加热
                
                last_time_check = current_time; // 重置计时器
                t12_state = STATE_PWM_OFF_WAIT_STABLE; // 切换到等待稳定状态
            }
            break;

        case STATE_PWM_OFF_WAIT_STABLE:
            // 状态 1: 关闭PWM，等待测温稳定
            if (current_time - last_time_check >= ADC_STABLE_WAIT_MS)
            {
                // 等待时间到，开始测温
                t12_state = STATE_MEASURE_TEMP; // 切换到测温状态
            }
            break;

        case STATE_MEASURE_TEMP:
            // 状态 2: 读取ADC温度
            t12_now = calculateTemp(T12_ADC_Read()); 
            
            t12_state = STATE_CALCULATE_PWM; // 切换到计算PWM状态
            break;

        case STATE_CALCULATE_PWM:
            // 状态 3: 计算新的PWM值
            {
                int16_t temp_error = (int16_t)temp_target - (int16_t)t12_now;

                if (temp_error > 0) // 需要加热
                {
                    if (temp_error > 20)
                    {
                        // 开关控制：全速加热
                        t12_pwm = 1999;
                        // 重置积分项
                        t12_pid.integral_sum = 0.0f; 
                        t12_pid.last_error = 0;
                    }
                    else
                    {
                        // PID控制：精确控温
                        t12_pwm = calculate_pid_pwm(&t12_pid, temp_target, t12_now);
                    }
                }
                else // 温度已达目标或过高
                {
                    t12_pwm = 0; // 关闭加热
                    // 防止积分项累积
                    t12_pid.integral_sum = 0.0f;
                    t12_pid.last_error = temp_error;
                }
                LL_TIM_OC_SetCompareCH1(TIM3, t12_pwm); // 应用新的加热功率
            }
            last_time_check = uWTick; // 重新开始计时，用于下一个控制周期
            t12_state = STATE_UPDATE_DISPLAY; // 切换到更新显示状态
            break;

        case STATE_UPDATE_DISPLAY:
            // 状态 4: 更新OLED显示 (可以增加一个更慢的计时器来控制显示频率)
            u8g2_FirstPage(&u8g2);
            do
            {
              u8g2_SetFontMode(&u8g2, 1);
              u8g2_SetFontDirection(&u8g2, 0);
              u8g2_SetFont(&u8g2, u8g2_font_9x15_tr);
              sprintf(sprintf_tmp, "vcc:%d mV", Vref_Read());
              u8g2_DrawStr(&u8g2, 0, 10, sprintf_tmp);
              sprintf(sprintf_tmp, "vin:%d mV", Vin_Read());
              u8g2_DrawStr(&u8g2, 0, 22, sprintf_tmp);
              sprintf(sprintf_tmp, "tmp:%d C", Temp_ADC_Read());
              u8g2_DrawStr(&u8g2, 0, 34, sprintf_tmp);
              sprintf(sprintf_tmp, "%d, %d, %d%%", temp_target, t12_now, t12_pwm/20);
              u8g2_DrawStr(&u8g2, 0, 46, sprintf_tmp);
              sprintf(sprintf_tmp, "enc:%d", Get_Rotary_Step_And_Clear());
              u8g2_DrawStr(&u8g2, 0, 58, sprintf_tmp);
            } while (u8g2_NextPage(&u8g2));
            
            t12_state = STATE_HEATING; // 切换回加热等待状态，开始下一个周期
            break;

        default:
            // 错误状态处理
            t12_state = STATE_HEATING;
            break;
    }
//    LL_TIM_OC_SetCompareCH1(TIM3, 0);
//    LL_mDelay(2);
//    t12_now = calculateTemp(T12_ADC_Read());

//    if (t12_now > 480) {
//        t12_pwm = 0; // 过热保护
//    } else {
//        // --- 混合控制逻辑 ---
//        int16_t error = temp_target - t12_now;

//        // 如果温差大于阈值，使用开关控制
//        if (abs(error) > BANG_BANG_THRESHOLD) {
//            if (error > 0) {
//                // 温度远低于目标，全速加热
//                t12_pwm = PWM_MAX_DUTY; 
//            } else {
//                // 温度高于目标（通常是降温时），关闭加热
//                t12_pwm = 0;
//            }
//            // 【关键】重置PID积分项，为平滑切换做准备
//            t12_pid.integral_sum = 0;
//            // 同时更新last_measurement，防止D项在切换时产生尖峰
//            t12_pid.last_measurement = t12_now; 
//        } else {
//            // 温差小于等于阈值，使用PID精确控制
//            t12_pwm = calculate_pid_pwm(&t12_pid, temp_target, t12_now);
//        }
//    }
//    printf("%dC, %d%%\n", t12_now, t12_pwm/20);
//    LL_TIM_OC_SetCompareCH1(TIM3, t12_pwm);
//  
//    u8g2_FirstPage(&u8g2);
//    do
//    {
//      u8g2_SetFontMode(&u8g2, 1);
//      u8g2_SetFontDirection(&u8g2, 0);
//      u8g2_SetFont(&u8g2, u8g2_font_9x15_tr);
//      sprintf(sprintf_tmp, "vcc:%d mV", Vref_Read());
//      u8g2_DrawStr(&u8g2, 0, 10, sprintf_tmp);
//      sprintf(sprintf_tmp, "vin:%d mV", Vin_Read());
//      u8g2_DrawStr(&u8g2, 0, 22, sprintf_tmp);
//      sprintf(sprintf_tmp, "tmp:%d C", Temp_ADC_Read());
//      u8g2_DrawStr(&u8g2, 0, 34, sprintf_tmp);
//      sprintf(sprintf_tmp, "T12:%d, %d%%", t12_now, t12_pwm/20);
//      u8g2_DrawStr(&u8g2, 0, 46, sprintf_tmp);
//      sprintf(sprintf_tmp, "enc:%d", Get_Rotary_Step_And_Clear());
//      u8g2_DrawStr(&u8g2, 0, 58, sprintf_tmp);
//    } while (u8g2_NextPage(&u8g2));
//    LL_mDelay(48);

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
  /* HSI configuration and activation */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Sysclk activation on the HSI */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {
  }

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);

  LL_Init1msTick(16000000);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(16000000);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI4_15_IRQn interrupt configuration */
  NVIC_SetPriority(EXTI4_15_IRQn, 2);
  NVIC_EnableIRQ(EXTI4_15_IRQn);
  /* TIM14_IRQn interrupt configuration */
  NVIC_SetPriority(TIM14_IRQn, 3);
  NVIC_EnableIRQ(TIM14_IRQn);
}

/* USER CODE BEGIN 4 */

volatile uint8_t control_flag = 0;

uint16_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
  // 增加边界检查，防止除以零
  if (in_max == in_min) {
    return out_min;
  }
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint16_t calculateTemp(uint16_t RawTemp)
{
    // 如果ADC值低于第一个校准点，直接映射
    if (RawTemp <= temp_calibration_table[1].adc_val) {
        return map(RawTemp,
                   temp_calibration_table[0].adc_val,
                   temp_calibration_table[1].adc_val,
                   temp_calibration_table[0].temp_c,
                   temp_calibration_table[1].temp_c);
    }

    // 遍历校准表，找到合适的区间进行线性插值
    for (size_t i = 1; i < NUM_CALIBRATION_POINTS - 1; ++i) {
        if (RawTemp <= temp_calibration_table[i+1].adc_val) {
            return map(RawTemp,
                       temp_calibration_table[i].adc_val,
                       temp_calibration_table[i+1].adc_val,
                       temp_calibration_table[i].temp_c,
                       temp_calibration_table[i+1].temp_c);
        }
    }

    // 如果超出所有范围，则使用最后两个点进行外插
    return map(RawTemp,
               temp_calibration_table[NUM_CALIBRATION_POINTS-2].adc_val,
               temp_calibration_table[NUM_CALIBRATION_POINTS-1].adc_val,
               temp_calibration_table[NUM_CALIBRATION_POINTS-2].temp_c,
               temp_calibration_table[NUM_CALIBRATION_POINTS-1].temp_c);
}

uint16_t calculate_pid_pwm(PID_Controller *pid, uint16_t target, uint16_t current)
{
    int16_t error = (int16_t)target - (int16_t)current;
    float P_term, I_term, D_term;
    float output;
    
    // 采样时间 (dt)，因为我们用了定时器中断，所以这是一个常量
    const float dt = 0.05f; // 50ms

    // 比例项 (P)
    P_term = pid->Kp * error;

    // 积分项 (I)
    pid->integral_sum += (float)error * dt; // 积分项与时间相关
    // 积分限幅
    if (pid->integral_sum > PID_INTEGRAL_MAX) pid->integral_sum = PID_INTEGRAL_MAX;
    if (pid->integral_sum < PID_INTEGRAL_MIN) pid->integral_sum = PID_INTEGRAL_MIN;
    I_term = pid->Ki * pid->integral_sum;

    // 微分项 (D) - 基于测量值变化 (Derivative on Measurement)
    // 避免了目标值突变带来的微分冲击
    D_term = pid->Kd * ((float)current - (float)pid->last_measurement) / dt;
    pid->last_measurement = current; // 更新上次的测量值

    // PID输出计算 (注意D项是负的，因为它是对输出起抑制作用)
    output = P_term + I_term - D_term;

    // 输出限幅
    if (output > PWM_MAX_DUTY) output = PWM_MAX_DUTY;
    if (output < 0.0) output = 0.0;

    return (uint16_t)output;
}

/**
 * @brief 获取编码器步数变化值并清零
 * @return 编码器步数变化值（>0 为正转，<0 为反转，=0 为未动）
 */
int16_t Get_Rotary_Step_And_Clear(void)
{
    int16_t step = 0;
    step = rotaryCount;
		if(step > 0)
		{
			temp_target += 5;
		} 
		else if(step < 0)
		{
			temp_target -= 5;
		}
    rotaryCount = 0; // 清除计数值
    return step;
}
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

#ifdef  USE_FULL_ASSERT
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
