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
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// ��������״̬
typedef enum {
    STATE_HEATING = 0,      // ���Ȳ��ȴ����¼��
    STATE_PWM_OFF_WAIT_STABLE, // �ر�PWM�ȴ��¶��ȶ�
    STATE_MEASURE_TEMP,     // ��ȡADC�¶�
    STATE_CALCULATE_PWM,    // �����µ�PWM
    STATE_UPDATE_DISPLAY    // ����OLED��ʾ
} T12_Control_State_t;

// �¶�У׼�� (ADCԭʼֵ -> ʵ���¶� ��C)
// �ṹ�嶨��
typedef struct {
    uint16_t adc_val;
    uint16_t temp_c;
} TempCalibrationPoint;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// PWM����
#define PWM_PERIOD          2000 // TIM3 ARR+1, ���� ARR=1999
#define PWM_MAX_DUTY        (PWM_PERIOD - 1)

// PID ���Ʋ���
#define PID_KP              5.f
#define PID_KI              1.f
#define PID_KD              0.2f
#define PID_INTEGRAL_MAX    1999.0f
#define PID_INTEGRAL_MIN    0.0f
#define PID_DELTA_T         1 // 50ms

#define NUM_CALIBRATION_POINTS (sizeof(temp_calibration_table) / sizeof(TempCalibrationPoint))
#define DEFAULT_TARGET_TEMP 205
#define BANG_BANG_THRESHOLD 20 // �л�PID���Ƶ��²���ֵ
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// У׼���ݱ�
const TempCalibrationPoint temp_calibration_table[] = {
    {0,    25},       // ADCΪ0ʱ����������
    {1212, 216},      // У׼��1
    {1696, 308},      // У׼��2
    {2181, 390},      // У׼��3
    // ���Ը�����Ҫ��Ӹ����
};

arm_pid_instance_f32 PID = {
        .Kp = PID_KP,
        .Ki = PID_KI * PID_DELTA_T,    
        .Kd = PID_KD / PID_DELTA_T 
    };

T12_Control_State_t t12_state = STATE_PWM_OFF_WAIT_STABLE;
uint32_t uWTick = 0;   // ��оƬ����������ms
uint32_t last_time_check = 0;   // �ϴ�ִ��ʱ��
const uint32_t ADC_STABLE_WAIT_MS = 1; // �����ȶ��ȴ�ʱ��
const uint32_t CONTROL_PERIOD_MS = 100; // ����������

// �洢����������ֵ����ת������
volatile int16_t rotaryCount = 0; 
// �洢��һ�ζ�ȡ���� A��/CLK ��״̬
volatile uint8_t lastCLKState = 0;

char sprintf_tmp[16];
static u8g2_t u8g2;
uint16_t t12_now = 0, t12_pwm = 0, temp_target = DEFAULT_TARGET_TEMP;
float input, output=0;
// ��Ļˢ�¼�ʱ��������
uint32_t last_display_time = 0; // �ϴ���Ļˢ��ʱ��
const uint32_t DISPLAY_PERIOD_MS = 100; // ��Ļˢ������ 100ms
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
int main(void){
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
  arm_pid_init_f32(&PID, 1);
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
    Get_Rotary_Step_And_Clear();
    // =========================================================================
    // ״̬�������߼�
    // =========================================================================

    switch (t12_state)
      {
        case STATE_HEATING:
            // ״̬ 0: �����У��ȴ���һ����������
            if (current_time - last_time_check >= CONTROL_PERIOD_MS)
            {
                // ����ʱ�䵽��׼��������½׶�
                LL_TIM_OC_SetCompareCH1(TIM3, 0); // �رռ���
                last_time_check = current_time; // ���ü�ʱ��
                t12_state = STATE_PWM_OFF_WAIT_STABLE; // �л����ȴ��ȶ�״̬
            }
            break;

        case STATE_PWM_OFF_WAIT_STABLE:
            // ״̬ 1: �ر�PWM���ȴ������ȶ�
            if (current_time - last_time_check >= ADC_STABLE_WAIT_MS)
            {
                // �ȴ�ʱ�䵽����ʼ����
                t12_state = STATE_MEASURE_TEMP; // �л�������״̬
            }
            break;

        case STATE_MEASURE_TEMP:
            // ״̬ 2: ��ȡADC�¶�
            t12_now = calculateTemp(T12_ADC_Read()); 
            //printf("t12:%d ", t12_now);
            t12_state = STATE_CALCULATE_PWM; // �л�������PWM״̬
            break;

        case STATE_CALCULATE_PWM:
            // ״̬ 3: �����µ�PWMֵ
            {
                int16_t temp_error = (int16_t)temp_target - (int16_t)t12_now;
                if (temp_error > BANG_BANG_THRESHOLD)
                {
                  // ���ؿ��ƣ�ȫ�ټ���
                  //t12_pwm = 1999;
                  t12_pwm = calculate_pid_pwm(temp_target, t12_now);
                }
                else
                {
                  // PID���ƣ���ȷ����
                  t12_pwm = calculate_pid_pwm(temp_target, t12_now);
                }
                SEGGER_RTT_printf(0,"%d %d %d\n", temp_target, t12_now, t12_pwm);
                //printf("err:%d pwm:%d\n", temp_error, t12_pwm);
                LL_TIM_OC_SetCompareCH1(TIM3, t12_pwm); // Ӧ���µļ��ȹ���
            }
            last_time_check = uWTick; // ���¿�ʼ��ʱ��������һ����������
            t12_state = STATE_UPDATE_DISPLAY; // �л���������ʾ״̬
            break;

        case STATE_UPDATE_DISPLAY:
            // ״̬ 4: ����OLED��ʾ (��������һ�������ļ�ʱ����������ʾƵ��)
            if (current_time - last_display_time >= DISPLAY_PERIOD_MS)
            {
                u8g2_FirstPage(&u8g2);
                do
                {
                  draw_t12_ui(&u8g2);
                } while (u8g2_NextPage(&u8g2));
                
                last_display_time = current_time; // ������Ļˢ��ʱ��
            }
            t12_state = STATE_HEATING; // �л��ؼ��ȵȴ�״̬����ʼ��һ������
            break;

        default:
            // ����״̬����
            t12_state = STATE_HEATING;
            break;
    }
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

uint16_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
  // ���ӱ߽��飬��ֹ������
  if (in_max == in_min) {
    return out_min;
  }
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint16_t calculateTemp(uint16_t RawTemp)
{
    // ���ADCֵ���ڵ�һ��У׼�㣬ֱ��ӳ��
    if (RawTemp <= temp_calibration_table[1].adc_val) {
        return map(RawTemp,
                   temp_calibration_table[0].adc_val,
                   temp_calibration_table[1].adc_val,
                   temp_calibration_table[0].temp_c,
                   temp_calibration_table[1].temp_c);
    }

    // ����У׼���ҵ����ʵ�����������Բ�ֵ
    for (size_t i = 1; i < NUM_CALIBRATION_POINTS - 1; ++i) {
        if (RawTemp <= temp_calibration_table[i+1].adc_val) {
            return map(RawTemp,
                       temp_calibration_table[i].adc_val,
                       temp_calibration_table[i+1].adc_val,
                       temp_calibration_table[i].temp_c,
                       temp_calibration_table[i+1].temp_c);
        }
    }

    // ����������з�Χ����ʹ�����������������
    return map(RawTemp,
               temp_calibration_table[NUM_CALIBRATION_POINTS-2].adc_val,
               temp_calibration_table[NUM_CALIBRATION_POINTS-1].adc_val,
               temp_calibration_table[NUM_CALIBRATION_POINTS-2].temp_c,
               temp_calibration_table[NUM_CALIBRATION_POINTS-1].temp_c);
}

uint16_t calculate_pid_pwm(uint16_t target, uint16_t current)
{
  int16_t error = (int16_t)target - (int16_t)current;
  int16_t output;
  output = arm_pid_f32(&PID, error);
  if (output > PID_INTEGRAL_MAX) {
    output = PID_INTEGRAL_MAX;
  } else if (output < PID_INTEGRAL_MIN) {
    output = PID_INTEGRAL_MIN;
  }
  return (uint16_t)output;
}

/**
 * @brief ��ȡ�����������仯ֵ������
 * @return �����������仯ֵ��>0 Ϊ��ת��<0 Ϊ��ת��=0 Ϊδ����
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
    rotaryCount = 0; // �������ֵ
    return step;
}

/**
 * @brief ����T12������������UI���� (���� 128x64 �ֱ���)
 */
void draw_t12_ui(u8g2_t *u8g2)
{
    u8g2_SetFontMode(u8g2, 1);
    u8g2_SetFontDirection(u8g2, 0);
    u8g2_SetFont(u8g2, u8g2_font_9x15_tr);
    sprintf(sprintf_tmp, "%d %d", Vref_Read(), Vin_Read());
    u8g2_DrawStr(u8g2, 0, 10, sprintf_tmp);
    sprintf(sprintf_tmp, "tmp:%d C", Temp_ADC_Read());
    u8g2_DrawStr(u8g2, 0, 22, sprintf_tmp);
    sprintf(sprintf_tmp, "%3d,%3d,%d%%", temp_target, t12_now, t12_pwm/20);
    u8g2_DrawStr(u8g2, 0, 34, sprintf_tmp);
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
