/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g0xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32g0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */
extern uint32_t uWTick;
extern volatile int16_t rotaryCount; 
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/

/******************************************************************************/
/* STM32G0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 4 to 15 interrupts.
  */
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */
    uint8_t a_value, b_value;
    int32_t step = 0; // ���������������

    // ����Ƿ��� EXTI_LINE_14 (A��) �������ж�
    if (LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_14) != RESET || 
        LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_14) != RESET)
    {
        // 1. �����ж��Է�ֹ�ڴ����ڼ���ظ����� (��ȥ��)
        LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_14);

        // 2. ��ȡ��ǰ�� A �� B �źŵ�ƽ
        // ע�⣺�����ж����� A ��ı��ش����ģ������ A ���ƽ�Ǳ���**���**��ƽ
        a_value = LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_14);
        b_value = LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_15);

        // 3. **�����߼���ʹ�� Quadrature ��������׼�жϷ���**
        // ��׼ A/B �������߼���
        // ˳ʱ�� (CW): A ���ش���ʱ��B ��ǰ (���磺A ������ʱ B Ϊ�͵�ƽ���� A �½���ʱ B Ϊ�ߵ�ƽ)
        // ��ʱ�� (CCW): A ���ش���ʱ��B �ͺ� (���磺A ������ʱ B Ϊ�ߵ�ƽ���� A �½���ʱ B Ϊ�͵�ƽ)

        // ������ A ��ı��ش���ȡ B ��ĵ�ƽ��
        if (a_value == b_value) // A �� B ��ƽ��ͬ (���� A=1/B=1 �� A=0/B=0)
        {
            // ˳ʱ�� (CW) ��ת��A �ͺ��� B 90��
            // �� A ��Ϊ�ߵ�ƽ (������) ʱ��B Ӧ��Ϊ�ߵ�ƽ��
            // �� A ��Ϊ�͵�ƽ (�½���) ʱ��B Ӧ��Ϊ�͵�ƽ��
            // ���� A ��״̬�Ǳ��غ��״̬������ a_value == b_value ʱ��˳ʱ�루����ʱ�룬ȡ���ڽ��ߣ�
            rotaryCount = -5;
        }
        else // A �� B ��ƽ��ͬ (���� A=1/B=0 �� A=0/B=1)
        {
            // ��ʱ�� (CCW) ��ת
            rotaryCount = 5;
        }

        // 4. ���¼����������ƶ���־
        //count = constrain(count + step, countMin, countMax);
        //handleMoved = true;
        
        // 5. ���������ж�
        LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_14);
    }
    
  /* USER CODE END EXTI4_15_IRQn 0 */
  if (LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_14) != RESET)
  {
    LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_14);
    /* USER CODE BEGIN LL_EXTI_LINE_14_RISING */
    /* USER CODE END LL_EXTI_LINE_14_RISING */
  }
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */
    // ������������� (EXTI4~15) ���жϣ�Ӧ�ڴ˴���Ӷ�Ӧ�ı�־����ʹ����߼�
  /* USER CODE END EXTI4_15_IRQn 1 */
}

/**
  * @brief This function handles TIM14 global interrupt.
  */
void TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM14_IRQn 0 */
	uWTick++;
	LL_TIM_ClearFlag_CC1(TIM14);
  /* USER CODE END TIM14_IRQn 0 */
  /* USER CODE BEGIN TIM14_IRQn 1 */

  /* USER CODE END TIM14_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
