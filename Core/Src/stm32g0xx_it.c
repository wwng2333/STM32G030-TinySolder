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
    int32_t step = 0; // 步长，用于增或减

    // 检查是否是 EXTI_LINE_14 (A相) 触发的中断
    if (LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_14) != RESET || 
        LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_14) != RESET)
    {
        // 1. 禁用中断以防止在处理期间的重复触发 (软去抖)
        LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_14);

        // 2. 读取当前的 A 和 B 信号电平
        // 注意：由于中断是由 A 相的边沿触发的，这里的 A 相电平是边沿**后的**电平
        a_value = LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_14);
        b_value = LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_15);

        // 3. **核心逻辑：使用 Quadrature 编码器标准判断方向**
        // 标准 A/B 编码器逻辑：
        // 顺时针 (CW): A 边沿触发时，B 超前 (例如：A 上升沿时 B 为低电平，或 A 下降沿时 B 为高电平)
        // 逆时针 (CCW): A 边沿触发时，B 滞后 (例如：A 上升沿时 B 为高电平，或 A 下降沿时 B 为低电平)

        // 我们在 A 相的边沿处读取 B 相的电平：
        if (a_value == b_value) // A 和 B 电平相同 (例如 A=1/B=1 或 A=0/B=0)
        {
            // 顺时针 (CW) 旋转，A 滞后于 B 90度
            // 当 A 变为高电平 (上升沿) 时，B 应该为高电平。
            // 当 A 变为低电平 (下降沿) 时，B 应该为低电平。
            // 由于 A 的状态是边沿后的状态，所以 a_value == b_value 时是顺时针（或逆时针，取决于接线）
            rotaryCount = -5;
        }
        else // A 和 B 电平不同 (例如 A=1/B=0 或 A=0/B=1)
        {
            // 逆时针 (CCW) 旋转
            rotaryCount = 5;
        }

        // 4. 更新计数并设置移动标志
        //count = constrain(count + step, countMin, countMax);
        //handleMoved = true;
        
        // 5. 重新启用中断
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
    // 如果有其他引脚 (EXTI4~15) 的中断，应在此处添加对应的标志清除和处理逻辑
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
