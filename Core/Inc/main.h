/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "stm32g0xx_ll_adc.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_exti.h"
#include "stm32g0xx_ll_cortex.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_pwr.h"
#include "stm32g0xx_ll_dma.h"
#include "stm32g0xx_ll_spi.h"
#include "stm32g0xx_ll_tim.h"
#include "stm32g0xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "u8g2.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

// PID
typedef struct {
    float Kp, Ki, Kd;
    float integral_sum;
    int16_t last_error;
	  int16_t last_measurement;
} PID_Controller;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
uint16_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
uint16_t calculateTemp(uint16_t RawTemp);
uint16_t calculate_pid_pwm(PID_Controller *pid, uint16_t target, uint16_t current);

int16_t Get_Rotary_Step_And_Clear(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENC_SW_Pin LL_GPIO_PIN_0
#define ENC_SW_GPIO_Port GPIOA
#define BUZZER_Pin LL_GPIO_PIN_4
#define BUZZER_GPIO_Port GPIOA
#define HW_SCL_Pin LL_GPIO_PIN_5
#define HW_SCL_GPIO_Port GPIOA
#define DISPLAY_RST_Pin LL_GPIO_PIN_6
#define DISPLAY_RST_GPIO_Port GPIOA
#define HW_SDA_Pin LL_GPIO_PIN_7
#define HW_SDA_GPIO_Port GPIOA
#define DISPLAY_DC_Pin LL_GPIO_PIN_0
#define DISPLAY_DC_GPIO_Port GPIOB
#define DISPLAY_CS_Pin LL_GPIO_PIN_1
#define DISPLAY_CS_GPIO_Port GPIOB
#define TIP_Pin LL_GPIO_PIN_2
#define TIP_GPIO_Port GPIOB
#define PWM_Pin LL_GPIO_PIN_6
#define PWM_GPIO_Port GPIOC
#define WAKE_Pin LL_GPIO_PIN_11
#define WAKE_GPIO_Port GPIOA
#define VIN_Pin LL_GPIO_PIN_7
#define VIN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
