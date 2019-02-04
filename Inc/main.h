/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_3
#define LED2_GPIO_Port GPIOE
#define PWR_ON_LOCK_Pin GPIO_PIN_4
#define PWR_ON_LOCK_GPIO_Port GPIOE
#define PWR_BTN_Pin GPIO_PIN_5
#define PWR_BTN_GPIO_Port GPIOE
#define OUT_CURRENT_Pin GPIO_PIN_0
#define OUT_CURRENT_GPIO_Port GPIOA
#define PWR_VOLT_Pin GPIO_PIN_1
#define PWR_VOLT_GPIO_Port GPIOA
#define DAC_OUT2_Pin GPIO_PIN_5
#define DAC_OUT2_GPIO_Port GPIOA
#define TX_PA_MUTE_Pin GPIO_PIN_0
#define TX_PA_MUTE_GPIO_Port GPIOB
#define RX_PA_SHDN_Pin GPIO_PIN_1
#define RX_PA_SHDN_GPIO_Port GPIOB
#define REL_RX_OFF_Pin GPIO_PIN_8
#define REL_RX_OFF_GPIO_Port GPIOE
#define REL_RX_SHORT_Pin GPIO_PIN_9
#define REL_RX_SHORT_GPIO_Port GPIOE
#define REL_RX_TUNE0_Pin GPIO_PIN_10
#define REL_RX_TUNE0_GPIO_Port GPIOE
#define REL_RX_TUNE1_Pin GPIO_PIN_11
#define REL_RX_TUNE1_GPIO_Port GPIOE
#define REL_TX_ON_Pin GPIO_PIN_12
#define REL_TX_ON_GPIO_Port GPIOE
#define REL_TX_TUNE0_Pin GPIO_PIN_13
#define REL_TX_TUNE0_GPIO_Port GPIOE
#define REL_TX_TUNE1_Pin GPIO_PIN_14
#define REL_TX_TUNE1_GPIO_Port GPIOE
#define CODEC_RESET_Pin GPIO_PIN_15
#define CODEC_RESET_GPIO_Port GPIOE
#define I2S2_BCLK_Pin GPIO_PIN_10
#define I2S2_BCLK_GPIO_Port GPIOB
#define PTT1_Pin GPIO_PIN_11
#define PTT1_GPIO_Port GPIOB
#define I2S2_WCLK_Pin GPIO_PIN_12
#define I2S2_WCLK_GPIO_Port GPIOB
#define PTT2_Pin GPIO_PIN_13
#define PTT2_GPIO_Port GPIOB
#define I2S2_DOUT_Pin GPIO_PIN_14
#define I2S2_DOUT_GPIO_Port GPIOB
#define I2S2_DIN_Pin GPIO_PIN_15
#define I2S2_DIN_GPIO_Port GPIOB
#define I2S2_MCLK_Pin GPIO_PIN_6
#define I2S2_MCLK_GPIO_Port GPIOC
#define UHF_SPK_ON_Pin GPIO_PIN_7
#define UHF_SPK_ON_GPIO_Port GPIOD
#define UHF_HILO_Pin GPIO_PIN_3
#define UHF_HILO_GPIO_Port GPIOB
#define UHF_SHDN_Pin GPIO_PIN_4
#define UHF_SHDN_GPIO_Port GPIOB
#define UHF_PTT_Pin GPIO_PIN_5
#define UHF_PTT_GPIO_Port GPIOB
#define BTN_Pin GPIO_PIN_1
#define BTN_GPIO_Port GPIOE

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
typedef    __packed struct 
{
    unsigned short    b15_0;
    unsigned short    b16_31;
    unsigned int    b32_63;
    unsigned int    b64_95;
} UNIQUE_ID;

#define __LED1_ON       HAL_GPIO_WritePin (LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET)
#define __LED1_OFF      HAL_GPIO_WritePin (LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET)
#define __LED1_SW       HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin)

#define __LED2_ON       HAL_GPIO_WritePin (LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET)
#define __LED2_OFF      HAL_GPIO_WritePin (LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET)
#define __LED2_SW       HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin)
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
