#ifndef _CORE_H // Блокируем повторное включение этого модуля
#define _CORE_H
//******************************************************************************
// Секция include: здесь подключаются заголовочные файлы используемых модулей
//******************************************************************************
#include "stm32f4xx_hal.h"
//******************************************************************************
// Секция определения констант
//******************************************************************************
//#define MY_CONST1 1
//#define MY_CONST2 2
//#define ...
//******************************************************************************
// Секция определения типов
//******************************************************************************
//typedef struct
//{
//...
//} T_STRUCT;
//typedef ...
//******************************************************************************
// Секция определения глобальных переменных
//******************************************************************************
extern UART_HandleTypeDef huart1;
extern RTC_HandleTypeDef hrtc;


//******************************************************************************
// Секция прототипов глобальных функций
//******************************************************************************
HAL_StatusTypeDef My_HAL_TIM_DMABurst_WriteStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress, uint32_t BurstRequestSrc,
                                              uint32_t* BurstBuffer, uint32_t  BurstLength, uint32_t Size_All_DMA_transfer);
HAL_StatusTypeDef My_HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
int16_t HP_Filter (int16_t In);                                                 // ФВЧ для удаления постоянки
int16_t HP_Filter_GA (int16_t In);                                                 // ФВЧ для удаления постоянки
uint8_t scan_button (void);  
uint8_t scan_button_freq (void);

void pleer_set_vol (uint8_t Vol);
void pleer_set_repit (void);
void pleer_play (void);



//******************************************************************************
// Секция определения макросов
//******************************************************************************
//---   RX   -------------------------------------------------------------------
#define __REL_RX_TUNE0_ON        (HAL_GPIO_WritePin(REL_RX_TUNE0_GPIO_Port, REL_RX_TUNE0_Pin, GPIO_PIN_SET))
#define __REL_RX_TUNE0_OFF       (HAL_GPIO_WritePin(REL_RX_TUNE0_GPIO_Port, REL_RX_TUNE0_Pin, GPIO_PIN_RESET))

#define __REL_RX_TUNE1_ON        (HAL_GPIO_WritePin(REL_RX_TUNE1_GPIO_Port, REL_RX_TUNE1_Pin, GPIO_PIN_SET))
#define __REL_RX_TUNE1_OFF       (HAL_GPIO_WritePin(REL_RX_TUNE1_GPIO_Port, REL_RX_TUNE1_Pin, GPIO_PIN_RESET))

#define __REL_RX_SHORT_ON        (HAL_GPIO_WritePin(REL_RX_SHORT_GPIO_Port, REL_RX_SHORT_Pin, GPIO_PIN_SET))
#define __REL_RX_SHORT_OFF       (HAL_GPIO_WritePin(REL_RX_SHORT_GPIO_Port, REL_RX_SHORT_Pin, GPIO_PIN_RESET))

#define __REL_RX_COIL_INVERS     (HAL_GPIO_WritePin(REL_RX_OFF_GPIO_Port, REL_RX_OFF_Pin, GPIO_PIN_SET))
#define __REL_RX_COIL_SERIAL     (HAL_GPIO_WritePin(REL_RX_OFF_GPIO_Port, REL_RX_OFF_Pin, GPIO_PIN_RESET))

#define __RX_PA_ON               (HAL_GPIO_WritePin(RX_PA_SHDN_GPIO_Port, RX_PA_SHDN_Pin, GPIO_PIN_SET))
#define __RX_PA_OFF              (HAL_GPIO_WritePin(RX_PA_SHDN_GPIO_Port, RX_PA_SHDN_Pin, GPIO_PIN_RESET))


//---   TX   -------------------------------------------------------------------
#define __REL_TX_TUNE0_ON        (HAL_GPIO_WritePin(REL_TX_TUNE0_GPIO_Port, REL_TX_TUNE0_Pin, GPIO_PIN_SET))
#define __REL_TX_TUNE0_OFF       (HAL_GPIO_WritePin(REL_TX_TUNE0_GPIO_Port, REL_TX_TUNE0_Pin, GPIO_PIN_RESET))

#define __REL_TX_TUNE1_ON        (HAL_GPIO_WritePin(REL_TX_TUNE1_GPIO_Port, REL_TX_TUNE1_Pin, GPIO_PIN_SET))
#define __REL_TX_TUNE1_OFF       (HAL_GPIO_WritePin(REL_TX_TUNE1_GPIO_Port, REL_TX_TUNE1_Pin, GPIO_PIN_RESET))

#define __REL_TX_ON_ON           (HAL_GPIO_WritePin(REL_TX_ON_GPIO_Port, REL_TX_ON_Pin, GPIO_PIN_SET))
#define __REL_TX_ON_OFF          (HAL_GPIO_WritePin(REL_TX_ON_GPIO_Port, REL_TX_ON_Pin, GPIO_PIN_RESET))

#define __TX_PA_ON               (HAL_GPIO_WritePin(TX_PA_MUTE_GPIO_Port, TX_PA_MUTE_Pin, GPIO_PIN_RESET))
#define __TX_PA_OFF              (HAL_GPIO_WritePin(TX_PA_MUTE_GPIO_Port, TX_PA_MUTE_Pin, GPIO_PIN_SET))


//---   PWR  -------------------------------------------------------------------
#define __PWR_ON_LOCK_ON         (HAL_GPIO_WritePin(PWR_ON_LOCK_GPIO_Port, PWR_ON_LOCK_Pin, GPIO_PIN_SET))
#define __PWR_ON_LOCK_OFF        (HAL_GPIO_WritePin(PWR_ON_LOCK_GPIO_Port, PWR_ON_LOCK_Pin, GPIO_PIN_RESET))




#endif // Закрывающий #endif к блокировке повторного включения
//******************************************************************************
// ENF OF FILE
//******************************************************************************