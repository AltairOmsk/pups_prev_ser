/*
* Copyright (C) 2018 Yuri Ryzhenko <Y.Ryzhenko@hi-tech.org>, Aleksey Kirsanov <a.kirsanov@iva-tech.ru>
* All rights reserved
*
* File Name  : core.c
* Description: DSP processing
*/
//******************************************************************************
// Секция include: здесь подключается заголовочный файл к модулю
//******************************************************************************
#include "core.h" // Включаем файл заголовка для нашего модуля
//******************************************************************************
// Секция определения переменных, используемых в модуле
//******************************************************************************
//------------------------------------------------------------------------------
// Глобальные
//------------------------------------------------------------------------------
//char GlobalVar1;
//char GlobalVar2;
//...
//------------------------------------------------------------------------------
// Локальные
//------------------------------------------------------------------------------
static void TIM_DMAPeriodElapsedCplt(DMA_HandleTypeDef *hdma);
static void TIM_DMATriggerCplt(DMA_HandleTypeDef *hdma);
//******************************************************************************
// Секция прототипов локальных функций
//******************************************************************************
//void local_func1 (void);
//void local_func2 (void);
//...
//******************************************************************************
// Секция описания функций (сначала глобальных, потом локальных)
//******************************************************************************
/**
  * @brief  Configure the DMA Burst to transfer Data from the memory to the TIM peripheral  
  * @param  htim pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  BurstBaseAddress TIM Base address from when the DMA will starts the Data write.
  *         This parameters can be on of the following values:
  *            @arg TIM_DMABASE_CR1  
  *            @arg TIM_DMABASE_CR2
  *            @arg TIM_DMABASE_SMCR
  *            @arg TIM_DMABASE_DIER
  *            @arg TIM_DMABASE_SR
  *            @arg TIM_DMABASE_EGR
  *            @arg TIM_DMABASE_CCMR1
  *            @arg TIM_DMABASE_CCMR2
  *            @arg TIM_DMABASE_CCER
  *            @arg TIM_DMABASE_CNT   
  *            @arg TIM_DMABASE_PSC   
  *            @arg TIM_DMABASE_ARR
  *            @arg TIM_DMABASE_RCR
  *            @arg TIM_DMABASE_CCR1
  *            @arg TIM_DMABASE_CCR2
  *            @arg TIM_DMABASE_CCR3  
  *            @arg TIM_DMABASE_CCR4
  *            @arg TIM_DMABASE_BDTR
  *            @arg TIM_DMABASE_DCR
  * @param  BurstRequestSrc TIM DMA Request sources.
  *         This parameters can be on of the following values:
  *            @arg TIM_DMA_UPDATE: TIM update Interrupt source
  *            @arg TIM_DMA_CC1: TIM Capture Compare 1 DMA source
  *            @arg TIM_DMA_CC2: TIM Capture Compare 2 DMA source
  *            @arg TIM_DMA_CC3: TIM Capture Compare 3 DMA source
  *            @arg TIM_DMA_CC4: TIM Capture Compare 4 DMA source
  *            @arg TIM_DMA_COM: TIM Commutation DMA source
  *            @arg TIM_DMA_TRIGGER: TIM Trigger DMA source
  * @param  BurstBuffer The Buffer address.
  * @param  BurstLength DMA Burst length. This parameter can be one value
  *         between TIM_DMABURSTLENGTH_1TRANSFER and TIM_DMABURSTLENGTH_18TRANSFERS.
  * @retval HAL status
  */
HAL_StatusTypeDef My_HAL_TIM_DMABurst_WriteStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress, uint32_t BurstRequestSrc,
                                              uint32_t* BurstBuffer, uint32_t  BurstLength, uint32_t Size_All_DMA_transfer)
{
  /* Check the parameters */
  assert_param(IS_TIM_DMABURST_INSTANCE(htim->Instance));
  assert_param(IS_TIM_DMA_BASE(BurstBaseAddress));
  assert_param(IS_TIM_DMA_SOURCE(BurstRequestSrc));
  assert_param(IS_TIM_DMA_LENGTH(BurstLength));
  
  if((htim->State == HAL_TIM_STATE_BUSY))
  {
     return HAL_BUSY;
  }
  else if((htim->State == HAL_TIM_STATE_READY))
  {
    if((BurstBuffer == 0U) && (BurstLength > 0U)) 
    {
      return HAL_ERROR;                                    
    }
    else
    {
      htim->State = HAL_TIM_STATE_BUSY;
    }
  }
  switch(BurstRequestSrc)
  {
    case TIM_DMA_UPDATE:
    {  
      /* Set the DMA Period elapsed callback */
      htim->hdma[TIM_DMA_ID_UPDATE]->XferCpltCallback = TIM_DMAPeriodElapsedCplt;
     
      /* Set the DMA error callback */
      htim->hdma[TIM_DMA_ID_UPDATE]->XferErrorCallback = TIM_DMAError ;
  
      /* Enable the DMA Stream */
      //HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_UPDATE], (uint32_t)BurstBuffer, (uint32_t)&htim->Instance->DMAR, ((BurstLength) >> 8U) + 1U); 
      HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_UPDATE], (uint32_t)BurstBuffer, (uint32_t)&htim->Instance->DMAR, Size_All_DMA_transfer); 
    }
    break;
    case TIM_DMA_CC1:
    {  
      /* Set the DMA Period elapsed callback */
      htim->hdma[TIM_DMA_ID_CC1]->XferCpltCallback = TIM_DMADelayPulseCplt;
     
      /* Set the DMA error callback */
      htim->hdma[TIM_DMA_ID_CC1]->XferErrorCallback = TIM_DMAError ;
  
      /* Enable the DMA Stream */
      HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC1], (uint32_t)BurstBuffer, (uint32_t)&htim->Instance->DMAR, ((BurstLength) >> 8U) + 1U);     
    }
    break;
    case TIM_DMA_CC2:
    {  
      /* Set the DMA Period elapsed callback */
      htim->hdma[TIM_DMA_ID_CC2]->XferCpltCallback = TIM_DMADelayPulseCplt;
     
      /* Set the DMA error callback */
      htim->hdma[TIM_DMA_ID_CC2]->XferErrorCallback = TIM_DMAError ;
  
      /* Enable the DMA Stream */
      HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC2], (uint32_t)BurstBuffer, (uint32_t)&htim->Instance->DMAR, ((BurstLength) >> 8U) + 1U);     
    }
    break;
    case TIM_DMA_CC3:
    {  
      /* Set the DMA Period elapsed callback */
      htim->hdma[TIM_DMA_ID_CC3]->XferCpltCallback = TIM_DMADelayPulseCplt;
     
      /* Set the DMA error callback */
      htim->hdma[TIM_DMA_ID_CC3]->XferErrorCallback = TIM_DMAError ;
  
      /* Enable the DMA Stream */
      HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC3], (uint32_t)BurstBuffer, (uint32_t)&htim->Instance->DMAR, ((BurstLength) >> 8U) + 1U);     
    }
    break;
    case TIM_DMA_CC4:
    {  
      /* Set the DMA Period elapsed callback */
      htim->hdma[TIM_DMA_ID_CC4]->XferCpltCallback = TIM_DMADelayPulseCplt;
     
      /* Set the DMA error callback */
      htim->hdma[TIM_DMA_ID_CC4]->XferErrorCallback = TIM_DMAError ;
  
      /* Enable the DMA Stream */
      HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC4], (uint32_t)BurstBuffer, (uint32_t)&htim->Instance->DMAR, ((BurstLength) >> 8U) + 1U);     
    }
    break;
    case TIM_DMA_COM:
    {  
      /* Set the DMA Period elapsed callback */
      htim->hdma[TIM_DMA_ID_COMMUTATION]->XferCpltCallback = TIMEx_DMACommutationCplt;
     
      /* Set the DMA error callback */
      htim->hdma[TIM_DMA_ID_COMMUTATION]->XferErrorCallback = TIM_DMAError ;
  
      /* Enable the DMA Stream */
      HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_COMMUTATION], (uint32_t)BurstBuffer, (uint32_t)&htim->Instance->DMAR, ((BurstLength) >> 8U) + 1U);     
    }
    break;
    case TIM_DMA_TRIGGER:
    {  
      /* Set the DMA Period elapsed callback */
      htim->hdma[TIM_DMA_ID_TRIGGER]->XferCpltCallback = TIM_DMATriggerCplt;
     
      /* Set the DMA error callback */
      htim->hdma[TIM_DMA_ID_TRIGGER]->XferErrorCallback = TIM_DMAError ;
  
      /* Enable the DMA Stream */
      HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_TRIGGER], (uint32_t)BurstBuffer, (uint32_t)&htim->Instance->DMAR, ((BurstLength) >> 8U) + 1U);     
    }
    break;
    default:
    break;  
  }
   /* configure the DMA Burst Mode */
   htim->Instance->DCR = BurstBaseAddress | BurstLength;  
   
   /* Enable the TIM DMA Request */
   __HAL_TIM_ENABLE_DMA(htim, BurstRequestSrc);  
   
   htim->State = HAL_TIM_STATE_READY;
  
  /* Return function status */
  return HAL_OK;
}


/**
  * @brief  TIM DMA Period Elapse complete callback. 
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void TIM_DMAPeriodElapsedCplt(DMA_HandleTypeDef *hdma)
{
  TIM_HandleTypeDef* htim = ( TIM_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  
  htim->State= HAL_TIM_STATE_READY;
  
  HAL_TIM_PeriodElapsedCallback(htim);
}

/**
  * @brief  TIM DMA Trigger callback. 
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void TIM_DMATriggerCplt(DMA_HandleTypeDef *hdma)
{
  TIM_HandleTypeDef* htim = ( TIM_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;  
  
  htim->State= HAL_TIM_STATE_READY; 
  
  HAL_TIM_TriggerCallback(htim);
}



/**
  * @brief  Starts the PWM signal generation.
  * @param  htim pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
HAL_StatusTypeDef My_HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel)
{
  /* Check the parameters */
  assert_param(IS_TIM_CCX_INSTANCE(htim->Instance, Channel));

  /* Enable the Capture compare channel */
  TIM_CCxChannelCmd(htim->Instance, Channel, TIM_CCx_ENABLE);
  
  if(IS_TIM_ADVANCED_INSTANCE(htim->Instance) != RESET)  
  {
    /* Enable the main output */
    __HAL_TIM_MOE_ENABLE(htim);
  }
    
  /* Enable the Peripheral */
  //__HAL_TIM_ENABLE(htim);
  
  /* Return function status */
  return HAL_OK;
} 



//******************************************************************************
//   ФВЧ для удаления постоянки
//******************************************************************************
int16_t HP_Filter (int16_t In) {
static int32_t Prev_in;
static int32_t  Prev_out;
        //DAC->DHR12R1 = input >> 2;
  Prev_out = In + ((Prev_out * 124) >> 7) - Prev_in;
  Prev_in =  In;
        //DAC->DHR12R1 = 2047 + prev_out;
  
  return Prev_out << 0;
}

//******************************************************************************
//   ФВЧ для удаления постоянки
//******************************************************************************
int16_t HP_Filter_GA (int16_t In) {
static int32_t Prev_in;
static int32_t  Prev_out;
        //DAC->DHR12R1 = input >> 2;
  Prev_out = In + ((Prev_out * 124) >> 7) - Prev_in;
  Prev_in =  In;
        //DAC->DHR12R1 = 2047 + prev_out;
  
  return Prev_out << 0;
}

//******************************************************************************
//   Сканирование кнопки на плате
//******************************************************************************
/*
Вызываем функцию каждые 1 мс
В каждом вызове проверяем ногу порта, если кнопка нажата, то считаем длительность нажатия

*/
uint8_t scan_button (void){
static uint32_t Cnt; 
uint8_t Out = 0;

  if (HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin) == GPIO_PIN_RESET){              // Если кнопка нажата
    if (Cnt < 10000) Cnt++;
    
  }
  else {
    if ((Cnt >= 10)   && (Cnt < 2000))    Out = 1;    
    if ((Cnt >= 2000) && (Cnt < 5000))    Out = 2;
    if  (Cnt >= 5000)                     Out = 3;
    Cnt = 0;
  }
  return Out;
}  
//******************************************************************************




//calc checksum (1~6 byte)
uint16_t get_checksum (uint8_t *thebuf) {
	uint16_t sum = 0;
	for (int i=1; i<7; i++) {
		sum += thebuf[i];
	}
	return -sum;
}


//******************************************************************************
//   Установить громкость воспроизведения плеера
//******************************************************************************
void pleer_set_vol (uint8_t Vol){
uint8_t   Str[10];  
uint8_t   VER  = 0xFF;
uint8_t   Len  = 0x06;
uint8_t   Cmd  = 0x06;
uint8_t   FB   = 0x00;
uint8_t   Par1 = 0x00;
uint8_t   Par2 = Vol;
uint16_t  ChS  = 0x00;

Str[0] = 0x7E;
Str[1] = VER;
Str[2] = Len;
Str[3] = Cmd;
Str[4] = FB;
Str[5] = Par1;
Str[6] = Par2;
ChS    = get_checksum(Str);
Str[7] = (ChS >> 8);
Str[8] = (ChS & 0xFF);
Str[9] = 0xEF;




HAL_UART_Transmit(&huart1, (uint8_t*)Str, 10, 10);


//char Str[] = {0x7E, 0xFF, 0x06, 0x06, 0x00, 0x00, 0x0F, 0xFF, 0xD5, 0xEF};    // specify the volume to 15
  
}


//******************************************************************************
//   Включить режим непрерывного повторения
//******************************************************************************
void pleer_set_repit (void){
uint8_t   Str[10];  
uint8_t   VER  = 0xFF;
uint8_t   Len  = 0x06;
uint8_t   Cmd  = 0x11;
uint8_t   FB   = 0x00;
uint8_t   Par1 = 0x00;
uint8_t   Par2 = 0x01;
uint16_t  ChS  = 0x00;

Str[0] = 0x7E;
Str[1] = VER;
Str[2] = Len;
Str[3] = Cmd;
Str[4] = FB;
Str[5] = Par1;
Str[6] = Par2;
ChS    = get_checksum(Str);
Str[7] = (ChS >> 8);
Str[8] = (ChS & 0xFF);
Str[9] = 0xEF;


HAL_UART_Transmit(&huart1, (uint8_t*)Str, 10, 10); 
}

//******************************************************************************
//   Включить воспроизведение
//******************************************************************************
void pleer_play (void){
uint8_t   Str[10];  
uint8_t   VER  = 0xFF;
uint8_t   Len  = 0x06;
uint8_t   Cmd  = 0x03;
uint8_t   FB   = 0x00;
uint8_t   Par1 = 0x00;
uint8_t   Par2 = 0x01;
uint16_t  ChS  = 0x00;

Str[0] = 0x7E;
Str[1] = VER;
Str[2] = Len;
Str[3] = Cmd;
Str[4] = FB;
Str[5] = Par1;
Str[6] = Par2;
ChS    = get_checksum(Str);
Str[7] = (ChS >> 8);
Str[8] = (ChS & 0xFF);
Str[9] = 0xEF;


HAL_UART_Transmit(&huart1, (uint8_t*)Str, 10, 10); 
}




//******************************************************************************
// ENF OF FILE
//******************************************************************************