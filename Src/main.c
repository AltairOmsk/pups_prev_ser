
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */
#include "core.h"
#include "codec.h"
#include "trx.h"
#include "com_control.h"    
#include "string.h"
#include "time.h"
#include "squelch.h"


#define MACRO ARM_MATH_CM4 
#define _FPU_PRESENT  1

/*
Это софт для предсерийного ПУПС на новой плате от 12.10.2018, начиная с версии 2Б.

Для того, что бы срабатывал колбэк для прерывания полного завершения передачи по ДМА
В файле stm32f4xx_hal_i2s_ex.c
В функции static void I2SEx_TxRxDMACplt(DMA_HandleTypeDef *hdma) добавить
вызов HAL_I2SEx_TxRxCpltCallback(hi2s);


при работе с АЦП2 ограничение сверху и снизу наступает при установке на генераторе:
Канал В, амплитуда 1,5в, смещение 830 мВ
  
  
Проблемы с sd картой решены по алгоритму из этой ссылки  
https://community.st.com/thread/46310-solution-to-fix-stm32f4xxx-stmcubemx-firmware-v118-sdio-fatfs-dma-not-working-issue


===   Время   ===
Закомментировать в инициализации RTC блок "Initialize RTC and set the Time and Date", что бы каждый запуск время не стиралось


===   Запись на sd карту   ===
В инициализации SDIO делитель с 118 заменить на 30.


===   Попугай   ===
Уменьшен буфер записи для опытов с FFT


===   FFT   ===
Добавить в пути препроцессора такой путь к папке Lib
C:\Program Files (x86)\IAR Systems\Embedded Workbench 6.5\arm\CMSIS\Lib
  
*/

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_i2s2_ext_rx;
DMA_HandleTypeDef hdma_spi2_tx;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_tx;

DMA_HandleTypeDef hdma_memtomem_dma2_stream1;
DMA_HandleTypeDef hdma_memtomem_dma2_stream2;
DMA_HandleTypeDef hdma_memtomem_dma2_stream4;
osThreadId defaultTaskHandle;
osThreadId UHF_taskHandle;
osThreadId USB_taskHandle;
osThreadId MainRadio_taskHandle;
osSemaphoreId myBinSem_DMA_ReadyHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
REG_t R;                                                                        // Реестр
UNIQUE_ID DevID;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S2_Init(void);
static void MX_RTC_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);
void Start_UHF_task(void const * argument);
void Start_USB_task(void const * argument);
void Start_MainRadio_task(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//******************************************************************************
//   Мигание светодиода 1 с периодом в мс
//******************************************************************************
static void led1_blink (void);                                                 
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static void led1_blink (void){                                          // Мигание светодиодом для теста работы

  __LED1_SW;
  R.UpTime++;

  HAL_RTC_GetDate(&hrtc, &(R.GDate), RTC_FORMAT_BIN);                           // Актуализируем дату/время
  osDelay(1);
  HAL_RTC_GetTime(&hrtc, &(R.GTime), RTC_FORMAT_BIN);
  osDelay(1);
  

//--- Тестовая отправка S метра в СОМ порт
  if ((R.TXRX_Mode == RX) && (R.RC.S_meterIntrval)) { S_meter_Send(); };

  
}


//******************************************************************************
//   Запись строки лога на флэшкарту с заданным периодом
//******************************************************************************
static void log_1s_write (void){                                        


uint8_t Header[]="Dev ID \t Up Time \t RX Freq \t TX Freq \t RxTx Mode \t Rx Gain \t S uV \t S ADC \t TX Current \t Main PWR mV \t CPU temperature \r\n";

uint8_t Buf[256];        
uint8_t Buf2[256];
UINT Bytes;
FRESULT res;



  HAL_GetUID((uint32_t*)&DevID);
  //printf("b0-15=%d, b16-32=%d, b32-64=%d, b64-96=%d\r\n", DevID.b15_0, DevID.b16_31, DevID.b32_63, DevID.b64_95);

  
  //----------------------------------------------------------------------------   Создать строку для записи
  if ((R.TXRX_Mode == TX) || (R.TXRX_Mode == TX_TONE_1kHz)){                    // Что бы в режиме передачи приемный уровень не смущал
      R.S_meter_mkv = 0;
      R.S_meter = 0;
  }
  
  sprintf(Buf,"%d%d \t %d.%02d.20%02d \t %d:%02d:%02d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \t %d ",
          DevID.b15_0, DevID.b16_31,                                            // DevID 
          R.GDate.Date,
          R.GDate.Month,
          R.GDate.Year,
          R.GTime.Hours,
          R.GTime.Minutes,
          R.GTime.Seconds,
          R.UpTime,                                                             // Up Time
          R.DDS_RX.Freq,                                                        // Частота приема
          R.DDS_TX.Freq,                                                        // Частота передачи
          R.TXRX_Mode,                                                          // RxTx Mode          
          R.RX_gain,                                                            // Rx Gain
          R.S_meter_mkv,                                                        // S uV
          R.S_meter,                                                            // S ADC
          R.TX_current_Peack,                                                   // TX current
          R.PWR_main_mV,                                                        // Питание
          R.CPU_temperature                                                     // CPU temperature
          );
  
   R.TX_current_Peack = 0;                                                      // Сброс пикового детектора
   
   
   
   sprintf(Buf2,"*\t %d \t %d %d %d %d\t %d %d %d %d \t %d%%  \r\n",
          R.RXBW, 
          R.Gain_MICPGAR,                                                       // Усиления кодека
          R.Gain_ADCR,
          R.Gain_DACR,
          R.Gain_HPR,
          R.Gain_MICPGAL,
          R.Gain_ADCL,
          R.Gain_DACL,
          R.Gain_HPL,
          R.TX_pwr                                                              // Мощность передачи  
          );
             
   
  
            //printf(Buf);
            
    if(f_mount(&SDFatFS, SDPath, 1) == FR_OK){                                  //подключаем флешку

      R.WriteSD = 1;                                                            // Для вывода сообщения в статусе
    
      uint8_t path[12];
      sprintf(path, "log%2d%2d.txt", DevID.b15_0, DevID.b16_31);
      path[11] = '\0';
      
//      res = f_open (&SDFile, (char*)path, FA_READ);                             // Проверяем наличие созданного файла
//      if (res != FR_OK) {
//        res = f_open (&SDFile, (char*)path, FA_WRITE | FA_OPEN_APPEND);         // Если его нет - создаем файл со строкой заголовка столбцов
//        f_write(&SDFile, Header, strlen(Header), &Bytes);
//        printf ("res=%d\r\n", res);
//      }
//      res = f_close(&SDFile);
      
      res = f_open (&SDFile, (char*)path, FA_WRITE | FA_OPEN_APPEND);           // открываем файл для записи в конец файла или если его нет, то создаем
      res = f_write(&SDFile, Buf,  strlen(Buf),  &Bytes);                       // Записать строку лога в конец файла
      res = f_write(&SDFile, Buf2, strlen(Buf2), &Bytes);                       // Записать строку лога в конец файла
      if (R.NewNote){                                                           // Если есть новая заметка в буфере
        res = f_write(&SDFile, R.Note, (char)strlen(R.Note), &Bytes);           // Записать заметку
        R.NewNote = 0;                                                          // Очищаем флаг новой заметки
      };
      res = f_close(&SDFile);
    };

}



void InitWatchDog (void)
 {

    IWDG->KR = 0x5555;   //Открывем доступ к перезагрузке  
    IWDG->PR = (IWDG_PR_PR_0 | IWDG_PR_PR_2);  //101: divider /128
    
    IWDG->KR = 0x5555;   //Открывем доступ к перезагрузке
    IWDG->RLR = 0xFFF;
    
    //Старт WatchDog
    IWDG->KR = 0x5555;   //Открывем доступ к перезагрузке
    IWDG->KR = 0xCCCC;   //Cтартуем 
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  
                            //InitWatchDog();                                                               // Запуск WatchDog
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  HAL_Delay(350);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_RTC_Init();
  MX_SDIO_SD_Init();
  MX_USART1_UART_Init();
  MX_TIM7_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  __PWR_ON_LOCK_ON;
  
//  R.Buf_LPF_8k_I[10] = 1;
//  
//  uint32_t Src[10], Dst[10];
//  for (uint8_t i=0;i<10;i++){
//    Src[i]=0;
//    Dst[i]=0;
//  }
//  
//  Src[5] = 255;
//  
//  while(1){
//      __HAL_DMA_DISABLE(&hdma_memtomem_dma2_stream1);
//      HAL_DMA_Start(&hdma_memtomem_dma2_stream1, (uint32_t)Src+1, (uint32_t)Src, 9);
//      
//      //HAL_Delay(100);
//      __no_operation();
//      __no_operation();
//      __no_operation();
//      __no_operation();
//  
//  }
  
  
  
  
  RC_start_Init();                                                              // Инициализация переменных для дистанционного управления
  squelch_init(&R.RXSQL);                                                       // Инициализация коэффициентов шумоподавителя
  restore_settings();                                                           // Восстановление сохраненных настроек из батарейного домена        

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of myBinSem_DMA_Ready */
  osSemaphoreDef(myBinSem_DMA_Ready);
  myBinSem_DMA_ReadyHandle = osSemaphoreCreate(osSemaphore(myBinSem_DMA_Ready), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 2048);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of UHF_task */
  osThreadDef(UHF_task, Start_UHF_task, osPriorityNormal, 0, 512);
  UHF_taskHandle = osThreadCreate(osThread(UHF_task), NULL);

  /* definition and creation of USB_task */
  osThreadDef(USB_task, Start_USB_task, osPriorityNormal, 0, 512);
  USB_taskHandle = osThreadCreate(osThread(USB_task), NULL);

  /* definition and creation of MainRadio_task */
  osThreadDef(MainRadio_task, Start_MainRadio_task, osPriorityAboveNormal, 0, 512);
  MainRadio_taskHandle = osThreadCreate(osThread(MainRadio_task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S|RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 50;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 5;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_VBAT;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT2 config 
    */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2S2 init function */
static void MX_I2S2_Init(void)
{

  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

//    /**Initialize RTC and set the Time and Date 
//    */
//  sTime.Hours = 0x0;
//  sTime.Minutes = 0x0;
//  sTime.Seconds = 0x0;
//  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
//  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
//  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }
//
//  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
//  sDate.Month = RTC_MONTH_OCTOBER;
//  sDate.Date = 0x25;
//  sDate.Year = 0x18;
//
//  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }

}

/* SDIO init function */
static void MX_SDIO_SD_Init(void)
{

  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 118;

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 0;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_stream1
  *   hdma_memtomem_dma2_stream2
  *   hdma_memtomem_dma2_stream4
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma2_stream1 on DMA2_Stream1 */
  hdma_memtomem_dma2_stream1.Instance = DMA2_Stream1;
  hdma_memtomem_dma2_stream1.Init.Channel = DMA_CHANNEL_0;
  hdma_memtomem_dma2_stream1.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream1.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_stream1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma2_stream1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_memtomem_dma2_stream1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_memtomem_dma2_stream1.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_stream1.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_memtomem_dma2_stream1.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma2_stream1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_memtomem_dma2_stream1.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream1.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /* Configure DMA request hdma_memtomem_dma2_stream2 on DMA2_Stream2 */
  hdma_memtomem_dma2_stream2.Instance = DMA2_Stream2;
  hdma_memtomem_dma2_stream2.Init.Channel = DMA_CHANNEL_0;
  hdma_memtomem_dma2_stream2.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream2.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_stream2.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma2_stream2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_memtomem_dma2_stream2.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_memtomem_dma2_stream2.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_stream2.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_memtomem_dma2_stream2.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma2_stream2.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_memtomem_dma2_stream2.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream2.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /* Configure DMA request hdma_memtomem_dma2_stream4 on DMA2_Stream4 */
  hdma_memtomem_dma2_stream4.Instance = DMA2_Stream4;
  hdma_memtomem_dma2_stream4.Init.Channel = DMA_CHANNEL_0;
  hdma_memtomem_dma2_stream4.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream4.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_stream4.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma2_stream4.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_memtomem_dma2_stream4.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_memtomem_dma2_stream4.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_stream4.Init.Priority = DMA_PRIORITY_LOW;
  hdma_memtomem_dma2_stream4.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma2_stream4.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_memtomem_dma2_stream4.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream4.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED1_Pin|LED2_Pin|PWR_ON_LOCK_Pin|REL_RX_OFF_Pin 
                          |REL_RX_SHORT_Pin|REL_RX_TUNE0_Pin|REL_RX_TUNE1_Pin|REL_TX_ON_Pin 
                          |REL_TX_TUNE0_Pin|REL_TX_TUNE1_Pin|CODEC_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TX_PA_MUTE_Pin|RX_PA_SHDN_Pin|UHF_HILO_Pin|UHF_SHDN_Pin 
                          |UHF_PTT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin PWR_ON_LOCK_Pin REL_RX_OFF_Pin 
                           REL_RX_SHORT_Pin REL_RX_TUNE0_Pin REL_RX_TUNE1_Pin REL_TX_ON_Pin 
                           REL_TX_TUNE0_Pin REL_TX_TUNE1_Pin CODEC_RESET_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|PWR_ON_LOCK_Pin|REL_RX_OFF_Pin 
                          |REL_RX_SHORT_Pin|REL_RX_TUNE0_Pin|REL_RX_TUNE1_Pin|REL_TX_ON_Pin 
                          |REL_TX_TUNE0_Pin|REL_TX_TUNE1_Pin|CODEC_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PWR_BTN_Pin */
  GPIO_InitStruct.Pin = PWR_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PWR_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TX_PA_MUTE_Pin RX_PA_SHDN_Pin UHF_HILO_Pin UHF_SHDN_Pin 
                           UHF_PTT_Pin */
  GPIO_InitStruct.Pin = TX_PA_MUTE_Pin|RX_PA_SHDN_Pin|UHF_HILO_Pin|UHF_SHDN_Pin 
                          |UHF_PTT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PTT1_Pin */
  GPIO_InitStruct.Pin = PTT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PTT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PTT2_Pin */
  GPIO_InitStruct.Pin = PTT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PTT2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UHF_SPK_ON_Pin */
  GPIO_InitStruct.Pin = UHF_SPK_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UHF_SPK_ON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for FATFS */
  MX_FATFS_Init();

  /* USER CODE BEGIN 5 */
  //BSP_SD_Init(); //!!!!!!!!!!!!!!!!!!!!!!!!! Вызывать перед MX_FATFS_Init();

  
 __LED1_OFF;
 __LED2_OFF;

 
 __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);                                   // Прерывание по приходу символа в UART
 
          //HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  CODEC_Init();                                                                 //Настройка кодека
  
  R.Modulation = SSB_USB;
  set_TXRX_mode (RX);

  //---   Запуск приемника   ---------------------------------------------------
      R.DDS_RX.Fdiskr = 48828;
      //R.DDS_RX.Freq = 8000;
      
      HAL_I2SEx_TransmitReceive_DMA(&hi2s2, (uint16_t*)R.CodecTxData, (uint16_t*)R.CodecRxData, CODEC_BUF_SIZE);
      __HAL_DMA_DISABLE_IT(&hdma_spi2_tx, DMA_IT_TC);
      __HAL_DMA_DISABLE_IT(&hdma_spi2_tx, DMA_IT_HT);
      
      //set_RX_Gain_codec();
  //----------------------------------------------------------------------------

  
  //---   Запуск передатчика ---------------------------------------------------
      R.DDS_TX.Fdiskr = 48828UL;
      //R.DDS_TX.Freq = 8000;
      //set_TX_Gain_codec();
      
      /*
      В новом варианте не  требуется периферия контроллера.
      Возможно нужно будет сделать выход MUTE для управления усилителем.
      */
  //----------------------------------------------------------------------------

 
//---   Bluetooth INIT   -------------------------------------------------------
//  sprintf(R.TmpStr, "AT\r\n",DevID.b15_0, DevID.b16_31);
//  HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);  osDelay(100);
//  osDelay(10);
  
  sprintf(R.TmpStr, "AT+ROLE0");      
  HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);  osDelay(100);
  osDelay(200);
  
  HAL_GetUID((uint32_t*)&DevID);
  sprintf(R.TmpStr, "AT+NAMEPUPS_%02d%02d",DevID.b15_0, DevID.b16_31);      
  HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);  osDelay(100);
  osDelay(500);
  
  sprintf(R.TmpStr, "AT+RESET");      
  HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);  osDelay(100);
  osDelay(10);
      

//---   Start DF pleer   -------------------------------------------------------
  
//  pleer_set_vol(30); osDelay(500);
//  pleer_play();      osDelay(500);
//  pleer_set_repit();
  
  //osDelay(1000);// Эта задержка в инициализации Bluetooth
  set_RX_Gain_codec();
  set_TX_Gain_codec();
 

  __PWR_ON_LOCK_ON;                                                             // Блокировка кнопки включения питания
  while (!HAL_GPIO_ReadPin(PWR_BTN_GPIO_Port, PWR_BTN_Pin));                    // Ожидаем отпускание кнопки
  R.PWR_State = 1;                                                              // Для обработки кнопки управления питанием

  
  /* Infinite loop */
  //----------------------------------------------------------------------------
  for(;;)
  {
    
    if (R.Tick_1s == 1){
      R.Tick_1s = 0;
      led1_blink();
      //if (R.FlashLogEn) log_1s_write();                                         // Запись текстового лога
      __LED2_OFF;
    }         
    //if (R.FlashLogEn) record_audio_to_uSD();                                    // Запись аудио лога
  }

  //----------------------------------------------------------------------------
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_Start_UHF_task */
/**
* @brief Function implementing the UHF_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_UHF_task */
void Start_UHF_task(void const * argument)
{
  /* USER CODE BEGIN Start_UHF_task */
  /* Infinite loop */
  for(;;)
  {
    rxtx_switch();

    //USART1_read_command();                                                      // Если принята новая строка с командами
    
    uint8_t Scan = scan_button();
    load_gain_settings (Scan);
    
    adc1_scan();
    
    osDelay(1);
  }
  /* USER CODE END Start_UHF_task */
}

/* USER CODE BEGIN Header_Start_USB_task */
/**
* @brief Function implementing the USB_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_USB_task */
void Start_USB_task(void const * argument)
{
  /* USER CODE BEGIN Start_USB_task */
  /* Infinite loop */
  for(;;)
  {
    USART1_read_command();                                                      // Если принята новая строка с командами
  }
  /* USER CODE END Start_USB_task */
}

/* USER CODE BEGIN Header_Start_MainRadio_task */
/**
* @brief Function implementing the MainRadio_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_MainRadio_task */
void Start_MainRadio_task(void const * argument)
{
  /* USER CODE BEGIN Start_MainRadio_task */
  /* Infinite loop */
  for(;;)
  {
      osSemaphoreWait(myBinSem_DMA_ReadyHandle , osWaitForever);
      main_radio();
  }
  /* USER CODE END Start_MainRadio_task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
