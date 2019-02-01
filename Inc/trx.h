#ifndef _TRX_H // Блокируем повторное включение этого модуля
#define _TRX_H
//******************************************************************************
// Секция include: здесь подключаются заголовочные файлы используемых модулей
//******************************************************************************
#include "stm32f4xx_hal.h"
#include "codec.h"
#include "dds16.h"
#include "cmsis_os.h"
#include "core.h"
#include "stdlib.h"
#include "string.h"
#include "fatfs.h"
#include "allpass.h"
#include "squelch.h"


//******************************************************************************
// Секция определения констант
//******************************************************************************
#define FW_REV "FW Rev.3.3 24.12.2018"

/*
Rev.3.3 24.12.2018
Обработка управления по уарт выведена в отдельную задачу, запись виснуть перестала от запроса статуса


*/



#define CODEC_BUF_SIZE  384                                     // 2 канала по 96 отсчетов * 2 полубуфера
#define HALF_DMA_SHIFT  (CODEC_BUF_SIZE/2)

#define LEFT_CH         0
#define RIGHT_CH        1

#define ADC_TX_BUF_SIZE (96*2)                                  // 2 полубуфера для приема отсчетов от АЦП микрофона
#define PWM_BUF_SIZE    (ADC_TX_BUF_SIZE*4*4)                   // 2 полубуфера в которых по 4 значения для каналов ОС для каждого такта

#define TX_OUT_AMP      100
#define TX_TONE_AMP     32000                                    // 

#define REC_BUF_SIZE    10 //24500                                   // Размер буфера магнитофона для записи в ОЗУ


//******************************************************************************
// Секция определения типов
//******************************************************************************
typedef enum {                                                                  // Режим работы трансивера
  RX,
  TX,
  TX_TONE_1kHz,
  TEST_SIGNAL_VOICE,
  PAROTT_TEST_SIGNAL,
  PAROTT_REC,
  PAROTT_PLAY,
  FFT,
  FSK100_RX,
  FSK100_TX,
  FSK400_RX,
  FSK400_TX,
  TXRX_MODE_CNT
} TXRX_MODE_e;   

typedef enum {                                                                  // Режим работы трансивера
  SSB_USB,
  //SSB_LSB,
  
} MODULATION_e; 

typedef enum {                                                                  // Режим работы трансивера
  NO_DATA,
  HALF_RX,
  COMPLETE_RX,
} DATA_READY_e;   

typedef enum {                                                                  // Усиление приемника
  RX_GAIN_LOW,
  RX_GAIN_MED,
  RX_GAIN_HIGH,
  RX_GAIN_CNT
}RX_GAIN_t;


typedef enum{                                                                   // Для формирования прерывистого звукового сигнала
  NO_BEEP,
  BEEP_1x1,
  BEEP_1x2,
  BEEP_1x3,
  BEEP_2x1,
  BEEP_2x2,
  BEEP_2x3,
  BEEP_CNT
}BEEP_TYPE_t;

    
typedef struct {                                                                // Значения 4 каналов СС для таймера1, который делает ШИМ передатчика
  uint32_t Ch1;
  uint32_t Ch2;
  uint32_t Ch3;
  uint32_t Ch4;
} CC_CH_SET_t;


typedef struct {                                                                // Контекст буфера УАРТ
  uint8_t  Buf[256];                                                            // Буфер
  uint8_t  Idx;                                                                 // Индекс буфера
  uint8_t  New_string;                                                          // Флаг "Принята новая строка"
} UART_t;


typedef struct {                                                                // Контекст системы АРУ
  uint8_t  Enable;
} AGC_t;

typedef struct {                                                                // Контекст компрессора 
  uint8_t  Enable;
} DRC_t;


typedef struct {                                                                // переменные дистанционного управоения
  uint8_t  S_meterIntrval;                                                      // Разрешение или запрет автоматической выдачи                                           
} RC_t;

typedef enum {                                                                  // Ширина приемного фильтра
  RXBW_NARROW,                                                                  // 900-1100
  RXBW_MEDIUM,                                                                  // 500-1500
  RXBW_WIDE,                                                                    // 350-2900
  RXBW_BYPASS,
  RXBW_CNT  
} RXBW_e;

typedef enum {                                                                  // Состояния автоматической записи-воспроизведения
  OFF,
  START,
  TONE_OUT,
  REC,
  PLAY
} PAROTT_STATE_e;

typedef struct
{
  uint8_t               PWR_State;                                              // 0 - выключено, 1 - включено 
  uint8_t               FlashLogEn;                                             // Разрешение записи лога на uSD карту
  TXRX_MODE_e           TXRX_Mode;                                              // Прием или передача
  MODULATION_e          Modulation;                                             // Вид модуляции
  BEEP_TYPE_t           Beep;                                                   // Сигнализация о включенной частоте и усилении (первая версия, без БТ)
  uint8_t               Tone_Puls;                                              // Включает прерывистый тоновый сигнал если 1
  uint16_t              Test_tone_freq;                                         // Звуковая частота передаваемого тестового сигнала (учесть +/- несущая) 
  uint8_t               SQL_Enable;                                             // Включение шумоподавителя
  uint8_t               WriteSD;                                                // Флаг. Показывает что запись на uSD карту идет
  
  UART_t                U1;                                                     // Буфер УАРТ1 для работы с Bluetooth
  char                  Note[256];                                              // Буфер для записи заметки в лог
  uint8_t               NewNote;                                                // Флаг. Если 1 - есть новая заметка для записи в лог
  RC_t                  RC;                                                     // Переменные дистанционного управления
  RTC_DateTypeDef       GDate;                                                  // Для работы со временем и датой
  RTC_TimeTypeDef       GTime;
  uint32_t              Tick_1s;                                                // Для односкундного таймера
  uint32_t              UpTime;                                                 // Время работы устьройства с момента включения в секундах
  uint32_t              PWR_main_mV;                                            // Напряжение питания устройства в мВ
  uint32_t              CPU_temperature;
  uint16_t              ADC1_buf[5];
  char                  TmpStr[128];                                            // Временная строка для сканов и пр.
  
  int16_t               CodecTxData[CODEC_BUF_SIZE]; 
  int16_t               CodecRxData[CODEC_BUF_SIZE];
  
  uint16_t              PWM_buf [PWM_BUF_SIZE];                                 // Кольцевой буфер из которого выводятся данные ШИМ
  uint16_t              ADC_TX_buf [ADC_TX_BUF_SIZE];                           // В этот буфер принимаем отсчеты от АЦП микрофона
  
  DATA_READY_e          ADC_DataReady;                                          // Флаг. В буфере есть данные готовые к обработке
  
  DDS16_t               DDS_RX;                                                 // Гетеродин несущей первого преобразования приема
  DDS16_t               DDS_TX;                                                 // Гетеродин несущей второго преобразования передачи
  DDS16_t               DDS_GA;                                                 // Гетеродин несущей гидроакустического канала
  DDS16_t               DDS_AUX;                                                // Вспомогательный генератор для генерации всяких тонов
  
  
  //---   SSB приемник  --------------------------------------------------------
  uint8_t               USB_On;                                                 // Флаг. Если ==1, то верхняя боковая при SSB                    
  RXBW_e                RXBW;                                                   // Ширина последнего полосового фильтра
  float                 Buf_LPF_48k_I[72];                                      // Буфер первого фильтра 32 порядка дециматора на 6
  float                 Buf_LPF_48k_Q[72];
  uint8_t               Buf_LPF_48k_idx;                                        // Указатель текущего отсчета
  
  float                 Buf_LPF_8k_I[128];                                      // Фильтр основной фильтрации и поворота.                                
  float                 Buf_LPF_8k_Q[128];
  
  float                 Buf_BPF_8k[128];                                        // Полосовой фильтр
  
  float                 Buf_48k_Out_Work[12];                                   // ФНЧ выходной для интерполятора. Отсчеты входные
  float                 Buf_48k_Out_Work_Q[12];
  float                 Buf_48k_Out[6];                                         // Отсчеты выходные
  float                 Buf_48k_Out_Q[6];
  float                 Buf_48k_Out_Work_Echo[12];                              // Отсчеты выходные для местного воспроизведения
  float                 Buf_48k_Out_Echo[6];
  
  float                 SSB_Out_F;                                              // Для хранения выходного значения
  
  SQUELCH_t             RXSQL;                                                  // Контекст шумоподавителя приемника
  //----------------------------------------------------------------------------
  
  
  
  //---   Гидроакустический канал  ---------------------------------------------   
  uint8_t               GA_USB_On;                                              // Флаг. Если ==1, то верхняя боковая при SSB                    
  RXBW_e                GA_RXBW;                                                // Ширина последнего полосового фильтра

  float                 GA_Buf_LPF_48k_I[72];                                   // Буфер первого фильтра 32 порядка дециматора на 6
  float                 GA_Buf_LPF_48k_Q[72];
  uint8_t               GA_Buf_LPF_48k_idx;                                     // Указатель текущего отсчета
  
  float                 GA_Buf_LPF_8k_I[128];                                   // Фильтр основной фильтрации и поворота.                                
  float                 GA_Buf_LPF_8k_Q[128];
  
  float                 GA_Buf_BPF_8k[128];                                     // Полосовой фильтр
  
  float                 GA_Buf_48k_Out_Work[12];                                // ФНЧ выходной для интерполятора. Отсчеты входные
  float                 GA_Buf_48k_Out_Work_Q[12];
  float                 GA_Buf_48k_Out[6];                                      // Отсчеты выходные
  float                 GA_Buf_48k_Out_Q[6];
  float                 GA_Buf_48k_Out_Work_Echo[12];                           // Отсчеты выходные для местного воспроизведения
  float                 GA_Buf_48k_Out_Echo[6];
  
  float                 GA_SSB_Out_F;                                           // Для хранения выходного значения
  //----------------------------------------------------------------------------


  //---   SSB передатчик  --------------------------------------------------------
  int32_t               TX_pwr;                                                 // Выход передатчика 0-100%
  
  //----------------------------------------------------------------------------
  
  AGC_t AGC;                                                                    // RX AGC SSB
  AGC_t TXAGC;                                                                  // TX AGC Microphone
  AGC_t RXAGC;
  DRC_t TXDRC;
  
  //---   Кодек   --------------------------------------------------------------
  int16_t               Gain_MICPGAL;
  int16_t               Gain_MICPGAR;
  int16_t               Gain_ADCL;
  int16_t               Gain_ADCR;
  int16_t               Gain_DACL;
  int16_t               Gain_DACR;
  int16_t               Gain_HPL;
  int16_t               Gain_HPR;
  //----------------------------------------------------------------------------
  
  
  //---   Эквалайзер   ---------------------------------------------------------
  uint8_t               EqRx_En;
  uint8_t               EqTx_En;
  
  ALLPASS_t             EqTxLow;
  ALLPASS_t             EqTxMed;
  ALLPASS_t             EqTxHigh;
  
  ALLPASS_t             EqRxLow;
  ALLPASS_t             EqRxMed;
  ALLPASS_t             EqRxHigh;
  //----------------------------------------------------------------------------
  
  
  
  //---    Магнитофон   --------------------------------------------------------
  uint32_t              Parott_En;                                              // Разрешение работы попугая
  PAROTT_STATE_e        Parott_State;
  int16_t               RecBuf[REC_BUF_SIZE];                                   // Буфер для записи 3 сек голоса в формате 16 бит.
  uint32_t              RecBufIdx;
  
  int8_t                SD_Rec_buf[8192];                                       // Два буфера по 2048 байт для записи на uSD
  uint16_t              SD_Rec_idx;
  int8_t                SD_Rec_Save;                                            // 0-нечего сохранять, 1-сохранить 0-511, 2-сохранить 512-1023
  //----------------------------------------------------------------------------
  
  
  
  //---   FFT   ----------------------------------------------------------------
//  float                 FFT_buf       [2048];
//  float                 FFT_testOutput[2048/2];
//  uint16_t              FFT_idx;
//  uint8_t               FFT_full;
//  float                 FFT_MaxVal;
//  uint32_t              FFT_MaxValIdx; 
  //----------------------------------------------------------------------------
  
  
  uint32_t              S_meter;
  RX_GAIN_t             RX_gain;
  uint32_t              S_meter_mkv;
  
  uint32_t              TX_current_AVG;                                         // Средний ток в цепи силового моста передатчика
  uint32_t              TX_current_Peack;                                       // Пиковый ток в цепи силового моста передатчика

  
  float Tmp_f;
  float TmpI; 
  float TmpQ;
  int32_t Tmp_i32;

                                 
  

} REG_t;
//******************************************************************************
// Секция определения глобальных переменных
//******************************************************************************
extern const float  Sin_table [4096];
extern REG_t R;                                                                 // Реестр
extern uint32_t Phase;
extern osSemaphoreId myBinSem_DMA_ReadyHandle;
extern I2S_HandleTypeDef hi2s2;
extern DMA_HandleTypeDef hdma_i2s2_ext_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern DMA_HandleTypeDef hdma_adc2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_tim1_up;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern const int8_t test_tone_8bit_pcm[31338];
extern UNIQUE_ID DevID;

//******************************************************************************
// Секция прототипов глобальных функций
//******************************************************************************
void main_radio                         (void);
void adc1_scan                          (void);                                 // Измерение питания и тока передачи
void record_audio_to_uSD                (void);                                 // Запись аудио на SD карту
void adc_mic_half_dma_routine           (void);
void adc_mic_complete_dma_routine       (void);
void set_TXRX_mode                      (TXRX_MODE_e TRX_Mode);
void set_DDS_freq                       (uint32_t Freq);
void off_HW_TX                          (void);                                 // Выключить периферию передатчика
void off_HW_RX                          (void);                                 // Выключить периферию приемника
void on_HW_TX                           (void);                                 // Включить периферию передатчика
void on_HW_RX                           (void);                                 // Включить периферию приемника
void rxtx_switch                        (void);                                 // Сканирование тангенты и переключение режима
void load_freq_settings                 (uint8_t Cmd);
void parott_control                     (uint8_t Enable);                       // Автоматическое управление режимами передатчика при режиме попугая 


//******************************************************************************
// Секция определения макросов
//******************************************************************************
//#define MACRO1 ...
//#define MACRO2 ...
//#define ...
#endif // Закрывающий #endif к блокировке повторного включения
//******************************************************************************
// ENF OF FILE
//******************************************************************************