#ifndef _TRX_H // ��������� ��������� ��������� ����� ������
#define _TRX_H
//******************************************************************************
// ������ include: ����� ������������ ������������ ����� ������������ �������
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
// ������ ����������� ��������
//******************************************************************************
#define FW_REV "FW Rev.3.3 24.12.2018"

/*
Rev.3.3 24.12.2018
��������� ���������� �� ���� �������� � ��������� ������, ������ ������� ��������� �� ������� �������


*/



#define CODEC_BUF_SIZE  384                                     // 2 ������ �� 96 �������� * 2 ����������
#define HALF_DMA_SHIFT  (CODEC_BUF_SIZE/2)

#define LEFT_CH         0
#define RIGHT_CH        1

#define ADC_TX_BUF_SIZE (96*2)                                  // 2 ���������� ��� ������ �������� �� ��� ���������
#define PWM_BUF_SIZE    (ADC_TX_BUF_SIZE*4*4)                   // 2 ���������� � ������� �� 4 �������� ��� ������� �� ��� ������� �����

#define TX_OUT_AMP      100
#define TX_TONE_AMP     32000                                    // 

#define REC_BUF_SIZE    10 //24500                                   // ������ ������ ����������� ��� ������ � ���


//******************************************************************************
// ������ ����������� �����
//******************************************************************************
typedef enum {                                                                  // ����� ������ ����������
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

typedef enum {                                                                  // ����� ������ ����������
  SSB_USB,
  //SSB_LSB,
  
} MODULATION_e; 

typedef enum {                                                                  // ����� ������ ����������
  NO_DATA,
  HALF_RX,
  COMPLETE_RX,
} DATA_READY_e;   

typedef enum {                                                                  // �������� ���������
  RX_GAIN_LOW,
  RX_GAIN_MED,
  RX_GAIN_HIGH,
  RX_GAIN_CNT
}RX_GAIN_t;


typedef enum{                                                                   // ��� ������������ ������������ ��������� �������
  NO_BEEP,
  BEEP_1x1,
  BEEP_1x2,
  BEEP_1x3,
  BEEP_2x1,
  BEEP_2x2,
  BEEP_2x3,
  BEEP_CNT
}BEEP_TYPE_t;

    
typedef struct {                                                                // �������� 4 ������� �� ��� �������1, ������� ������ ��� �����������
  uint32_t Ch1;
  uint32_t Ch2;
  uint32_t Ch3;
  uint32_t Ch4;
} CC_CH_SET_t;


typedef struct {                                                                // �������� ������ ����
  uint8_t  Buf[256];                                                            // �����
  uint8_t  Idx;                                                                 // ������ ������
  uint8_t  New_string;                                                          // ���� "������� ����� ������"
} UART_t;


typedef struct {                                                                // �������� ������� ���
  uint8_t  Enable;
} AGC_t;

typedef struct {                                                                // �������� ����������� 
  uint8_t  Enable;
} DRC_t;


typedef struct {                                                                // ���������� �������������� ����������
  uint8_t  S_meterIntrval;                                                      // ���������� ��� ������ �������������� ������                                           
} RC_t;

typedef enum {                                                                  // ������ ��������� �������
  RXBW_NARROW,                                                                  // 900-1100
  RXBW_MEDIUM,                                                                  // 500-1500
  RXBW_WIDE,                                                                    // 350-2900
  RXBW_BYPASS,
  RXBW_CNT  
} RXBW_e;

typedef enum {                                                                  // ��������� �������������� ������-���������������
  OFF,
  START,
  TONE_OUT,
  REC,
  PLAY
} PAROTT_STATE_e;

typedef struct
{
  uint8_t               PWR_State;                                              // 0 - ���������, 1 - �������� 
  uint8_t               FlashLogEn;                                             // ���������� ������ ���� �� uSD �����
  TXRX_MODE_e           TXRX_Mode;                                              // ����� ��� ��������
  MODULATION_e          Modulation;                                             // ��� ���������
  BEEP_TYPE_t           Beep;                                                   // ������������ � ���������� ������� � �������� (������ ������, ��� ��)
  uint8_t               Tone_Puls;                                              // �������� ����������� ������� ������ ���� 1
  uint16_t              Test_tone_freq;                                         // �������� ������� ������������� ��������� ������� (������ +/- �������) 
  uint8_t               SQL_Enable;                                             // ��������� ��������������
  uint8_t               WriteSD;                                                // ����. ���������� ��� ������ �� uSD ����� ����
  
  UART_t                U1;                                                     // ����� ����1 ��� ������ � Bluetooth
  char                  Note[256];                                              // ����� ��� ������ ������� � ���
  uint8_t               NewNote;                                                // ����. ���� 1 - ���� ����� ������� ��� ������ � ���
  RC_t                  RC;                                                     // ���������� �������������� ����������
  RTC_DateTypeDef       GDate;                                                  // ��� ������ �� �������� � �����
  RTC_TimeTypeDef       GTime;
  uint32_t              Tick_1s;                                                // ��� ������������� �������
  uint32_t              UpTime;                                                 // ����� ������ ����������� � ������� ��������� � ��������
  uint32_t              PWR_main_mV;                                            // ���������� ������� ���������� � ��
  uint32_t              CPU_temperature;
  uint16_t              ADC1_buf[5];
  char                  TmpStr[128];                                            // ��������� ������ ��� ������ � ��.
  
  int16_t               CodecTxData[CODEC_BUF_SIZE]; 
  int16_t               CodecRxData[CODEC_BUF_SIZE];
  
  uint16_t              PWM_buf [PWM_BUF_SIZE];                                 // ��������� ����� �� �������� ��������� ������ ���
  uint16_t              ADC_TX_buf [ADC_TX_BUF_SIZE];                           // � ���� ����� ��������� ������� �� ��� ���������
  
  DATA_READY_e          ADC_DataReady;                                          // ����. � ������ ���� ������ ������� � ���������
  
  DDS16_t               DDS_RX;                                                 // ��������� ������� ������� �������������� ������
  DDS16_t               DDS_TX;                                                 // ��������� ������� ������� �������������� ��������
  DDS16_t               DDS_GA;                                                 // ��������� ������� ������������������ ������
  DDS16_t               DDS_AUX;                                                // ��������������� ��������� ��� ��������� ������ �����
  
  
  //---   SSB ��������  --------------------------------------------------------
  uint8_t               USB_On;                                                 // ����. ���� ==1, �� ������� ������� ��� SSB                    
  RXBW_e                RXBW;                                                   // ������ ���������� ���������� �������
  float                 Buf_LPF_48k_I[72];                                      // ����� ������� ������� 32 ������� ���������� �� 6
  float                 Buf_LPF_48k_Q[72];
  uint8_t               Buf_LPF_48k_idx;                                        // ��������� �������� �������
  
  float                 Buf_LPF_8k_I[128];                                      // ������ �������� ���������� � ��������.                                
  float                 Buf_LPF_8k_Q[128];
  
  float                 Buf_BPF_8k[128];                                        // ��������� ������
  
  float                 Buf_48k_Out_Work[12];                                   // ��� �������� ��� �������������. ������� �������
  float                 Buf_48k_Out_Work_Q[12];
  float                 Buf_48k_Out[6];                                         // ������� ��������
  float                 Buf_48k_Out_Q[6];
  float                 Buf_48k_Out_Work_Echo[12];                              // ������� �������� ��� �������� ���������������
  float                 Buf_48k_Out_Echo[6];
  
  float                 SSB_Out_F;                                              // ��� �������� ��������� ��������
  
  SQUELCH_t             RXSQL;                                                  // �������� �������������� ���������
  //----------------------------------------------------------------------------
  
  
  
  //---   ����������������� �����  ---------------------------------------------   
  uint8_t               GA_USB_On;                                              // ����. ���� ==1, �� ������� ������� ��� SSB                    
  RXBW_e                GA_RXBW;                                                // ������ ���������� ���������� �������

  float                 GA_Buf_LPF_48k_I[72];                                   // ����� ������� ������� 32 ������� ���������� �� 6
  float                 GA_Buf_LPF_48k_Q[72];
  uint8_t               GA_Buf_LPF_48k_idx;                                     // ��������� �������� �������
  
  float                 GA_Buf_LPF_8k_I[128];                                   // ������ �������� ���������� � ��������.                                
  float                 GA_Buf_LPF_8k_Q[128];
  
  float                 GA_Buf_BPF_8k[128];                                     // ��������� ������
  
  float                 GA_Buf_48k_Out_Work[12];                                // ��� �������� ��� �������������. ������� �������
  float                 GA_Buf_48k_Out_Work_Q[12];
  float                 GA_Buf_48k_Out[6];                                      // ������� ��������
  float                 GA_Buf_48k_Out_Q[6];
  float                 GA_Buf_48k_Out_Work_Echo[12];                           // ������� �������� ��� �������� ���������������
  float                 GA_Buf_48k_Out_Echo[6];
  
  float                 GA_SSB_Out_F;                                           // ��� �������� ��������� ��������
  //----------------------------------------------------------------------------


  //---   SSB ����������  --------------------------------------------------------
  int32_t               TX_pwr;                                                 // ����� ����������� 0-100%
  
  //----------------------------------------------------------------------------
  
  AGC_t AGC;                                                                    // RX AGC SSB
  AGC_t TXAGC;                                                                  // TX AGC Microphone
  AGC_t RXAGC;
  DRC_t TXDRC;
  
  //---   �����   --------------------------------------------------------------
  int16_t               Gain_MICPGAL;
  int16_t               Gain_MICPGAR;
  int16_t               Gain_ADCL;
  int16_t               Gain_ADCR;
  int16_t               Gain_DACL;
  int16_t               Gain_DACR;
  int16_t               Gain_HPL;
  int16_t               Gain_HPR;
  //----------------------------------------------------------------------------
  
  
  //---   ����������   ---------------------------------------------------------
  uint8_t               EqRx_En;
  uint8_t               EqTx_En;
  
  ALLPASS_t             EqTxLow;
  ALLPASS_t             EqTxMed;
  ALLPASS_t             EqTxHigh;
  
  ALLPASS_t             EqRxLow;
  ALLPASS_t             EqRxMed;
  ALLPASS_t             EqRxHigh;
  //----------------------------------------------------------------------------
  
  
  
  //---    ����������   --------------------------------------------------------
  uint32_t              Parott_En;                                              // ���������� ������ �������
  PAROTT_STATE_e        Parott_State;
  int16_t               RecBuf[REC_BUF_SIZE];                                   // ����� ��� ������ 3 ��� ������ � ������� 16 ���.
  uint32_t              RecBufIdx;
  
  int8_t                SD_Rec_buf[8192];                                       // ��� ������ �� 2048 ���� ��� ������ �� uSD
  uint16_t              SD_Rec_idx;
  int8_t                SD_Rec_Save;                                            // 0-������ ���������, 1-��������� 0-511, 2-��������� 512-1023
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
  
  uint32_t              TX_current_AVG;                                         // ������� ��� � ���� �������� ����� �����������
  uint32_t              TX_current_Peack;                                       // ������� ��� � ���� �������� ����� �����������

  
  float Tmp_f;
  float TmpI; 
  float TmpQ;
  int32_t Tmp_i32;

                                 
  

} REG_t;
//******************************************************************************
// ������ ����������� ���������� ����������
//******************************************************************************
extern const float  Sin_table [4096];
extern REG_t R;                                                                 // ������
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
// ������ ���������� ���������� �������
//******************************************************************************
void main_radio                         (void);
void adc1_scan                          (void);                                 // ��������� ������� � ���� ��������
void record_audio_to_uSD                (void);                                 // ������ ����� �� SD �����
void adc_mic_half_dma_routine           (void);
void adc_mic_complete_dma_routine       (void);
void set_TXRX_mode                      (TXRX_MODE_e TRX_Mode);
void set_DDS_freq                       (uint32_t Freq);
void off_HW_TX                          (void);                                 // ��������� ��������� �����������
void off_HW_RX                          (void);                                 // ��������� ��������� ���������
void on_HW_TX                           (void);                                 // �������� ��������� �����������
void on_HW_RX                           (void);                                 // �������� ��������� ���������
void rxtx_switch                        (void);                                 // ������������ �������� � ������������ ������
void load_freq_settings                 (uint8_t Cmd);
void parott_control                     (uint8_t Enable);                       // �������������� ���������� �������� ����������� ��� ������ ������� 


//******************************************************************************
// ������ ����������� ��������
//******************************************************************************
//#define MACRO1 ...
//#define MACRO2 ...
//#define ...
#endif // ����������� #endif � ���������� ���������� ���������
//******************************************************************************
// ENF OF FILE
//******************************************************************************