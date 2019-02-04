#ifndef _CODEC_H
#define _CODEC_H


#include "stdint.h"
#include "stdlib.h"
#include "intrinsics.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "trx.h"
#include "math.h"




//---   Reg 27. Audio Interface Setting Register 1   ---
#define AUDIO_INTERFACE_I2S     (0<<6)
#define AUDIO_INTERFACE_DSP     (1<<6)
#define AUDIO_INTERFACE_RJF     (2<<6)
#define AUDIO_INTERFACE_LJF     (3<<6)



#define  CODEC_I2C_ADRESS   0x30


void     CODEC_Init                     (void);
void     I2C_Write                      (uint8_t codec_register, uint8_t data); //�������� � ����� �o ������ �������� ��������
uint8_t  I2C_Read                       (uint8_t codec_register);

void     Set_MICPGA_L_Gain              (uint8_t gain);                         //���������� �������� MICPGA. � ������������� �� 47.5 ��
void     Set_MICPGA_R_Gain              (uint8_t gain);         
void     Set_ADC_Vol_L                  (int8_t Vol);                           //���������� ��������� ���   
void     Set_ADC_Vol_R                  (int8_t Vol);                           //
void     Set_DAC_Vol_L                  (int8_t Vol);                           //���������� ��������� ���
void     Set_DAC_Vol_R                  (int8_t Vol);                           //
void     Set_HPL_Gain                   (int8_t Gain);                          //���������� �������� ��������� ��������� ������ ������
void     Set_HPR_Gain                   (int8_t Gain);                          //���������� �������� ��������� ��������� ������� ������

void     load_gain_settings             (uint8_t Cmd);                          // ��������� ��������� � ����� �� ������� ���������� �������
void     set_RX_Gain_codec              (void);                                 // ��������� ��������� �������� � ����� ������ ����� (�����)
void     set_TX_Gain_codec              (void);                                 // ��������� ��������� �������� � ����� ����� ����� (��������)

void     codec_mic_to_left_phone        (void);                                 // TX. �������������� �������� ������ �� ����� ����� ��������� ������
void     codec_IN1R_to_ADC              (void);                                 // RX. �������������� ���� 1R � ���, ��� �� �������� ������ � ����� ���������. 


void left_DAC_mute (void);
void right_DAC_mute (void);
void DAC_UNmute (void);
void codec_RX_mode (void);
void codec_TX_mode (void);

void beep_R (uint16_t Freq, int16_t Vol, uint16_t Duration);


#define __CODEC_RESET_OFF       (HAL_GPIO_WritePin(CODEC_RESET_GPIO_Port, CODEC_RESET_Pin, GPIO_PIN_SET))
#define __CODEC_RESET_ON        (HAL_GPIO_WritePin(CODEC_RESET_GPIO_Port, CODEC_RESET_Pin, GPIO_PIN_RESET))


#endif