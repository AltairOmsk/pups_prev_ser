/*
* Copyright (C) 2018 Yuri Ryzhenko <Y.Ryzhenko@hi-tech.org>, Aleksey Kirsanov <a.kirsanov@iva-tech.ru>
* All rights reserved
*
* File Name  : codec.c
* Description: DSP processing
*/
//******************************************************************************
// ������ include: ����� ������������ ������������ ���� � ������
//******************************************************************************
#include "codec.h" // �������� ���� ��������� ��� ������ ������
#include "cmsis_os.h"
//******************************************************************************
// ������ ����������� ����������, ������������ � ������
//******************************************************************************
//------------------------------------------------------------------------------
// ����������
//------------------------------------------------------------------------------
extern I2C_HandleTypeDef hi2c1;

//------------------------------------------------------------------------------
// ���������
//------------------------------------------------------------------------------
uint8_t I2C_data_buf[2];

//******************************************************************************
// ������ ���������� ��������� �������
//******************************************************************************
void codec_AGC_left  (uint8_t Enable);
void codec_AGC_right (uint8_t Enable);
void codec_DRC_left  (uint8_t Enable);

//******************************************************************************
// ������ �������� ������� (������� ����������, ����� ���������)
//******************************************************************************
void     CODEC_Init     (void) {
  static uint8_t temp;

  __CODEC_RESET_OFF;

  
  
      //--------------------------------------------------------------------------------------------------  
    I2C_Write (0x00, 0x00);             // Initialize to Page 0
    I2C_Write (0x01, 0x01);             // Initialize the device through software reset
    
    I2C_Write (0x12, 0x81);             // Power up NADC divider with value 1
    I2C_Write (0x13, 0x82);             // Power up MADC divider with value 2
    I2C_Write (0x14, 0x80);             // Program OSR for ADC to 128
 
    I2C_Write (0x0b, 0x81);             // Power up the NDAC divider with value 1
    I2C_Write (0x0c, 0x82);             // Power up the MDAC divider with value 2   
    I2C_Write (0x0d, 0x00);             // Program the OSR of DAC to 128 (CODEC default settings)
    I2C_Write (0x0e, 0x80);
    
    I2C_Write (0x1b, 0x00);             // Set the word length of Audio Interface to 16 bits PTM_P4
    I2C_Write (0x3c, 0x08);             // Set the DAC Mode to PRB_P8 (without DRC)   
          //I2C_Write (0x3c, 0x19);             // Set the DAC Mode to PRB_P25
    I2C_Write (0x3d, 0x01);             // Select ADC PRB_R1
    
    I2C_Write (0x36, 0x02);             // DIN is enabled for Primary Data Input or Digital Microphone Input or General Purpose Clock
    
    
    
    //--------------------------------------------------------------------------------------------------
    I2C_Write (0x00, 0x01);             // Select Page 1
    I2C_Write (0x01, 0x08);             // Disable Internal Crude AVdd in presence of external AVdd supply or before powering up internal AVdd LDO
    I2C_Write (0x02, 0x01);             // Enable Master Analog Power Control
    I2C_Write (0x7b, 0x01);             // Set the REF charging time to 40ms
    I2C_Write (0x33, 0x40);             // Power-up MIC BIAS
    I2C_Write (0x14, 0x25);             /* HP soft stepping settings for optimal pop performance at power up
                                           Rpop used is 6k with N = 6 and soft step = 20usec. This should work with 47uF coupling
                                           capacitor. Can try N=5,6 or 7 time constants as well. Trade-off delay vs �pop� sound.*/
    I2C_Write (0x0a, 0x00);             // Set the Input Common Mode to 0.9V and Output Common Mode for Headphone to Input Common Mode

    I2C_Write (0x0c, 0x08);             // Route Left DAC to HPL
    //I2C_Write (0x0c, 0x02);           // MAL output is routed to HPL
    I2C_Write (0x0d, 0x08);             // Route Right DAC to HPR    
//    I2C_Write (0x0e, 0x08);           // Route Left DAC to LOL
//    I2C_Write (0x0f, 0x08);           // Route Right DAC to  LOR
    
    
    
    I2C_Write (0x03, 0x00);             // Set the DAC PTM mode to PTM_P3/4
    I2C_Write (0x04, 0x00);
    
    //I2C_Write (0x10, 0x0A);           // Set the HPL gain to 10dB
    //I2C_Write (0x10, 0x14);           // Set the HPL gain to 20dB
    I2C_Write (0x10, 0x00);             // Set the HPL gain to 0dB
    //I2C_Write (0x11, 0x0A);           // Set the HPR gain to 10dB    
    I2C_Write (0x11, 0x00);             // Set the HPR gain to 0dB
    
     
    I2C_Write (0x12, 0x03);             // Set the LOL gain to 3dB
    I2C_Write (0x13, 0x03);             // Set the LOR gain to 3dB
    
    I2C_Write (0x09, 0x3F);             // Power up HPL,HPR,LOL,LOR,MixAmp drivers
    
    I2C_Write (0x3d, 0x00);             // Select ADC PTM_R4
    I2C_Write (0x47, 0x32);             // Set MicPGA startup delay to 3.1ms
    I2C_Write (0x7b, 0x01);             // Set the REF charging time to 40ms

//                I2C_Write (0x34, 0x04);             // IN3L is routed to Left MICPGA with 10k resistance. ��� ��������.
//                I2C_Write (0x36, 0x04);             // IN3R is routed to Left MICPGA with 10k resistance  
//    I2C_Write (0x37, 0x04);             // IN3R is routed to Right MICPGA with 10k resistance
//    I2C_Write (0x39, 0x04);             // IN3L is routed to Right MICPGA with 10k resistance
    
//    I2C_Write (0x34, 0x40);             // IN1L is routed to Left MICPGA with 10k resistance
//    I2C_Write (0x36, 0x40);             // CM is routed to Left MICPGA via CM1L with 10k resistance  
    I2C_Write (0x37, 0x40);             // IN1R is routed to Right MICPGA with 10k resistance
    I2C_Write (0x39, 0x40);             // CM is routed to Right MICPGA via CM1R with 10k resistance    
    
    
//    // �������� �� IN2L (������ ���� �� 2�)
//    I2C_Write (0x34, 0x10);             // IN2L is routed to Left MICPGA with 10k resistance
//    I2C_Write (0x36, 0x01);             // CM is routed to Left MICPGA via CM1R with 10k resistance 
    
    // �������� �� ���������������� IN3L � ������ 10 �
    I2C_Write (0x34, 0x04);             // IN2L is routed to Left MICPGA with 10k resistance
    I2C_Write (0x36, 0x04);             // CM is routed to Left MICPGA via CM1R with 10k resistance 
    
    
    

//    I2C_Write (0x3b, 0x00);             // Unmute Left MICPGA, Gain 0dB
    I2C_Write (0x3c, 0x00);             // Unmute Right MICPGA, Gain 0dB
//    I2C_Write (0x3b, 0x28);             // Unmute Left MICPGA, Gain 20dB
//    I2C_Write (0x3c, 0x28);             // Unmute Right MICPGA, Gain 20dB
    //I2C_Write (0x3b, 0x3C);             // Unmute Left MICPGA, Gain 30dB
    I2C_Write (0x3b, 0x50);             // Unmute Left MICPGA, Gain 40dB
//    I2C_Write (0x3c, 0x3C);             // Unmute Right MICPGA, Gain 30dB
  
    
    osDelay(1000);                      // Wait for 2.5 sec for soft stepping to take effect
    
    
    //--------------------------------------------------------------------------------------------------
    I2C_Write (0x00, 0x00);             // Select Page 0      
    I2C_Write (0x3f, 0xd6);             // Power up the Left and Right DAC Channels with route the Left Audio digital data to
                                        // Left Channel DAC and Right Audio digital data to Right Channel DAC
    I2C_Write (0x40, 0x00);             // Unmute the DAC digital volume control
    I2C_Write (0x41, 0x00);             // Left DAC 0011 0000: Digital Volume Control = 0dB
    I2C_Write (0x42, 0x00);             // Right DAC 0011 0000: Digital Volume Control = 0dB
//    I2C_Write (0x41, 0x14);             // Left DAC 0011 0000: Digital Volume Control = 10dB
//    I2C_Write (0x42, 0x14);             // Right DAC 0011 0000: Digital Volume Control = 10dB
//    I2C_Write (0x41, 0x30);             // Left DAC 0011 0000: Digital Volume Control = +24dB
//    I2C_Write (0x42, 0x30);             // Right DAC 0011 0000: Digital Volume Control = +24dB    
    
    I2C_Write (0x51, 0xc0);             // Power up Left and Right ADC Channels
    I2C_Write (0x52, 0x00);             // Unmute Left and Right ADC Digital Volume Control.
//    I2C_Write (0x53, 0x28);             // Left ADC Channel Volume = 20.0dB
//    I2C_Write (0x54, 0x28);             // Right ADC Channel Volume = 20.0dB
    
    
//    I2C_Write (0x1D, 0x10);             // LOOP I2S Stereo ADC output is routed to Stereo DAC input
//    I2C_Write (0x1D, 0x20);             // LOOP Audio Data in is routed to Audio Data out. (Works only when WCLK is configured as input.)

    
    set_RX_Gain_codec();
    set_TX_Gain_codec();
    
    codec_AGC_left (R.TXAGC.Enable);
    codec_AGC_right(R.RXAGC.Enable);
    
    codec_DRC_left(R.TXDRC.Enable);
    
    
    
}

void codec_RX_mode (void){
    I2C_Write (0x00, 0x01);             // Select Page 1
    I2C_Write (0x34, 0x40);             // IN1L is routed to Left MICPGA with 10k resistance
    I2C_Write (0x36, 0x40);             // CM is routed to Left MICPGA via CM1L with 10k resistance  
    I2C_Write (0x37, 0x40);             // IN1R is routed to Right MICPGA with 10k resistance
    I2C_Write (0x39, 0x40);             // CM is routed to Right MICPGA via CM1R with 10k resistance 
    I2C_Write (0x3b, 0x50);             // Unmute Left MICPGA, Gain 40dB
    I2C_Write (0x3c, 0x50);             // Unmute Right MICPGA, Gain 30dB
    
    I2C_Write (0x00, 0x00);             // Select Page 0      
    I2C_Write (0x3f, 0xd6);             // Power up the Left and Right DAC Channels with route the Left Audio digital data to
                                        // Left Channel DAC and Right Audio digital data to Right Channel DAC
    I2C_Write (0x40, 0x00);             // Unmute the DAC digital volume control
    I2C_Write (0x41, 0x00);             // Left DAC 0011 0000: Digital Volume Control = 0dB
    I2C_Write (0x42, 0x00);             // Right DAC 0011 0000: Digital Volume Control = 0dB
  
    
    I2C_Write (0x51, 0xc0);             // Power up Left and Right ADC Channels
    I2C_Write (0x52, 0x00);             // Unmute Left and Right ADC Digital Volume Control.
//    I2C_Write (0x53, 0x28);             // Left ADC Channel Volume = 20.0dB
//    I2C_Write (0x54, 0x28);             // Right ADC Channel Volume = 20.0dB
};
void codec_TX_mode (void){};

void left_DAC_mute (void){
I2C_Write (0x00, 0x00);                                                         // Select Page 0
I2C_Write (0x40, (1 << 3));                                                     // Mute the Left DAC digital volume control
}

void right_DAC_mute (void){
I2C_Write (0x00, 0x00);                                                         // Select Page 0
I2C_Write (0x40, (1 << 2));                                                     // Mute theRight DAC digital volume control
}


void DAC_UNmute (void){
I2C_Write (0x00, 0x00);                                                         // Select Page 0
I2C_Write (0x40, 0x00);                                                         // Unmute the Left DAC digital volume control
}




//***   MICPGA Gain Control   **************************************************
void     Set_MICPGA_L_Gain (uint8_t gain)  {
  if (gain <= (47.5 / 0.5)){
    I2C_Write (00, 01);                   // Set Page 1
    I2C_Write (0x3b, gain);               // Unmute Left MICPGA, Gain 0dB
  }
}
void     Set_MICPGA_R_Gain (uint8_t gain)  {
  if (gain <= (47.5 / 0.5)){
    I2C_Write (00, 01);                   // Set Page 1
    I2C_Write (0x3c, gain);               // Unmute Right MICPGA, Gain 0dB
  }
}


//***   ADC Volume control   ***************************************************
void     Set_ADC_Vol_L     (int8_t Vol) {
  if ((Vol >= -24) && (Vol <= 40)) {
    I2C_Write (00, 00);                   // Set Page 0
    I2C_Write (0x53, (uint8_t)Vol);       // Left ADC Channel Volume
  }
}
void     Set_ADC_Vol_R     (int8_t Vol) {
  if ((Vol >= -24) && (Vol <= 40)) {
    I2C_Write (00, 00);                   // Set Page 0
    I2C_Write (0x54, (uint8_t)Vol);       // Right ADC Channel Volume
  }
}


//***   DAC Volume control   ***************************************************
void     Set_DAC_Vol_L     (int8_t Vol){    //���������� ��������� ���
  if ((Vol >= -127) && (Vol <= 48)) {
    I2C_Write (00, 00);                   // Set Page 0
    I2C_Write (0x41, (uint8_t)Vol);       // Left ADC Channel Volume
  }
}
void     Set_DAC_Vol_R     (int8_t Vol){    //���������� ��������� ���
  if ((Vol >= -127) && (Vol <= 48)) {
    I2C_Write (00, 00);                   // Set Page 0
    I2C_Write (0x42, (uint8_t)Vol);       // Right ADC Channel Volume
  }
}


//***   HPL Gain control   *****************************************************
void     Set_HPL_Gain    (int8_t Gain){   //���������� �������� ��������� ���������
  if ((Gain >= -6) && (Gain <= 29)){
    I2C_Write (00, 01);                   // Set Page 1
    I2C_Write (0x10, Gain);               // Unmute Left MICPGA, Gain 0dB
  }
}

//***   HPR Gain control   *****************************************************
void     Set_HPR_Gain    (int8_t Gain){   //���������� �������� ��������� ���������
  if ((Gain >= -6) && (Gain <= 29)){
    I2C_Write (00, 01);                   // Set Page 1
    I2C_Write (0x11, Gain);               // Unmute Right MICPGA, Gain 0dB
  }
}



//***   ��������� ��������� �������� � �����   *********************************
void set_RX_Gain_codec (void) {
    Set_MICPGA_R_Gain(R.Gain_MICPGAR);             // ���������� �������� MICPGA. � ������������� �� 47.5 ��
    Set_ADC_Vol_R(R.Gain_ADCR);                    // -24  40 � �������������
    Set_DAC_Vol_R(R.Gain_DACR);                    // -127 48 � �������������
    Set_HPR_Gain(R.Gain_HPR);                      // -6 29  
}

void set_TX_Gain_codec (void) {
    Set_MICPGA_L_Gain(R.Gain_MICPGAL);             // ���������� �������� MICPGA. � ������������� �� 47.5 ��
    Set_ADC_Vol_L(R.Gain_ADCL);                    // -24  40 � �������������
    Set_DAC_Vol_L(R.Gain_DACL);                    // -127 48 � �������������
    Set_HPL_Gain(R.Gain_HPL);                      // -6 29  
}




//***   ��������� ��������� �������� � ����� (��� ���������� �� ������ �� �����)  
void     load_gain_settings (uint8_t Cmd) {
static uint8_t Current_set=0;

  if (Cmd == 1) {
    if (++R.RX_gain >= RX_GAIN_CNT) { R.RX_gain = RX_GAIN_LOW;}
    switch (R.RX_gain){
    case RX_GAIN_LOW:
        R.Gain_MICPGAR  = 20;
        R.Gain_ADCR     = 0;
        R.Gain_DACR     = 0;
        R.Gain_HPR      = 0;
//        Set_MICPGA_R_Gain(20);             // ���������� �������� MICPGA. � ������������� �� 47.5 ��
//        Set_ADC_Vol_R(0);                 // -24 40
//        Set_DAC_Vol_R(0);                 // -127 48
//        Set_HPR_Gain(0);                // -6 29
//        R.Beep = BEEP_1x1;
      break;
    case RX_GAIN_MED:
        R.Gain_MICPGAR  = 40;
        R.Gain_ADCR     = 10;
        R.Gain_DACR     = 10;
        R.Gain_HPR      = 0;
//        Set_MICPGA_R_Gain(40);            // ���������� �������� MICPGA. � ������������� �� 47.5 ��
//        Set_ADC_Vol_R(10);                 // -24 40
//        Set_DAC_Vol_R(10);                 // -127 48
//        Set_HPR_Gain(00);                // -6 29  
//        R.Beep = BEEP_1x2;
      break;
    case RX_GAIN_HIGH:
        R.Gain_MICPGAR  = 40;
        R.Gain_ADCR     = 20;
        R.Gain_DACR     = 20;
        R.Gain_HPR      = 10;
//        Set_MICPGA_R_Gain(40);            // ���������� �������� MICPGA. � ������������� �� 47.5 ��
//        Set_ADC_Vol_R(20);                 // -24 40
//        Set_DAC_Vol_R(20);                 // -127 48
//        Set_HPR_Gain(10);                // -6 29     
//        R.Beep = BEEP_1x3;
      break;
    default:
        R.Gain_MICPGAR  = 20;
        R.Gain_ADCR     = 0;
        R.Gain_DACR     = 0;
        R.Gain_HPR      = 0;
//        Set_MICPGA_R_Gain(0);             // ���������� �������� MICPGA. � ������������� �� 47.5 ��
//        Set_ADC_Vol_R(0);                 // -24 40
//        Set_DAC_Vol_R(0);                 // -127 48
//        Set_HPR_Gain(0);                // -6 29      
      break;
    }
    set_RX_Gain_codec();
  } // if (Cmd == 1)
}


void I2C_Write (uint8_t codec_register, uint8_t data){                    //�������� � ����� �� ������ �������� ��������
uint8_t out_buf[2];
/**
  * @brief  Transmits in master mode an amount of data in blocking mode.
  * @param  hi2c : Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress: Target device address
  * @param  pData: Pointer to data buffer
  * @param  Size: Amount of data to be sent
  * @param  Timeout: Timeout duration
  * @retval HAL status
  */

out_buf[0] = codec_register;
out_buf[1] = data;

HAL_I2C_Master_Transmit(&hi2c1, CODEC_I2C_ADRESS, out_buf, 2, 10000);

}  

//******************************************************************************
//   TX. �������������� �������� ������ �� ����� ����� ��������� ������
//******************************************************************************
// ������� �� �������� �����
//          O-Spk- 19 Ohm
//    Spk-*   *-mic
//          *-mic 145 Ohm 
void     codec_mic_to_left_phone        (void){
//    //��������� ������. ������������� �������� ��������� 1
//    I2C_Write (0x00, 0x01);             // Select Page 1
//    I2C_Write (0x0e, 0x02);             // Route Left  MAL to LOL (TX)
//    //I2C_Write (0x0e, 0x10);             // Route Right DAC to  LOR (RX)
//    I2C_Write (0x12, 0x00);             // LOL driver gain is 0 dB (TX)
    

    //I2C_Write (0x1D, 0x10);             // LOOP I2S Stereo ADC output is routed to Stereo DAC input
    //I2C_Write (0x1D, 0x20);             // LOOP Audio Data in is routed to Audio Data out. (Works only when WCLK is configured as input.)

  
}                                

 

//******************************************************************************
// RX. �������������� ���� 1R � ���, ��� �� �������� ������ � ����� ���������. 
//******************************************************************************
void     codec_IN1R_to_ADC              (void){
  
//        //��������� ������. ������������� �������� ��������� 1
//    I2C_Write (0x00, 0x01);             // Select Page 1
//    //I2C_Write (0x0e, 0x02);             // Route Left  MAL to LOL (TX)
//    I2C_Write (0x0e, 0x10);             // Route Right DAC to  LOR (RX)
//    I2C_Write (0x12, 0x1D);             // LOL driver gain is 29dB (RX)
  
  
  
  
  
        //��������� ������. ������������� �������� ��������� 1
        //I2C_Write (0x00, 0x01);             // Select Page 1
        //������������� �������� ������������ ��������� 0 ��
        //I2C_Write (0x3b, 0x14);             // Unmute Left  MICPGA, Gain 0dB
        //I2C_Write (0x3c, 0x00);             // Unmute Right MICPGA, Gain 0dB
        //���� ��� ��������� � ��������� ����� 1 (��� �������� �� ���������� � ���������)
//        I2C_Write (0x34, 0x40);             // IN1L is routed to Left MICPGA with 10k resistance
//        I2C_Write (0x36, 0x40);             // CM is routed to Left MICPGA via CM1L with 10k resistance  
        //I2C_Write (0x37, 0x40);             // IN1R is routed to Right MICPGA with 10k resistance
        //I2C_Write (0x39, 0x40);             // CM is routed to Right MICPGA via CM1R with 10k resistance   
               
//        //���� ��� ��������� � ��������� ����� 2, ���������� ������ 1
//        I2C_Write (0x34, 0x10);             // IN2R is routed to Right MICPGA with 10k resistance
//        I2C_Write (0x36, 0x01);             // CM is routed to Left MICPGA via CM1L with 10k resistance  
//        I2C_Write (0x37, 0x10);             // IN1R is routed to Right MICPGA with 10k resistance
//        I2C_Write (0x39, 0x01);             // CM is routed to Right MICPGA via CM1R with 10k resistance 
        
        //���� ��� ��������� � ��������� ����� 3 (��������) ��������� ��� ��������, ���� ������� �����
//        I2C_Write (0x34, 0x04);             // IN3L is routed to Left MICPGA with 10k resistance
//        I2C_Write (0x36, 0x04);             // IN3R is routed to Left MICPGA with 10k resistance  
//        I2C_Write (0x37, 0x04);             // IN3R is routed to Right MICPGA with 10k resistance
//        I2C_Write (0x39, 0x04);             // IN3L is routed to Right MICPGA with 10k resistance
        
//        //���� ��� ��������� ��������������� � ����� 3 � ����������� �� ������ ����� ���
//        I2C_Write (0x34, 0x00);             // 
//        I2C_Write (0x36, 0x00);             //  
//        I2C_Write (0x37, 0x04);             // IN3R is routed to Right MICPGA with 10k resistance
//        I2C_Write (0x39, 0x04);             // IN3L is routed to Right MICPGA with 10k resistance
        
        
//        //����� ��� ���������� �� ��������
//        I2C_Write (0x0e, 0x00);             // UnRoute Left DAC to LOL
//        I2C_Write (0x0f, 0x00);             // UnRoute Right DAC to  LOR
//        //I2C_Write (0x0c, 0x08);             // Route Left DAC to HPL
//        I2C_Write (0x0c, 0x02);             // MAL output is routed to HPL
//        I2C_Write (0x0d, 0x08);             // Route Right DAC to HPR 
} 


//******************************************************************************
//   Codec I2C Read
//******************************************************************************
uint8_t I2C_Read  (uint8_t codec_register) {
uint8_t out;

/**
  * @brief  Receives in master mode an amount of data in blocking mode.
  * @param  hi2c : Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress: Target device address
  * @param  pData: Pointer to data buffer
  * @param  Size: Amount of data to be sent
  * @param  Timeout: Timeout duration
  * @retval HAL status
  */
HAL_I2C_Master_Transmit(&hi2c1, CODEC_I2C_ADRESS, &codec_register, 1, 10000);
HAL_I2C_Master_Receive (&hi2c1, CODEC_I2C_ADRESS, &out, 1, 10000);


return out;
}






/*
��������� �� 0 �� -63
*/
void beep_R (uint16_t Freq, int16_t Vol, uint16_t Duration) {
float Tmp;
uint32_t Tmp32;

#define FDISKR          48828
#define PI              3.141592653

    I2C_Write (0x00, 0x00);             // Initialize to Page 0
    
//    I2C_Write (0x49, 0x01);
//    I2C_Write (0x4A, 0x77);
//    I2C_Write (0x4B, 0x00);
//    I2C_Write (0x4C, 0x23);
//    I2C_Write (0x4D, 0xFB);
//    I2C_Write (0x4E, 0x7A);
//    I2C_Write (0x4F, 0xD7);
//    
//    I2C_Write (0x48, 0x04);
//    I2C_Write (0x47, 0x84);
    
    
    
    
    //---   Vol   ---
    if (Vol > 0)   Vol = 0;
    if (Vol < -63) Vol = -63;
    I2C_Write (72, abs(Vol));
    
    //---  Freq   ---
    Tmp = sin(2 * 3.14f * ((float)Freq/FDISKR));
    Tmp *= 32768;
    Tmp32 = (uint32_t)Tmp;
    
        //printf ("sin=%x \r\n", Tmp32);
        
    I2C_Write (76, (0xFF & (Tmp32>>8)));
    I2C_Write (77, (0xFF & (Tmp32)));
    
    Tmp = cos(2 * 3.14f * ((float)Freq/FDISKR));
    Tmp *= 32768;
    Tmp32 = (uint32_t)Tmp;
    I2C_Write (78, (0xFF & (Tmp32>>8)));
    I2C_Write (79, (0xFF & (Tmp32)));
    
        //printf ("cos=%x \r\n", Tmp32);
    
    //---   Duration   ---
    Tmp32 = ((uint32_t)Freq * (uint32_t)Duration * FDISKR) / 1000000;
    I2C_Write (73, (0xFF & (Tmp32>>16)));
    I2C_Write (74, (0xFF & (Tmp32>>8)));
    I2C_Write (75, (0xFF & (Tmp32)));
    
        //printf ("Dur=%x \r\n", Tmp32);
    
    I2C_Write (71, (1<<7));
    
        //printf ("En=%x \r\n", (1<<7));
    
    
    __no_operation();
}


//******************************************************************************
// ��������� � ��������� ��� ������ ������ ������ ���. �������� �� ����� ������ ����� �� ������
//******************************************************************************
void codec_AGC_left (uint8_t Enable){
//uint8_t Tmp;  
//
  if (Enable != 0) {
//    I2C_Write (0x00, 0x00);             // Initialize to Page 0
//    I2C_Write (86, 0xF2);               // AGC Enable(D7), Target level(D6:0) // ������         �������
//    
////    Tmp  = 0;
////    Tmp |= (0x02 << 6);               // Hysteresis(D7:6)
////    Tmp |= (0x2A);                    // Noise Threshold(dB)(D5:1)
//    I2C_Write (87, 0xAA);               // Applay settings
//    
//    I2C_Write (88, 0x53);               // MaxPGA applicable(dB)D(6:0)
//    I2C_Write (89, 0x50);               // Attack time
//    I2C_Write (90, 0x00);               // Decay time
//    I2C_Write (91, 0x0C);               // Noise Debounce time D(4:0)
//    I2C_Write (92, 0x03);               // Signal Debounce time D(3:0)
    
    
    
    // AGC gain 15 dB, target -24 no threshold
    I2C_Write (0x00, 0x00);             // Initialize to Page 0
    I2C_Write (86, 0xF2);               // AGC Enable(D7), Target level(D6:0)
    I2C_Write (87, 0x80);               // Applay settings
    I2C_Write (88, 0x1E);               // MaxPGA applicable(dB)D(6:0)
    I2C_Write (89, 0x08);               // Attack time
    I2C_Write (90, 0x32);               // Decay time
    I2C_Write (91, 0x00);               // Noise Debounce time D(4:0)
    I2C_Write (92, 0x06);
  } else {
    // Reset state
    I2C_Write (0x00, 0x00);             // Initialize to Page 0
    I2C_Write (86, 0x00);               // AGC Enable(D7), Target level(D6:0)
    I2C_Write (87, 0x00);               // Applay settings
    I2C_Write (88, 0x7F);               // MaxPGA applicable(dB)D(6:0)
    I2C_Write (89, 0x00);               // Attack time
    I2C_Write (90, 0x00);               // Decay time
    I2C_Write (91, 0x00);               // Noise Debounce time D(4:0)
    I2C_Write (92, 0x00);
  
  }
    
    
    
//    // ����� ������� ��������
//    I2C_Write (86, 0xF2);               // AGC Enable(D7), Target level(D6:0)
//    I2C_Write (87, 0x80);               // Applay settings
//    I2C_Write (88, 0x64);               // MaxPGA applicable(dB)D(6:0)
//    I2C_Write (89, 0x50);               // Attack time
//    I2C_Write (90, 0x00);               // Decay time
//    I2C_Write (91, 0x0C);               // Noise Debounce time D(4:0)
//    I2C_Write (92, 0x03);               // Signal Debounce time D(3:0)
  

}



// �����
void codec_AGC_right (uint8_t Enable){

  if (Enable != 0) {
    
    
     // AGC gain 15 dB, target -10 no threshold
    I2C_Write (0x00, 0x00);             // Initialize to Page 0
    I2C_Write (94, 0xA2);               // AGC Enable(D7), Target level(D6:0)
    I2C_Write (95, 0x80);               // Applay settings
    I2C_Write (96, 0x1E);               // MaxPGA applicable(dB)D(6:0)
    I2C_Write (97, 0x08);               // Attack time
    I2C_Write (98, 0x32);               // Decay time
    I2C_Write (99, 0x00);               // Noise Debounce time D(4:0)
    I2C_Write (100, 0x06);
    
    
    
//    // AGC gain 40 dB, target -20 no threshold
//    I2C_Write (0x00, 0x00);             // Initialize to Page 0
//    I2C_Write (94, 0xE2);               // AGC Enable(D7), Target level(D6:0)
//    I2C_Write (95, 0x80);               // Applay settings
//    I2C_Write (96, 0x50);               // MaxPGA applicable(dB)D(6:0)
//    I2C_Write (97, 0x50);               // Attack time
//    I2C_Write (98, 0x00);               // Decay time
//    I2C_Write (99, 0x0C);               // Noise Debounce time D(4:0)
//    I2C_Write (100, 0x03);

//    // AGC gain 40 dB, target -24 no threshold
//    I2C_Write (0x00, 0x00);             // Initialize to Page 0
//    I2C_Write (94, 0xF2);               // AGC Enable(D7), Target level(D6:0)
//    I2C_Write (95, 0x80);               // Applay settings
//    I2C_Write (96, 0x50);               // MaxPGA applicable(dB)D(6:0)
//    I2C_Write (97, 0x50);               // Attack time
//    I2C_Write (98, 0x00);               // Decay time
//    I2C_Write (99, 0x0C);               // Noise Debounce time D(4:0)
//    I2C_Write (100, 0x03);
  } else {
    // Reset state
    I2C_Write (0x00, 0x00);             // Initialize to Page 0
    I2C_Write (94, 0x00);               // AGC Enable(D7), Target level(D6:0)
    I2C_Write (95, 0x00);               // Applay settings
    I2C_Write (96, 0x7F);               // MaxPGA applicable(dB)D(6:0)
    I2C_Write (97, 0x00);               // Attack time
    I2C_Write (98, 0x00);               // Decay time
    I2C_Write (99, 0x00);               // Noise Debounce time D(4:0)
    I2C_Write (100, 0x00);
  
  }
    

}








//******************************************************************************
// ��������� � ��������� ����������� ���. �������� �� ����� ������ ����� �� ������
//******************************************************************************
void codec_DRC_left (uint8_t Enable){
//uint8_t Tmp;  
//
  if (Enable != 0) {

    // ��������� ������ �� ��
    I2C_Write (0x00, 0x00);             // Initialize to Page 0
    I2C_Write (68, 0xF2);               // 
    I2C_Write (69, 0x00);               // 
    I2C_Write (70, 0xE2);               // 

  } else {
    // ����������
    I2C_Write (0x00, 0x00);             // Initialize to Page 0
    I2C_Write (68, 0x00);               // 
    I2C_Write (69, 0x00);               // 
    I2C_Write (70, 0x00);               // 
  }
    


}

//******************************************************************************
// ENF OF FILE
//******************************************************************************
