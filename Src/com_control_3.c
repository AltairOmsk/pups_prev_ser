/*
* Copyright (C) 2018 Yuri Ryzhenko <Y.Ryzhenko@hi-tech.org>, Aleksey Kirsanov <a.kirsanov@iva-tech.ru>
* All rights reserved
*
* File Name  : com_control.c
* Description: DSP processing
*/
//******************************************************************************
// ������ include: ����� ������������ ������������ ���� � ������
//******************************************************************************
#include "com_control.h" // �������� ���� ��������� ��� ������ ������
//******************************************************************************
// ������ ����������� ����������, ������������ � ������
//******************************************************************************
//------------------------------------------------------------------------------
// ����������
//------------------------------------------------------------------------------
//char GlobalVar1;
//char GlobalVar2;
//...
//------------------------------------------------------------------------------
// ���������
//------------------------------------------------------------------------------
//static char LocalVar1;
//static char LocalVar2;
//...
//******************************************************************************
// ������ ���������� ��������� �������
//******************************************************************************
//void local_func1 (void);
//void local_func2 (void);
//...
//******************************************************************************
// ������ �������� ������� (������� ����������, ����� ���������)
//******************************************************************************
/*
��������� ����-�������
������: �������
        ��������
        ��� � �������
        S-����
        ������� ��� ���� ����������������
*/


//******************************************************************************
//   ��������� ������������� ����������
//******************************************************************************
void RC_start_Init (void){
  R.DDS_RX.Freq                 = 10000;                                        // ������� ������
  R.DDS_TX.Freq                 = 10000;                                        // ������� ��������
  R.RC.S_meterIntrval           = S_METER_INTERVAL;
  R.RXBW                        = RXBW_WIDE;
  R.Parott_En                   = 0;                                            // ������� ��������
  R.Test_tone_freq              = 1000;                                         // �������� ��� �������� ������� ��� ����� �������
  R.TX_pwr                      = 100;                                          // �������� �������� ����������� 0-100%
  R.AGC.Enable                  = 0;                                            // ��� �� ��������� 1-�������� 0-���������
  R.TXAGC.Enable                = 0;                                            // ��� �� ��������� 1-�������� 0-���������
  
  
  R.Gain_MICPGAR                = 20;                                           // 1/2dB   RX
  R.Gain_ADCR                   = 0;                                            // 1/2dB
  R.Gain_DACR                   = 0;                                            // 1/2dB
  R.Gain_HPR                    = 0;                                            // dB
  
  R.Gain_MICPGAL                = 20;                                           // 1/2dB   TX
  R.Gain_ADCL                   = 0;                                            // 1/2dB
  R.Gain_DACL                   = 0;                                            // 1/2dB
  R.Gain_HPL                    = 00;                                           // dB
 
  R.SD_Rec_idx                  = 0;
  
  
  R.EqRx_En                     = 0;
  R.EqTx_En                     = 0;
                                                                                // k1, k2, Gain
//  allpass_set_parameters(&R.EqTxLow,  -0.9166575,       0.7567788461538,        0.9899714708138);
//  allpass_set_parameters(&R.EqTxMed,  -0.379625833333,  -0.176105769231,        0.1680282664833);
//  allpass_set_parameters(&R.EqTxHigh, 0.3703666666667,  0.680625,               4.1287272487189);
//  
//  allpass_set_parameters(&R.EqRxLow,  -0.9166575,       0.7567788461538,        0.9899714708138);
//  allpass_set_parameters(&R.EqRxMed,  -0.379625833333,  -0.176105769231,        0.1680282664833);
//  allpass_set_parameters(&R.EqRxHigh, 0.3703666666667,  0.680625,               4.1287272487189);
  
  
  allpass_set_parameters(&R.EqTxLow,  0,0,1);
  allpass_set_parameters(&R.EqTxMed,  0,0,1);
  allpass_set_parameters(&R.EqTxHigh, 0,0.668,7);
  
  allpass_set_parameters(&R.EqRxLow,  0,0,1);
  allpass_set_parameters(&R.EqRxMed,  0,0,1);
  allpass_set_parameters(&R.EqRxHigh, 0,0.668,3);
  
}

//******************************************************************************
//   ����� ������ ������� � ����������
//******************************************************************************
void UART1_IT_routine (void) {

  R.U1.Buf[R.U1.Idx] = USART1->DR;                                              // �������� ������ �� ���� � ��

  if (R.U1.Buf[R.U1.Idx] == 0x0A) {                                             // ���� ��������� �������� ������ - LF
    if (R.U1.Buf[R.U1.Idx - 1] == 0x0D) {                                       // � ���������� CR,
      
      if (R.U1.Idx < 5) {                                                       // ������ �� ��������� ������� �������� ���������
        R.U1.Idx = 0;
        return;
      }

      R.U1.Buf[R.U1.Idx + 1] = 0;                                               // ���������� ���������� ������ ���������  
      R.U1.Idx = 0;   
      R.U1.New_string = 1;                                                      // ������ ���� ������ ������ ���������      
      return;                                                                   // ��� �� �� ���������� ������ ������ � ����� ����������� ����������

    }
  } 
  R.U1.Idx++;
}



//******************************************************************************
// ��������� ����� ������ � ������
//******************************************************************************
void USART1_read_command (void){

  if (R.U1.New_string == 0)    return;                                          // ������� ���� ��� ����� ������                 
  
  //----------------------------------------------------------------------------
  R.U1.New_string = 0;                                                          // �������� ����� ������
  
char Tmp_buf[12];
char *Ptr;
uint8_t i, ii;
int32_t Tmp32;


  //----------------------------------------------------------------------------
  Ptr=strstr(R.U1.Buf, "AT+RESET");                                            // AT+RESET
  if (Ptr){
    sprintf(R.TmpStr, "+RESET\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    NVIC_SystemReset();
  }
    
    
  //----------------------------------------------------------------------------
  Ptr=strstr(R.U1.Buf, "AT+PWROFF");                                            // AT+PWROFF
  if (Ptr){
    sprintf(R.TmpStr, "+TRX POWER OFF\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    __PWR_ON_LOCK_OFF;                                                          // ������� ���������� ��������� �������
  }  
  
  

  //----------------------------------------------------------------------------
  Ptr=strstr(R.U1.Buf, "AT+FLASHLOG=");                                            // AT+FLASHLOG=
  if (Ptr){
    Ptr +=strlen("AT+FLASHLOG=");   
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; }
    while (*Ptr != '\r'){                                                       // Enable / Disable
        if (ii>5) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[4]=0;
    }
    Ptr++;                                                                      // ������������� �������
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32>=0) && (Tmp32<=1)){
      R.FlashLogEn = Tmp32;      
    }
    
    (R.FlashLogEn == 1) ? (sprintf(R.TmpStr, "FLASH LOG ENABLED\r\n")) : (sprintf(R.TmpStr, "FLASH LOG DISABLED\r\n"));
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
  }
  
          
  //----------------------------------------------------------------------------
  Ptr=strstr(R.U1.Buf, "AT+SETDEF");                                            // AT+SETDEF Set default settings to BAT backup domen
  if (Ptr){
    sprintf(R.TmpStr, "+SETDEF\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    load_default_settings();
  }


  //----------------------------------------------------------------------------
  Ptr=strstr(R.U1.Buf, "AT+NOTE");                                              // AT+NOTE
  if (Ptr){
    memcpy(R.Note, R.U1.Buf+2, 255);
    R.NewNote = 1;
    sprintf(R.TmpStr, "OK\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
  }


  //----------------------------------------------------------------------------
  Ptr=strstr(R.U1.Buf, "AT+SMETER");                                            // AT+SMETER
  if (Ptr){
    Ptr +=strlen("AT+SMETER");
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; } 
    while (*Ptr != '\r'){
        if (ii>6) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[5]=0;
    }
    R.RC.S_meterIntrval = atoi(Tmp_buf);
    S_meter_Send();
  }
  
  
  //----------------------------------------------------------------------------
//  Ptr=strstr(R.U1.Buf, "AT+FFTBIN");                                            // AT+FFTBIN
//  if (Ptr){
//    Ptr +=strlen("AT+FFTBIN");
//    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; } 
//    while (*Ptr != '\r'){
//        if (ii>6) { break; }
//        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
//        Tmp_buf[5]=0;
//    }
//    sprintf(R.TmpStr, "FFT MAX BIN %d Hz Amp=%d\r\n", R.FFT_MaxValIdx*(48828/1024), R.FFT_MaxVal); // bin*Fd/2 / FFTlen
//    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
//  }
  
  
    
  //----------------------------------------------------------------------------
  Ptr=strstr(R.U1.Buf, "AT+TRXMODE=");                                            // AT+TRXMODE=<mode>
  if (Ptr){
    Ptr +=strlen("AT+TRXMODE=");
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; } 
    while (*Ptr != '\r'){
        if (ii>6) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[5]=0;
    }
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32>=0) && (Tmp32 < TXRX_MODE_CNT)){                                 // ������� ���������� ���� TXRX_MODE_e
      R.TXRX_Mode = (TXRX_MODE_e)Tmp32;
      set_TXRX_mode(R.TXRX_Mode);
      R.RecBufIdx = 0;
      sprintf(R.TmpStr, "+TXRXMODE %d\r\n", R.TXRX_Mode);
      HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    }
    else {
      sprintf(R.TmpStr, "ERROR\r\n");
      HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    }
  }
  


  //----------------------------------------------------------------------------
  Ptr=strstr(R.U1.Buf, "AT+TXPWR=");                                            // AT+TXPWR=<mode>
  if (Ptr){
    Ptr +=strlen("AT+TXPWR=");
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; } 
    while (*Ptr != '\r'){
        if (ii>6) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[5]=0;
    }
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32>=0) && (Tmp32 <= 100)){                                          // 0-100%
      R.TX_pwr = Tmp32;
      sprintf(R.TmpStr, "+TXPWR %d%%\r\n", R.TX_pwr);
      HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    }
    else {
      sprintf(R.TmpStr, "ERROR\r\n");
      HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    }
  }
  

  //----------------------------------------------------------------------------
  Ptr=strstr(R.U1.Buf, "AT+FREQ=");                                             // AT+FREQ=
  if (Ptr){
    Ptr +=strlen("AT+FREQ=");
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; } 
    while (*Ptr != ','){                                                        // ������������ ������� ������
        if (ii>8) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[7]=0;
    }
    Ptr++;                                                                      // ������������� �������
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32 > 0) && (Tmp32 <= 20000)){
      R.DDS_RX.Freq = Tmp32;
    }
    
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; } 
    while (*Ptr != '\r'){                                                       // ������������ ������� ��������
        if (ii>8) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[7]=0;
    }
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32 > 0) && (Tmp32 <= 20000)){
      R.DDS_TX.Freq = Tmp32;
    }
    sprintf(R.TmpStr, "+FREQ\r\n RX freq = %d Hz\r\n TX freq = %d Hz\r\n", R.DDS_RX.Freq, R.DDS_TX.Freq);
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
  }
  
  //----------------------------------------------------------------------------
  Ptr=strstr(R.U1.Buf, "AT+FREQUP=");                                             // AT+FREQUP= <UP>
  if (Ptr){
    Ptr +=strlen("AT+FREQUP=");
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; } 
    while (*Ptr != '\r'){
        if (ii>8) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[7]=0;
    }
    Tmp32 = atoi(Tmp_buf);
    if (Tmp32 <1000){
      R.DDS_RX.Freq = R.DDS_RX.Freq+Tmp32;
      R.DDS_TX.Freq = R.DDS_TX.Freq+Tmp32;
      sprintf(R.TmpStr, "+FREQ\r\n RX freq = %d Hz\r\n TX freq = %d Hz\r\n", R.DDS_RX.Freq, R.DDS_TX.Freq);
      HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
      sprintf(R.TmpStr, " TX current = %d mA\r\n", R.TX_current_Peack);         // TX Current 
      HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    }
    
  }
  
  //----------------------------------------------------------------------------
  Ptr=strstr(R.U1.Buf, "AT+FREQDOWN=");                                             // AT+FREQDOWN=<step>
  if (Ptr){
    Ptr +=strlen("AT+FREQDOWN=");
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; } 
    while (*Ptr != '\r'){
        if (ii>8) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[7]=0;
    }
    Tmp32 = atoi(Tmp_buf);
    if (Tmp32 <1000){
      R.DDS_RX.Freq = R.DDS_RX.Freq-Tmp32;
      R.DDS_TX.Freq = R.DDS_TX.Freq-Tmp32;
      sprintf(R.TmpStr, "+FREQ\r\n RX freq = %d Hz\r\n TX freq = %d Hz\r\n", R.DDS_RX.Freq, R.DDS_TX.Freq);
      HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
      sprintf(R.TmpStr, " TX current = %d mA\r\n", R.TX_current_Peack);         // TX Current 
      HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    }
    
  }
  
  
  //----------------------------------------------------------------------------
  Ptr=strstr(R.U1.Buf, "AT+RXBW=");                                             // AT+RXBW=<bw>
  if (Ptr){
    Ptr +=strlen("AT+RXBW=");
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; } 
    while (*Ptr != '\r'){
        if (ii>5) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[4]=0;
    }
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32 >=0 ) && (Tmp32 < 4)){
      R.RXBW = (RXBW_e)Tmp32;
        switch (R.RXBW){
        case RXBW_NARROW:
          sprintf(R.TmpStr, "+RXBW 200 Hz\r\n");
          HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
          break;
        case RXBW_MEDIUM:
          sprintf(R.TmpStr, "+RXBW 1 kHz\r\n");
          HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
          break;
        case RXBW_WIDE:
          sprintf(R.TmpStr, "+RXBW 3 kHz\r\n");
          HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
          break;   
        case RXBW_BYPASS:
          sprintf(R.TmpStr, "+RXBW BYPASS MODE\r\n");
          HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
          break;    
        }
    }
    else {
        sprintf(R.TmpStr, "ERROR\r\n", R.DDS_RX.Freq, R.DDS_TX.Freq);
        HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    }
    
  }
  
  
  //----------------------------------------------------------------------------
  Ptr=strstr(R.U1.Buf, "AT+RXGAIN=");                                           // AT+RXGAIN=
  if (Ptr){
    Ptr +=strlen("AT+RXGAIN=");
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; } 
    while (*Ptr != ','){                                                        // MIC PGA
        if (ii>5) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[4]=0;
    }
    Ptr++;                                                                      // ������������� �������
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32>=0) && (Tmp32<=95)){
      R.Gain_MICPGAR = Tmp32;      
    }
    
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; }
    while (*Ptr != ','){                                                        // ADC
        if (ii>5) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[4]=0;
    }
    Ptr++;                                                                      // ������������� �������
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32>=-24) && (Tmp32<=40)){
      R.Gain_ADCR = Tmp32;      
    }
    
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; }
    while (*Ptr != ','){                                                        // DAC
        if (ii>5) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[4]=0;
    }
    Ptr++;                                                                      // ������������� �������
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32>=-127) && (Tmp32<=40)){
      R.Gain_DACR = Tmp32;      
    }
    
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; }
    while (*Ptr != '\r'){                                                       // HPR
        if (ii>5) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[4]=0;
    }
    Ptr++;                                                                      // ������������� �������
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32>=-6) && (Tmp32<=29)){
      R.Gain_HPR = Tmp32;      
    }
    
    set_RX_Gain_codec();                                                        // �������� � ����� ������������� ��������
    set_TX_Gain_codec();
    
    sprintf(R.TmpStr, "+RXGAIN\r\n MIC PGA %d (0 95 (1/2 dB))\r\n ADC     %d (-24 40 (1/2 dB))\r\n DAC     %d (-127 48 (1/2 dB))\r\n HPR     %d (-6 29 dB)\r\n", R.Gain_MICPGAR, R.Gain_ADCR, R.Gain_DACR, R.Gain_HPR);
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
  }
  


  //----------------------------------------------------------------------------
  Ptr=strstr(R.U1.Buf, "AT+EQSETH=");                                           // AT+EQSETH=   (������� ��, ������ ��, ������� ���)
  if (Ptr){
    Ptr +=strlen("AT+EQSETH=");
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; } 
    while (*Ptr != ','){                                                        // Freq
        if (ii>6) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[5]=0;
    }
    Ptr++;                                                                      // ������������� �������
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32>=0) && (Tmp32<=4000)){
      R.EqRxHigh.Freq_Hz = Tmp32;      
    }
    
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; }
    while (*Ptr != ','){                                                        // Bandwidth
        if (ii>6) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[5]=0;
    }
    Ptr++;                                                                      // ������������� �������
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32>=0) && (Tmp32<=4000)){
      R.EqRxHigh.BW_Hz = Tmp32;      
    }
    
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; }
    while (*Ptr != '\r'){                                                       // Gain
        if (ii>5) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[4]=0;
    }
    Ptr++;                                                                      // ������������� �������
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32>=-100) && (Tmp32<=100)){
      R.EqRxHigh.Gain_dBv = Tmp32;    
    }

    allpass_convert_parameters(&R.EqRxHigh);                                    // ������������� ��������� ��������� � ������������ �������
    
    sprintf(R.TmpStr, "+EQSETH\r\n Freq %d Hz, BW %d Hz, Gain %d dBv\r\n", R.EqRxHigh.Freq_Hz, R.EqRxHigh.BW_Hz, R.EqRxHigh.Gain_dBv);
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
  }
  
  
  
  
  //----------------------------------------------------------------------------
  Ptr=strstr(R.U1.Buf, "AT+RXEQU=");                                            // AT+RXEQU=
  if (Ptr){
    Ptr +=strlen("AT+RXEQU=");   
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; }
    while (*Ptr != '\r'){                                                       // Enable / Disable
        if (ii>5) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[4]=0;
    }
    Ptr++;                                                                      // ������������� �������
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32>=0) && (Tmp32<=1)){
      R.EqRx_En = Tmp32;      
    }
    
    (R.EqRx_En == 1) ? (sprintf(R.TmpStr, "RX EQUALISER ENABLE\r\n")) : (sprintf(R.TmpStr, "RX EQUALISER DISABLE\r\n"));
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
  }


  
  
  //----------------------------------------------------------------------------
  Ptr=strstr(R.U1.Buf, "AT+RXAGC=");                                            // AT+RXAGC=
  if (Ptr){
    Ptr +=strlen("AT+RXAGC=");   
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; }
    while (*Ptr != '\r'){                                                       // HPR
        if (ii>5) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[4]=0;
    }
    Ptr++;                                                                      // ������������� �������
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32>=0) && (Tmp32<=1)){
      R.AGC.Enable = Tmp32;      
    }
    
    (R.AGC.Enable == 1) ? (sprintf(R.TmpStr, "RX AGC ENABLE\r\n")) : (sprintf(R.TmpStr, "RX AGC DISABLE\r\n"));
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
  }
  
  
  //----------------------------------------------------------------------------
  Ptr=strstr(R.U1.Buf, "AT+TXAGC=");                                            // AT+TXAGC=
  if (Ptr){
    Ptr +=strlen("AT+TXAGC=");   
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; }
    while (*Ptr != '\r'){                                                       // HPR
        if (ii>5) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[4]=0;
    }
    Ptr++;                                                                      // ������������� �������
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32>=0) && (Tmp32<=1)){
      R.TXAGC.Enable = Tmp32;   
      CODEC_Init();
    }
    (R.TXAGC.Enable == 1) ? (sprintf(R.TmpStr, "TX AGC ENABLE\r\n")) : (sprintf(R.TmpStr, "TX AGC DISABLE\r\n"));
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
  }
  
  
  
  
  //----------------------------------------------------------------------------
  Ptr=strstr(R.U1.Buf, "AT+TXGAIN=");                                           // AT+TXGAIN=
  if (Ptr){
    Ptr +=strlen("AT+TXGAIN=");
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; } 
    while (*Ptr != ','){                                                        // MIC PGA
        if (ii>5) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[4]=0;
    }
    Ptr++;                                                                      // ������������� �������
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32>=0) && (Tmp32<=95)){
      R.Gain_MICPGAL = Tmp32;      
    }
    
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; }
    while (*Ptr != ','){                                                        // ADC
        if (ii>5) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[4]=0;
    }
    Ptr++;                                                                      // ������������� �������
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32>=-24) && (Tmp32<=40)){
      R.Gain_ADCL = Tmp32;      
    }
    
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; }
    while (*Ptr != ','){                                                        // DAC
        if (ii>5) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[4]=0;
    }
    Ptr++;                                                                      // ������������� �������
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32>=-127) && (Tmp32<=40)){
      R.Gain_DACL = Tmp32;      
    }
    
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; }
    while (*Ptr != '\r'){                                                       // HPR
        if (ii>5) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[4]=0;
    }
    Ptr++;                                                                      // ������������� �������
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32>=-6) && (Tmp32<=29)){
      R.Gain_HPL = Tmp32;      
    }
                                                       
    set_TX_Gain_codec();                                                        // �������� � ����� ������������� ��������
    
    sprintf(R.TmpStr, "+TXGAIN\r\n MIC PGA %d (0 95 (1/2 dB))\r\n ADC     %d (-24 40 (1/2 dB))\r\n DAC     %d (-127 48 (1/2 dB))\r\n HPR     %d (-6 29 dB)\r\n", R.Gain_MICPGAL, R.Gain_ADCL, R.Gain_DACL, R.Gain_HPL);
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
  }
  
  
  //----------------------------------------------------------------------------
  Ptr=strstr(R.U1.Buf, "AT+STATUS");                                            // AT+STATUS
  if (Ptr){
//    Ptr +=strlen("AT+STATUS");
//    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; } 
//    while (*Ptr != '\r'){
//        if (ii>6) { break; }
//        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
//        Tmp_buf[5]=0;
//    }
//    R.RC.S_meterIntrval = atoi(Tmp_buf);
    Status_Send();
  }
  


  //----------------------------------------------------------------------------                                                                              
  Ptr=strstr(R.U1.Buf, "AT+DATE=");                                             // AT+DATE=<day>, <month>, <year>
  if (Ptr){
    Ptr +=strlen("AT+DATE=");
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; }                                  // Day
    while (*Ptr != ','){
        if (ii>6) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[5]=0;
    }
    Ptr++;                                                                      // ������������� �������
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32>0) && (Tmp32<=31)) R.GDate.Date = Tmp32;
    
    
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; }                                  // Month
    while (*Ptr != ','){
        if (ii>6) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[5]=0;
    }
    Ptr++;
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32>0) && (Tmp32<=12)) R.GDate.Month = Tmp32;
    
    
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; }                                  // Year
    while (*Ptr != '\r'){
        if (ii>6) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[5]=0;
    }
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32>17) && (Tmp32<=99)) R.GDate.Year = Tmp32;
    
    HAL_RTC_SetDate(&hrtc, &(R.GDate), RTC_FORMAT_BIN);                         // �������� ����/�����
    HAL_RTC_SetTime(&hrtc, &(R.GTime), RTC_FORMAT_BIN);
  }
  
  
  
  
  //----------------------------------------------------------------------------                                                                              
  Ptr=strstr(R.U1.Buf, "AT+TIME=");                                             // AT+TIME=<hour>, <min>
  if (Ptr){
    Ptr +=strlen("AT+TIME=");
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; }                                  // Hour
    while (*Ptr != ','){
        if (ii>6) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[5]=0;
    }
    Ptr++;                                                                      // ������������� �������
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32>=0) && (Tmp32<24)) R.GTime.Hours = Tmp32;
    
    
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; }                                  // Min
    while (*Ptr != '\r'){
        if (ii>6) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[5]=0;
    }
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32>=0) && (Tmp32<60)) R.GTime.Minutes = Tmp32;
    R.GTime.Seconds = 0;
    
    HAL_RTC_SetDate(&hrtc, &(R.GDate), RTC_FORMAT_BIN);                         // �������� ����/�����
    HAL_RTC_SetTime(&hrtc, &(R.GTime), RTC_FORMAT_BIN);
  }
  
  
  //----------------------------------------------------------------------------
  
  save_settings();                                                              // ���������� �������� � ���������� �����
  
  for (ii=0;ii<255;ii++) R.U1.Buf[ii]=0;                                        // �������� ������ ����� �������
  
}




//******************************************************************************
// ��������� � ��� ���� ��������� S-�����
//******************************************************************************
void S_meter_Send (void){
  sprintf(R.TmpStr, "+SMETER =%d,\r\n G=%d, S_uV=%d uV\r\n", R.S_meter, R.RX_gain, R.S_meter_mkv);
  //CDC_Transmit_FS(Str, strlen(R.TmpStr));
  HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
}   



//******************************************************************************
// ��������� � ��� ���� ������
//******************************************************************************
void Status_Send (void){

    // DevID, Date and time
    sprintf(R.TmpStr, "+STATUS\r\n DevID %d%d\r\n Device Date %2d.%02d.20%02d   %2d:%02d:%02d\r\n", DevID.b15_0, DevID.b16_31, R.GDate.Date, R.GDate.Month, R.GDate.Year, R.GTime.Hours, R.GTime.Minutes, R.GTime.Seconds);
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
    // FW revision
    sprintf(R.TmpStr, "%s\r\n", FW_REV);
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
  
    // Bat power  
    sprintf(R.TmpStr, "BAT pwr = %.2f V\r\n", (float)R.PWR_main_mV/100);
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
    // Enable log
    if (R.FlashLogEn == 1){
      sprintf(R.TmpStr, "Logging to uSD card enabled\r\n");
    }
    else {
      sprintf(R.TmpStr, "Logging to uSD card disabled\r\n");
    }
    R.WriteSD = 0;
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);

    // WriteSD 
    if (R.WriteSD == 1){
      sprintf(R.TmpStr, "Log is writinig on uSD card\r\n");
    }
    else {
      sprintf(R.TmpStr, "uSD card error\r\n");
    }
    R.WriteSD = 0;
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
    
    // RX Freq TX Freq  
    sprintf(R.TmpStr, " RX freq = %d Hz\r\n TX freq = %d Hz\r\n", R.DDS_RX.Freq, R.DDS_TX.Freq);
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
    // CODEC Gain  
    sprintf(R.TmpStr, "Codec RX path gain:  ");
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    sprintf(R.TmpStr, "%d %d %d %d\r\n", R.Gain_MICPGAR, R.Gain_ADCR, R.Gain_DACR, R.Gain_HPR);
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
    sprintf(R.TmpStr, "Codec TX path gain:  ");
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    sprintf(R.TmpStr, "%d %d %d %d\r\n", R.Gain_MICPGAL, R.Gain_ADCL, R.Gain_DACL, R.Gain_HPL);
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
    // AGC status
    (R.AGC.Enable == 1) ? (sprintf(R.TmpStr, "RX AGC ENABLE\r\n")) : (sprintf(R.TmpStr, "RX AGC DISABLE\r\n"));
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    (R.TXAGC.Enable == 1) ? (sprintf(R.TmpStr, "TX AGC ENABLE\r\n")) : (sprintf(R.TmpStr, "TX AGC DISABLE\r\n"));
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
    // RX Bandwidth 
    switch (R.RXBW){
      case RXBW_NARROW:
        sprintf(R.TmpStr, " RXBW 200 Hz\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
        break;
      case RXBW_MEDIUM:
        sprintf(R.TmpStr, " RXBW 1 kHz\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
        break;
      case RXBW_WIDE:
        sprintf(R.TmpStr, " RXBW 3 kHz\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
        break;   
      }
    
    //RX Equaliser
    if (R.EqRx_En == 1){
      sprintf(R.TmpStr, "RX equaliser ON\r\n");
    }
    else {
      sprintf(R.TmpStr, "RX equaliser OFF\r\n");
    }
    R.WriteSD = 0;
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
    
    //TX Equaliser
    if (R.EqTx_En == 1){
      sprintf(R.TmpStr, "TX equaliser ON\r\n");
    }
    else {
      sprintf(R.TmpStr, "TX equaliser OFF\r\n");
    }
    R.WriteSD = 0;
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
  

  // TX Current  
  sprintf(R.TmpStr, " TX current = %d mA  at %d%% output POWER\r\n", R.TX_current_Peack*1346/500, R.TX_pwr); 
  HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);

    
    
    
//    // CODEC gain
//    sprintf(R.TmpStr, " MIC PGA Gain = %d db/2 \r\n ADC Gain = %d dB/2\r\n DAC Gain = %d dB\r\n Head Phone Gain = %d dB", 
//            R.DDS_RX.Freq, R.DDS_TX.Freq);
//    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
}  








//******************************************************************************
//   ��������� � ����� ������ ������� ��������� (20 int32 ��������)
//******************************************************************************
/* ��������� ���������:
- ������� ��������� 16 ���
- ������� ����������� 16 ���
- �������� � ������ ��������� 8+8+8+8 ���
- �������� � ������ ����������� 8+8+8+8 ���
- �������� ����������� 8 ���
- ��������� ���� ����� ����������� ������� ������ �������� 
- ������ ��������� �� ����� 4 ���

�������:
- ���������/���������� ���

*/
void save_settings (void) {
uint32_t ToSave;

  ToSave  = R.DDS_RX.Freq;                                                      // ������� 
  ToSave += R.DDS_TX.Freq << 16;
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, ToSave);
  
  
  ToSave  = R.Gain_MICPGAL;                                                     // Left PGA and ADC
  ToSave += R.Gain_ADCL << 16;
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, ToSave);
  
  ToSave  = R.Gain_DACL;                                                        // Left DAC and HPL
  ToSave += R.Gain_HPL << 16;
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, ToSave);
  
  ToSave  = R.Gain_MICPGAR;                                                     // Right PGA and ADC
  ToSave += R.Gain_ADCR << 16;
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, ToSave);
  
  ToSave  = R.Gain_DACR;                                                        // Right DAC and HPR
  ToSave += R.Gain_HPR << 16;
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR4, ToSave);
  
  ToSave  = R.TX_pwr & 0xFF;                                                    // TX Power
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR5, ToSave);
  
  //--- FLAGS   ----------------------------------------------------------------
  ToSave = 0;
  ToSave |= (R.AGC.Enable       << 0);
  ToSave |= (R.EqRx_En          << 1);
  ToSave |= (R.EqTx_En          << 2);
  ToSave |= (R.FlashLogEn       << 3);
  ToSave |= (R.TXAGC.Enable     << 4);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR19, ToSave);
}


void load_default_settings (void) {                                             // ��������� ��������� ���������
    RC_start_Init();
    save_settings();
    restore_settings();
}
                                                
//******************************************************************************
//   ������������ ��������� �� ����� ������
//******************************************************************************
void restore_settings (void){
uint32_t In;  
   In = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0);                                 // �������
   R.DDS_RX.Freq = (In & 0xFFFF);  R.DDS_TX.Freq = (In >> 16);
   
   In = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);                                 // Left PGA and ADC
   R.Gain_MICPGAL = (In & 0xFFFF);                                                    
   R.Gain_ADCL    = (In >> 16);

   In = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);                                 // Left DAC and HPL
   R.Gain_DACL = (In & 0xFFFF);                                                       
   R.Gain_HPL  = (In >> 16);

   In = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3);                                 // Right PGA and ADC
   R.Gain_MICPGAR = (In & 0xFFFF);                                                    
   R.Gain_ADCR    = (In >> 16);

   In = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR4);                                 // Right DAC and HPR
   R.Gain_DACR = (In & 0xFFFF);                                                       
   R.Gain_HPR  = (In >> 16);
   
   In = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR5);                                 // TX power
   R.TX_pwr = (In & 0xFF);                                                       
   
//---   FLAGS   ----------------------------------------------------------------
  In = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR19);
  
  R.AGC.Enable          = ((In & (1 << 0)) != 0) ? (1) : (0);
  R.EqRx_En             = ((In & (1 << 1)) != 0) ? (1) : (0);
  R.EqTx_En             = ((In & (1 << 2)) != 0) ? (1) : (0);
  R.FlashLogEn          = ((In & (1 << 3)) != 0) ? (1) : (0);   
  R.TXAGC.Enable        = ((In & (1 << 4)) != 0) ? (1) : (0);
}


//******************************************************************************
// ENF OF FILE
//******************************************************************************
