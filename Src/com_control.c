/*
* Copyright (C) 2018 Yuri Ryzhenko <Y.Ryzhenko@hi-tech.org>, Aleksey Kirsanov <a.kirsanov@iva-tech.ru>
* All rights reserved
*
* File Name  : com_control.c
* Description: DSP processing
*/
//******************************************************************************
// Секция include: здесь подключается заголовочный файл к модулю
//******************************************************************************
#include "com_control.h" // Включаем файл заголовка для нашего модуля
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
//static char LocalVar1;
//static char LocalVar2;
//...
//******************************************************************************
// Секция прототипов локальных функций
//******************************************************************************
//void local_func1 (void);
//void local_func2 (void);
//...
//******************************************************************************
// Секция описания функций (сначала глобальных, потом локальных)
//******************************************************************************
/*
Установка даты-времени
Статус: Частота
        Мощность
        Ток в антенне
        S-метр
        Уровень АРУ либо чувствительность
*/


//******************************************************************************
//   Стартовая инициализация переменных
//******************************************************************************
void RC_start_Init (void){
  R.DDS_RX.Freq                 = 9000;                                         // Частота приема
  R.DDS_TX.Freq                 = 9000;                                         // Частота передачи
  R.RC.S_meterIntrval           = S_METER_INTERVAL;
  R.RXBW                        = RXBW_WIDE;
  R.Parott_En                   = 0;                                            // Попугай выключен
  R.Test_tone_freq              = 1000;                                         // Тестовый тон звуковая частота без учета несущей
  R.TX_pwr                      = 100;                                          // Выходная мощность передатчика 0-100%
  //R.AGC.Enable                  = 0;                                            // АРУ soft rx по умолчанию 1-включена 0-выключена
  R.TXAGC.Enable                = 1;                                            // АРУ по умолчанию 1-включена 0-выключена
  R.RXAGC.Enable                = 0;                                            // АРУ по умолчанию 1-включена 0-выключена
  R.Tone_Puls                   = 0;                                            // Тон по умолчанию непрерывный
  
  
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
//   Прием нового символа в прерывании
//******************************************************************************
void UART1_IT_routine (void) {

  R.U1.Buf[R.U1.Idx] = USART1->DR;                                              // Получаем данные из УАРТ и со

  if (R.U1.Buf[R.U1.Idx] == 0x0A) {                                             // Если последний ПРИНЯТЫЙ СИМВОЛ - LF
    if (R.U1.Buf[R.U1.Idx - 1] == 0x0D) {                                       // а предидущий CR,
      
      if (R.U1.Idx < 5) {                                                       // Защита от обработки слишком кооткого сообщения
        R.U1.Idx = 0;
        return;
      }

      R.U1.Buf[R.U1.Idx + 1] = 0;                                               // Безусловно оканчиваем строку сообщения  
      R.U1.Idx = 0;   
      R.U1.New_string = 1;                                                      // Ставим флаг приема нового сообщения      
      return;                                                                   // Что бы не прибавлять индекс буфера в конце обработчика прерывания

    }
  } 
  R.U1.Idx++;
}



//******************************************************************************
// Обработка новой строки в буфере
//******************************************************************************
void USART1_read_command (void){

  if (R.U1.New_string == 0)    return;                                          // Выходим если нет новой строки                 
  
  //----------------------------------------------------------------------------
  R.U1.New_string = 0;                                                          // Начинаем поиск команд
  
char Tmp_buf[12];
char *Ptr;
uint8_t i, ii;
int32_t Tmp32, Tmp32_2;


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
    sprintf(R.TmpStr, "+TRX %d%d POWER OFF\r\n", DevID.b15_0, DevID.b16_31);
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    osDelay(200);
    __PWR_ON_LOCK_OFF;                                                          // Снимаем блокировку включения питания
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
    Ptr++;                                                                      // Перескакиваем запятую
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
    if ((Tmp32>=0) && (Tmp32 < TXRX_MODE_CNT)){                                 // Границы переменной типа TXRX_MODE_e
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
    while (*Ptr != ','){                                                        // Обрабатываем частоту приема
        if (ii>8) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[7]=0;
    }
    Ptr++;                                                                      // Перескакиваем запятую
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32 > 0) && (Tmp32 <= 20000)){
      R.DDS_RX.Freq = Tmp32;
    }
    
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; } 
    while (*Ptr != '\r'){                                                       // Обрабатыввем частоту передачи
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
    Ptr++;                                                                      // Перескакиваем запятую
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
    Ptr++;                                                                      // Перескакиваем запятую
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
    Ptr++;                                                                      // Перескакиваем запятую
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
    Ptr++;                                                                      // Перескакиваем запятую
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32>=-6) && (Tmp32<=29)){
      R.Gain_HPR = Tmp32;      
    }
    
    set_RX_Gain_codec();                                                        // Загрузка в кодек поготовленных значений
    set_TX_Gain_codec();
    
    sprintf(R.TmpStr, "+RXGAIN\r\n MIC PGA %d (0 95 (1/2 dB))\r\n ADC     %d (-24 40 (1/2 dB))\r\n DAC     %d (-127 48 (1/2 dB))\r\n HPR     %d (-6 29 dB)\r\n", R.Gain_MICPGAR, R.Gain_ADCR, R.Gain_DACR, R.Gain_HPR);
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
  }
  
  
  
  //===   AGC   ================================================================
  Ptr=strstr(R.U1.Buf, "AT+AGCSETL=");                                          // AT+AGCSETL=
  if (Ptr){
    Ptr +=strlen("AT+AGCSETL=");
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; } 
    while (*Ptr != ','){                                                        // Имя параметра
        if (ii>4) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[3]=0;
    }
    Ptr++;                                                                      // Перескакиваем запятую
    Tmp32 = atoi(Tmp_buf);                                                      // Получили номер имени параметра

    
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; }
    while (*Ptr != '\r'){                                                       // Значение параметра
        if (ii>5) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[4]=0;
    }
    Tmp32_2 = atoi(Tmp_buf);                                                    // Получение значения параметра
    
    switch (Tmp32){
      case 0:
        if ((Tmp32_2 >= 0) && (Tmp32_2 <= 1))   R.AGC_L.Enable      = Tmp32_2;  // Enable
        else goto EXIT;
      break;
      
      case 1:
        if ((Tmp32_2 >= 0) && (Tmp32_2 <= 7))   R.AGC_L.TargetLevel = Tmp32_2;  // TargetLevel
        else goto EXIT;
      break;
      
      case 2:
        if ((Tmp32_2 >= 0) && (Tmp32_2 <= 31)) R.AGC_L.AttackTime  = Tmp32_2;   // AttackTime
        else goto EXIT;
      break;
      
      case 3:
        if ((Tmp32_2 >= 0) && (Tmp32_2 <= 31)) R.AGC_L.DecayTime   = Tmp32_2;   // DecayTime
        else goto EXIT;
      break;
      
      case 4:
        if ((Tmp32_2 >= 0) && (Tmp32_2 <= 31)) R.AGC_L.NoiseThreshold=Tmp32_2;  // NoiseThreshold
        else goto EXIT;
      break;
      
      case 5:
        if ((Tmp32_2 >= 0) && (Tmp32_2 <= 116)) R.AGC_L.MAX_PGAgain  = Tmp32_2; // MAX_PGAgain
        else goto EXIT;
      break;
      
      case 6:
        if ((Tmp32_2 >= 0) && (Tmp32_2 <= 3))   R.AGC_L.Hysteresis   = Tmp32_2; // Hysteresis
        else goto EXIT;
      break;
      
      case 7:
        if ((Tmp32_2 >= 0) && (Tmp32_2 <= 31)) R.AGC_L.NoiseDebounce =Tmp32_2;  // NoiseDebounce
        else goto EXIT;
      break;
      
      case 8:
        if ((Tmp32_2 >= 0) && (Tmp32_2 <= 15)) R.AGC_L.SignalDebounce =Tmp32_2; // SignalDebounce
        else goto EXIT;
      break;
      
      default:                                                                  // Параметр вне допустимых границ
        sprintf(R.TmpStr, "ERROR: Parameter number is wrong (1-8)\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
        goto EXIT;
      break;
    }
    
    load_AGC_L_settings();
    
    sprintf(R.TmpStr, "+AGC_L:\r\n0.Enable               = %d\r\n", R.AGC_L.Enable);
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
    sprintf(R.TmpStr, "1.Target Level         = %d\r\n", R.AGC_L.TargetLevel);
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
    sprintf(R.TmpStr, "2.Attack Time          = %d\r\n", R.AGC_L.AttackTime);
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
    sprintf(R.TmpStr, "3.Decay Time           = %d\r\n", R.AGC_L.DecayTime);
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
    sprintf(R.TmpStr, "4.Noise Threshold      = %d\r\n", R.AGC_L.NoiseThreshold);
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
    sprintf(R.TmpStr, "5.Max PGA gain         = %d dB\r\n", R.AGC_L.MAX_PGAgain/2);
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
    sprintf(R.TmpStr, "6.Hysteresis           = %d\r\n", R.AGC_L.Hysteresis);
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
    sprintf(R.TmpStr, "7.Noise debounce time  = %d\r\n", R.AGC_L.NoiseDebounce);
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
    sprintf(R.TmpStr, "8.Signal debounce time = %d\r\n", R.AGC_L.SignalDebounce);
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
  }
  
  
  Ptr=strstr(R.U1.Buf, "AT+AGCSETR=");                                          // AT+AGCSETR=
  if (Ptr){
    Ptr +=strlen("AT+AGCSETR=");
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; } 
    while (*Ptr != ','){                                                        // Имя параметра
        if (ii>4) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[3]=0;
    }
    Ptr++;                                                                      // Перескакиваем запятую
    Tmp32 = atoi(Tmp_buf);                                                      // Получили номер имени параметра

    
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; }
    while (*Ptr != '\r'){                                                       // Значение параметра
        if (ii>5) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[4]=0;
    }
    Tmp32_2 = atoi(Tmp_buf);                                                    // Получение значения параметра
    
    switch (Tmp32){
      case 0:
        if ((Tmp32_2 >= 0) && (Tmp32_2 <= 1))   R.AGC_R.Enable      = Tmp32_2;  // Enable
        else goto EXIT;
      break;
      
      case 1:
        if ((Tmp32_2 >= 0) && (Tmp32_2 <= 7))   R.AGC_R.TargetLevel = Tmp32_2;  // TargetLevel
        else goto EXIT;
      break;
      
      case 2:
        if ((Tmp32_2 >= 0) && (Tmp32_2 <= 31)) R.AGC_R.AttackTime  = Tmp32_2;   // AttackTime
        else goto EXIT;
      break;
      
      case 3:
        if ((Tmp32_2 >= 0) && (Tmp32_2 <= 31)) R.AGC_R.DecayTime   = Tmp32_2;   // DecayTime
        else goto EXIT;
      break;
      
      case 4:
        if ((Tmp32_2 >= 0) && (Tmp32_2 <= 31)) R.AGC_R.NoiseThreshold=Tmp32_2;  // NoiseThreshold
        else goto EXIT;
      break;
      
      case 5:
        if ((Tmp32_2 >= 0) && (Tmp32_2 <= 116)) R.AGC_R.MAX_PGAgain  = Tmp32_2; // MAX_PGAgain
        else goto EXIT;
      break;
      
      case 6:
        if ((Tmp32_2 >= 0) && (Tmp32_2 <= 3))   R.AGC_R.Hysteresis   = Tmp32_2; // Hysteresis
        else goto EXIT;
      break;
      
      case 7:
        if ((Tmp32_2 >= 0) && (Tmp32_2 <= 31)) R.AGC_R.NoiseDebounce =Tmp32_2;  // NoiseDebounce
        else goto EXIT;
      break;
      
      case 8:
        if ((Tmp32_2 >= 0) && (Tmp32_2 <= 15)) R.AGC_R.SignalDebounce =Tmp32_2; // SignalDebounce
        else goto EXIT;
      break;
      
      default:                                                                  // Параметр вне допустимых границ
        sprintf(R.TmpStr, "ERROR: Parameter number is wrong (1-8)\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
        goto EXIT;
      break;
    }
    
    load_AGC_R_settings();
    
    sprintf(R.TmpStr, "+AGC_R:\r\nEnable               = %d\r\n", R.AGC_R.Enable);
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
    sprintf(R.TmpStr, "Target Level         = %d\r\n", R.AGC_R.TargetLevel);
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
    sprintf(R.TmpStr, "Attack Time          = %d\r\n", R.AGC_R.AttackTime);
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
    sprintf(R.TmpStr, "Decay Time           = %d\r\n", R.AGC_R.DecayTime);
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
    sprintf(R.TmpStr, "Noise Threshold      = %d\r\n", R.AGC_R.NoiseThreshold);
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
    sprintf(R.TmpStr, "Max PGA gain         = %2f dB\r\n", R.AGC_R.MAX_PGAgain);
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
    sprintf(R.TmpStr, "Hysteresis           = %d\r\n", R.AGC_R.Hysteresis);
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
    sprintf(R.TmpStr, "Noise debounce time  = %d\r\n", R.AGC_R.NoiseDebounce);
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
    sprintf(R.TmpStr, "Signal debounce time = %d\r\n", R.AGC_R.SignalDebounce);
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
    
    
  }
  
  
  //============================================================================
  
  
  
  
  
  
  
  
  
  //----------------------------------------------------------------------------
  Ptr=strstr(R.U1.Buf, "AT+USB\r");                                             // AT+USB
  if (Ptr){
    R.USB_On = 1;
    sprintf(R.TmpStr, "+USB\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
  }
  
  
  //----------------------------------------------------------------------------
  Ptr=strstr(R.U1.Buf, "AT+LSB\r");                                             // AT+LSB
  if (Ptr){
    R.USB_On = 0;
    sprintf(R.TmpStr, "+LSB\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
  }
  


  //----------------------------------------------------------------------------
  Ptr=strstr(R.U1.Buf, "AT+EQSETH=");                                           // AT+EQSETH=   (частота Гц, полоса Гц, уровень дБв)
  if (Ptr){
    Ptr +=strlen("AT+EQSETH=");
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; } 
    while (*Ptr != ','){                                                        // Freq
        if (ii>6) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[5]=0;
    }
    Ptr++;                                                                      // Перескакиваем запятую
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
    Ptr++;                                                                      // Перескакиваем запятую
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
    Ptr++;                                                                      // Перескакиваем запятую
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32>=-100) && (Tmp32<=100)){
      R.EqRxHigh.Gain_dBv = Tmp32;    
    }

    allpass_convert_parameters(&R.EqRxHigh);                                    // Преобразовать сохренные параметры в коэффициенты фильтра
    
    sprintf(R.TmpStr, "+EQSETH\r\n Freq %d Hz, BW %d Hz, Gain %d dBv\r\n", R.EqRxHigh.Freq_Hz, R.EqRxHigh.BW_Hz, R.EqRxHigh.Gain_dBv);
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
  }
  

  
  //----------------------------------------------------------------------------
  Ptr=strstr(R.U1.Buf, "AT+PULS=");                                             // AT+PULS=
  if (Ptr){
    Ptr +=strlen("AT+PULS=");   
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; }
    while (*Ptr != '\r'){                                                       // 
        if (ii>5) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[4]=0;
    }
    Ptr++;                                                                      // Перескакиваем запятую
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32>=0) && (Tmp32<=1)){
      R.Tone_Puls = Tmp32;   
    }
    (R.Tone_Puls == 1) ? (sprintf(R.TmpStr, "TONE PULSE ENABLE\r\n")) : (sprintf(R.TmpStr, "TONE PULSE DISABLE\r\n"));
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
    Ptr++;                                                                      // Перескакиваем запятую
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32>=0) && (Tmp32<=1)){
      R.EqRx_En = Tmp32;      
    }
    
    (R.EqRx_En == 1) ? (sprintf(R.TmpStr, "RX EQUALISER ENABLE\r\n")) : (sprintf(R.TmpStr, "RX EQUALISER DISABLE\r\n"));
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
  }
  
  
  
  
  //----------------------------------------------------------------------------
  Ptr=strstr(R.U1.Buf, "AT+TXEQU=");                                            // AT+TXEQU=
  if (Ptr){
    Ptr +=strlen("AT+TXEQU=");   
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; }
    while (*Ptr != '\r'){                                                       // Enable / Disable
        if (ii>5) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[4]=0;
    }
    Ptr++;                                                                      // Перескакиваем запятую
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32>=0) && (Tmp32<=1)){
      R.EqTx_En = Tmp32;      
    }
    
    (R.EqTx_En == 1) ? (sprintf(R.TmpStr, "TX EQUALISER ENABLE\r\n")) : (sprintf(R.TmpStr, "TX EQUALISER DISABLE\r\n"));
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
  }


  
  
//  //----------------------------------------------------------------------------
//  Ptr=strstr(R.U1.Buf, "AT+RXAGC=");                                            // AT+RXAGC=
//  if (Ptr){
//    Ptr +=strlen("AT+RXAGC=");   
//    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; }
//    while (*Ptr != '\r'){                                                       // HPR
//        if (ii>5) { break; }
//        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
//        Tmp_buf[4]=0;
//    }
//    Ptr++;                                                                      // Перескакиваем запятую
//    Tmp32 = atoi(Tmp_buf);
//    if ((Tmp32>=0) && (Tmp32<=1)){
//      R.AGC.Enable = Tmp32;      
//    }
//    
//    (R.AGC.Enable == 1) ? (sprintf(R.TmpStr, "RX AGC ENABLE\r\n")) : (sprintf(R.TmpStr, "RX AGC DISABLE\r\n"));
//    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
//    
//  }
  
  
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
    Ptr++;                                                                      // Перескакиваем запятую
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32>=0) && (Tmp32<=1)){
      R.TXAGC.Enable = Tmp32;   
      CODEC_Init();
    }
    (R.TXAGC.Enable == 1) ? (sprintf(R.TmpStr, "TX AGC ENABLE\r\n")) : (sprintf(R.TmpStr, "TX AGC DISABLE\r\n"));
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
  }
  
  
  //----------------------------------------------------------------------------
  Ptr=strstr(R.U1.Buf, "AT+TXDRC=");                                            // AT+TXDRC=
  if (Ptr){
    Ptr +=strlen("AT+TXDRC=");   
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; }
    while (*Ptr != '\r'){                                                       // HPR
        if (ii>5) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[4]=0;
    }
    Ptr++;                                                                      // Перескакиваем запятую
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32>=0) && (Tmp32<=1)){
      R.TXDRC.Enable = Tmp32;   
      CODEC_Init();
    }
    (R.TXDRC.Enable == 1) ? (sprintf(R.TmpStr, "TX DRC ENABLE\r\n")) : (sprintf(R.TmpStr, "TX DRC DISABLE\r\n"));
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
    Ptr++;                                                                      // Перескакиваем запятую
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32>=0) && (Tmp32<=1)){
      R.RXAGC.Enable = Tmp32;   
      CODEC_Init();
    }
    (R.RXAGC.Enable == 1) ? (sprintf(R.TmpStr, "RX AGC ENABLE\r\n")) : (sprintf(R.TmpStr, "RX AGC DISABLE\r\n"));
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
  }
  
  
  //----------------------------------------------------------------------------
  Ptr=strstr(R.U1.Buf, "AT+RXSQL=");                                            // AT+RXSQL=
  if (Ptr){
    Ptr +=strlen("AT+RXSQL=");   
    ii=0; for (i=0;i<12;i++) { Tmp_buf[i]=0; }
    while (*Ptr != '\r'){                                                       // 
        if (ii>5) { break; }
        Tmp_buf[ii]=*Ptr;   Ptr++;   ii++;
        Tmp_buf[4]=0;
    }
    Ptr++;                                                                      // Перескакиваем запятую
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32>=0) && (Tmp32<=1)){
      R.RXSQL.Enable = Tmp32;   
    }
    (R.RXSQL.Enable == 1) ? (sprintf(R.TmpStr, "RX SQUELCH ENABLE\r\n")) : (sprintf(R.TmpStr, "RX SQUELCH DISABLE\r\n"));
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
    Ptr++;                                                                      // Перескакиваем запятую
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
    Ptr++;                                                                      // Перескакиваем запятую
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
    Ptr++;                                                                      // Перескакиваем запятую
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
    Ptr++;                                                                      // Перескакиваем запятую
    Tmp32 = atoi(Tmp_buf);
    if ((Tmp32>=-6) && (Tmp32<=29)){
      R.Gain_HPL = Tmp32;      
    }
                                                       
    set_TX_Gain_codec();                                                        // Загрузка в кодек поготовленных значений
    
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
    Ptr++;                                                                      // Перескакиваем запятую
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
    
    HAL_RTC_SetDate(&hrtc, &(R.GDate), RTC_FORMAT_BIN);                         // Записать дату/время
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
    Ptr++;                                                                      // Перескакиваем запятую
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
    
    HAL_RTC_SetDate(&hrtc, &(R.GDate), RTC_FORMAT_BIN);                         // Записать дату/время
    HAL_RTC_SetTime(&hrtc, &(R.GTime), RTC_FORMAT_BIN);
  }
  
  
  
  
  
  
  
  //----------------------------------------------------------------------------
  
  save_settings();                                                              // Сохранение настроек в батарейный домен

  
EXIT:
  for (ii=0;ii<255;ii++) R.U1.Buf[ii]=0;                                        // Зачитска буфера перед выходом
  
}




//******************************************************************************
// Отправить в СОМ порт показания S-метра
//******************************************************************************
void S_meter_Send (void){
  sprintf(R.TmpStr, "+SMETER =%d,\r\n G=%d, S_uV=%d uV\r\n", R.S_meter, R.RX_gain, R.S_meter_mkv);
  //CDC_Transmit_FS(Str, strlen(R.TmpStr));
  HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
}   



//******************************************************************************
// Отправить в СОМ порт статус
//******************************************************************************
void Status_Send (void){
  
                                  //HAL_UART_Transmit(&huart1, "OK\r\n", 4, 500);      return;
    

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
//    (R.AGC.Enable == 1) ? (sprintf(R.TmpStr, "RX AGC ENABLE\r\n")) : (sprintf(R.TmpStr, "RX AGC DISABLE\r\n"));
//    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    (R.TXAGC.Enable == 1) ? (sprintf(R.TmpStr, "TX AGC ENABLE\r\n")) : (sprintf(R.TmpStr, "TX AGC DISABLE\r\n"));
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
    // TXDRC status
    (R.TXDRC.Enable == 1) ? (sprintf(R.TmpStr, "TX DRC ENABLE\r\n")) : (sprintf(R.TmpStr, "TX DRC DISABLE\r\n"));
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
    // Tone Pulse Enable
    (R.Tone_Puls == 1) ? (sprintf(R.TmpStr, "TONE PULSED\r\n")) : (sprintf(R.TmpStr, "TONE CONTINUOUS\r\n"));
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
    
    // USB - LSB
    (R.USB_On == 1) ? (sprintf(R.TmpStr, "USB\r\n")) : (sprintf(R.TmpStr, "LSB\r\n"));
    HAL_UART_Transmit(&huart1, (uint8_t*)R.TmpStr, strlen(R.TmpStr), 500);
    
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
//   Сохраняем в бэкап домене текущие настройки (20 int32 значений)
//******************************************************************************
/* Требуется сохранять:
- Частота приемника 16 бит
- Частота передатчика 16 бит
- Усиления в тракте приемника 8+8+8+8 бит
- Усиление в тракте передатчика 8+8+8+8 бит
- Мощность передатчика 8 бит
- Параметры трех полос эквалайзера частота полоса усиление 
- Полоса приемника на прием 4 бит

Битовые:
- Включение/выключение АРУ

*/
void save_settings (void) {
uint32_t ToSave;

  ToSave  = R.DDS_RX.Freq;                                                      // Частота 
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
  ToSave  = 0;
  //ToSave |= (R.AGC.Enable       << 0);
  ToSave |= (R.EqRx_En          << 1);
  ToSave |= (R.EqTx_En          << 2);
  ToSave |= (R.FlashLogEn       << 3);
  ToSave |= (R.TXAGC.Enable     << 4);
  ToSave |= (R.RXSQL.Enable     << 5);
  ToSave |= (R.RXAGC.Enable     << 6);
  ToSave |= (R.TXDRC.Enable     << 7);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR19, ToSave);
  
  ToSave  = 0;
  ToSave |= (R.Tone_Puls        << 0);
  ToSave |= (R.USB_On           << 1);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR18, ToSave);
}


void load_default_settings (void) {                                             // Загрузить стартовые настройки
    RC_start_Init();
    save_settings();
    restore_settings();
}
                                                
//******************************************************************************
//   Восстановить настройки из бэкап домена
//******************************************************************************
void restore_settings (void){
uint32_t In;  
   In = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0);                                 // Частота
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
  //R.AGC.Enable          = ((In & (1 << 0)) != 0) ? (1) : (0);
  R.EqRx_En             = ((In & (1 << 1)) != 0) ? (1) : (0);
  R.EqTx_En             = ((In & (1 << 2)) != 0) ? (1) : (0);
  R.FlashLogEn          = ((In & (1 << 3)) != 0) ? (1) : (0);   
  R.TXAGC.Enable        = ((In & (1 << 4)) != 0) ? (1) : (0);
  R.RXSQL.Enable        = ((In & (1 << 5)) != 0) ? (1) : (0);
  R.RXAGC.Enable        = ((In & (1 << 6)) != 0) ? (1) : (0);
  R.TXDRC.Enable        = ((In & (1 << 7)) != 0) ? (1) : (0);
  
  
  In = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR18);
  R.Tone_Puls          = ((In & (1 << 0)) != 0) ? (1) : (0);
  R.USB_On             = ((In & (1 << 1)) != 0) ? (1) : (0);

}


//******************************************************************************
// ENF OF FILE
//******************************************************************************
