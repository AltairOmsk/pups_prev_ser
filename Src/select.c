//******************************************************************************
// ������ include: ����� ������������ ������������ ���� � ������
//******************************************************************************
#include "select.h" // �������� ���� ��������� ��� ������ ������
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
static DDS16_t  Gen;                                                            // ��������� ���� ��� ��������
static DDS16_t  Gen_RX_M;                                                       // ���������� ���� MARK � SPACE ��� ������
static DDS16_t  Gen_RX_S;
static SQL_RX_t SqR;
//******************************************************************************
// ������ ���������� ��������� �������
//******************************************************************************
//void local_func1 (void);
//void local_func2 (void);
//...
//******************************************************************************
// ������ �������� ������� (������� ����������, ����� ���������)
//******************************************************************************


//******************************************************************************
//   ������������� ������ �� �������� ��� �������� ��������������
//******************************************************************************
/*
���������� � �������� 8.138 ���
���� ��� �������� ����� ������ ������������ - ������ ���������� ������� ������ ��� ���������
���������� �� ������� ������, ������� �������� ����� ������ � ���������
*/
float create_TX_SQL_signal   (uint16_t *Type, uint16_t Scale, float In){
static uint16_t Signal=0;                                                       // ������������ ���������
static uint16_t BitTick=0;                                                      // ������������ ���� � �����
static uint16_t BitCnt=0;                                                       // ����� �������� ������������� ����
  
  if ((*Type) == 0)             return In;                                      // ��������� ���������, ������ ��������� ������� ������

  if ((BitTick == 0) && (BitCnt == 0)) {                                        // ������ ��������� ������ �������
    Signal  = *Type;
    BitTick = 0;
    BitCnt  = 0;
  }
  
  
  if (++BitTick >= SAMPLE_LEN_TICK) {  BitTick = 0;                             // ������� ����� � �������
    if (++BitCnt > SIGNAL_LEN_BIT) goto END_SIGNAL;                             // ��������� ��� � ��������� ������������ �����
  }
  
  

  
  if (Signal & (1<<BitCnt)) {                                                   // ���������� ������� ������������� �������
    Gen.Freq = MARK_TONE;                                                       // � ����������� �� �������� ����
  } else {
    Gen.Freq = SPACE_TONE;
  }
  
  
  

  
  Gen.Fdiskr = SAMPLE_RATE;
  dds16(&Gen);                                                                  // ��������� ���
  return Gen.I * Scale;                                                         // � ������ ��� � ���������������� ����

  
//------------------------------------------------------------------------------
END_SIGNAL:                                                                     // ������ �������, ��������� � �������� ������ ����
  *Type = 0;
  Signal = 0;
  BitTick = 0;
  BitCnt = 0;
  return 0;
}                                





//******************************************************************************
// �������� ����� � ������� ����� ������ ���
//******************************************************************************
/*
�������� ������ ����� �� SIN � COS MARR � SPACE ��, �������� 8 �����
�������� ������ ������ �� ������ ������ - ��� 4 �����
���������� ������ ������ ��� ���� �������, �������� 2 ������� ������
������ ������� ����� ���������� � ��������� ����������� � ����������� ������������������

���� � ����� �� ������ ���� �������� ������������������ - ���������!! �� ������ ����� ������
���� ��� �������� ������ ���������� ������� ������ �� ������

*/
uint8_t  select_RX_channel (float MA_Ch, float GA_Ch){

  Gen_RX_M.Fdiskr = SAMPLE_RATE;
  Gen_RX_S.Fdiskr = SAMPLE_RATE;
  dds16(&Gen_RX_M);                                                             // ���������� ������� ���������� �����������
  dds16(&Gen_RX_S);
  
  SqR.MA_M_I = Gen_RX_M.I * MA_Ch;                                              // ������������ �������� ������� �� �� � ��
  SqR.MA_M_Q = Gen_RX_M.Q * MA_Ch;                                              // �� ������� �������� �������
  SqR.MA_S_I = Gen_RX_M.I * MA_Ch;
  SqR.MA_S_Q = Gen_RX_M.Q * MA_Ch;
  
  SqR.GA_M_I = Gen_RX_M.I * GA_Ch;                                              // ��� �� �������
  SqR.GA_M_Q = Gen_RX_M.Q * GA_Ch;
  SqR.GA_S_I = Gen_RX_M.I * GA_Ch;
  SqR.GA_S_Q = Gen_RX_M.Q * GA_Ch;
  
  SqR.MA_M   = (SqR.MA_M_I * SqR.MA_M_I) + (SqR.MA_M_Q * SqR.MA_M_Q);           // ��������� ������ MARK � SPACE ��� ��������� ������
  SqR.MA_S   = (SqR.MA_S_I * SqR.MA_S_I) + (SqR.MA_S_Q * SqR.MA_S_Q);
  
  SqR.GA_M   = (SqR.GA_M_I * SqR.GA_M_I) + (SqR.GA_M_Q * SqR.GA_M_Q);           // ��������� ������ MARK � SPACE ��� �� ������
  SqR.GA_S   = (SqR.GA_S_I * SqR.GA_S_I) + (SqR.GA_S_Q * SqR.GA_S_Q);
  
  
  
  
  
  return 0;
}   






//******************************************************************************
// ENF OF FILE
//******************************************************************************