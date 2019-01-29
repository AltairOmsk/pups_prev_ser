#ifndef _SQUELCH_H // ��������� ��������� ��������� ����� ������
#define _SQUELCH_H
//******************************************************************************
// ������ include: ����� ������������ ������������ ����� ������������ �������
//******************************************************************************
#include <math.h>
#include "stm32f4xx_hal.h"
//******************************************************************************
// ������ ����������� ��������
//******************************************************************************
//#define MY_CONST1 1
//#define MY_CONST2 2
//#define ...
//******************************************************************************
// ������ ����������� �����
//******************************************************************************
typedef struct
{
  uint8_t Enable;                       // ����. 0 - ��������� 1 - ��������
  float   Out;
  float   Tmp_f;
  
  float   Mu;
  float   Leakage;
  
  float   In_buf[40];                   // ������� ����� � ����� ��������
  //float   Del_buf[40];                  // ����� ��������
  float   Tmp_buf[32];                  // ��������� ����� ��� �������� �� 1 ����
  float   Coeff[32];                    // ������������ ������� �������
  
  
} SQUELCH_t;

//******************************************************************************
// ������ ����������� ���������� ����������
//******************************************************************************
//extern char GlobalVar1;
//extern char GlobalVar2;
//extern ...
//******************************************************************************
// ������ ���������� ���������� �������
//******************************************************************************
float squelch           (float In, SQUELCH_t *S);                               // LMS
float squelch_N         (float In, SQUELCH_t *S);                               // NLMS
void  squelch_init      (SQUELCH_t *S);

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