//******************************************************************************
// ������ include: ����� ������������ ������������ ���� � ������
//******************************************************************************
#include "squelch.h" // �������� ���� ��������� ��� ������ ������
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
static float   fir32 (float *In, float *Coeff);                                 // FIR filetr 32 tap
static void    shift_buf_32 (float *In);                                        // Shift back buff 32 
static void    shift_buf_40 (float *In);                                        // Shift back buff 40
static void    vector32_mul (float *In, float *Out, float *Const);              // �������� ����������� ������ �� �����
static void    vector32_add (float *In, float *Add);                            // ��������� ����������� � ������� In ������ Add                       

//******************************************************************************
// ������ �������� ������� (������� ����������, ����� ���������)
//******************************************************************************

void  squelch_init      (SQUELCH_t *S){
  S->Mu         = 0.01;
  S->Leakage    = 0.99;
}

float squelch (float In, SQUELCH_t *S){
  
  //if (S->Enable == 0) return In;
  
  *(S->In_buf+39) = In;                                                         // �������� ����� ������
  S->Out = fir32(S->In_buf, S->Coeff);                                          // ��������� �������� ������
  
  S->Tmp_f = (S->Out * (-1)) + *(S->In_buf);                                    // ��������������� �������� ������ ��������� � ����������� �������
  
  S->Tmp_f *= S->Mu;                                                            // �������� Mu � ������� ������
  
  vector32_mul (S->In_buf, S->Tmp_buf, &S->Tmp_f);                              // �������� ������� ������ �� ������ ������        
  
  vector32_add (S->Tmp_buf, S->Coeff);                                          // ���������� ���������� ������������ � ������
  
  vector32_mul (S->Tmp_buf, S->Coeff, &S->Leakage);                             // ��������� ����� ����� �������������
  
  
  //---   ���������� � ������ �����   ------------------------------------------
  shift_buf_40(S->In_buf);                                                      // ����������� ����� � ������ ��� ����� ������
  return S->Out;
}


















//******************************************************************************
// FIR Filter 32 tap
//******************************************************************************
static float fir32 (float *In, float *Coeff){
  float out1 = 0;
  
  out1 += (*In++) * (*Coeff++);
  out1 += (*In++) * (*Coeff++);
  out1 += (*In++) * (*Coeff++);
  out1 += (*In++) * (*Coeff++);
  out1 += (*In++) * (*Coeff++);
  out1 += (*In++) * (*Coeff++);
  out1 += (*In++) * (*Coeff++);
  out1 += (*In++) * (*Coeff++);
  out1 += (*In++) * (*Coeff++);
  out1 += (*In++) * (*Coeff++);
  out1 += (*In++) * (*Coeff++);
  out1 += (*In++) * (*Coeff++);
  out1 += (*In++) * (*Coeff++);
  out1 += (*In++) * (*Coeff++);
  out1 += (*In++) * (*Coeff++);
  out1 += (*In++) * (*Coeff++); // 16
  
  out1 += (*In++) * (*Coeff++);
  out1 += (*In++) * (*Coeff++);
  out1 += (*In++) * (*Coeff++);
  out1 += (*In++) * (*Coeff++);
  out1 += (*In++) * (*Coeff++);
  out1 += (*In++) * (*Coeff++);
  out1 += (*In++) * (*Coeff++);
  out1 += (*In++) * (*Coeff++);
  out1 += (*In++) * (*Coeff++);
  out1 += (*In++) * (*Coeff++);
  out1 += (*In++) * (*Coeff++);
  out1 += (*In++) * (*Coeff++);
  out1 += (*In++) * (*Coeff++);
  out1 += (*In++) * (*Coeff++);
  out1 += (*In++) * (*Coeff++);
  out1 += (*In++) * (*Coeff++); // 32

  return out1; 
}



//******************************************************************************
// ����� ������ �������� 32 �� 1 ������� �����
//******************************************************************************
static void shift_buf_32 (float *In) {
  *(In +	0	) = *(In + 	1	);
  *(In +	1	) = *(In + 	2	);
  *(In +	2	) = *(In + 	3	);
  *(In +	3	) = *(In + 	4	);
  *(In +	4	) = *(In + 	5	);
  *(In +	5	) = *(In + 	6	);
  *(In +	6	) = *(In + 	7	);
  *(In +	7	) = *(In + 	8	);
  *(In +	8	) = *(In + 	9	);
  *(In +	9	) = *(In + 	10	);
  *(In +	10	) = *(In + 	11	);
  *(In +	11	) = *(In + 	12	);
  *(In +	12	) = *(In + 	13	);
  *(In +	13	) = *(In + 	14	);
  *(In +	14	) = *(In + 	15	);
  *(In +	15	) = *(In + 	16	);
  *(In +	16	) = *(In + 	17	);
  *(In +	17	) = *(In + 	18	);
  *(In +	18	) = *(In + 	19	);
  *(In +	19	) = *(In + 	20	);
  *(In +	20	) = *(In + 	21	);
  *(In +	21	) = *(In + 	22	);
  *(In +	22	) = *(In + 	23	);
  *(In +	23	) = *(In + 	24	);
  *(In +	24	) = *(In + 	25	);
  *(In +	25	) = *(In + 	26	);
  *(In +	26	) = *(In + 	27	);
  *(In +	27	) = *(In + 	28	);
  *(In +	28	) = *(In + 	29	);
  *(In +	29	) = *(In + 	30	);
  *(In +	30	) = *(In + 	31	);
  
}



//******************************************************************************
// ����� ������ �������� 40 �� 1 ������� �����
//******************************************************************************
static void shift_buf_40 (float *In) {
  *(In +	0	) = *(In + 	1	);
  *(In +	1	) = *(In + 	2	);
  *(In +	2	) = *(In + 	3	);
  *(In +	3	) = *(In + 	4	);
  *(In +	4	) = *(In + 	5	);
  *(In +	5	) = *(In + 	6	);
  *(In +	6	) = *(In + 	7	);
  *(In +	7	) = *(In + 	8	);
  *(In +	8	) = *(In + 	9	);
  *(In +	9	) = *(In + 	10	);
  *(In +	10	) = *(In + 	11	);
  *(In +	11	) = *(In + 	12	);
  *(In +	12	) = *(In + 	13	);
  *(In +	13	) = *(In + 	14	);
  *(In +	14	) = *(In + 	15	);
  *(In +	15	) = *(In + 	16	);
  *(In +	16	) = *(In + 	17	);
  *(In +	17	) = *(In + 	18	);
  *(In +	18	) = *(In + 	19	);
  *(In +	19	) = *(In + 	20	);
  *(In +	20	) = *(In + 	21	);
  *(In +	21	) = *(In + 	22	);
  *(In +	22	) = *(In + 	23	);
  *(In +	23	) = *(In + 	24	);
  *(In +	24	) = *(In + 	25	);
  *(In +	25	) = *(In + 	26	);
  *(In +	26	) = *(In + 	27	);
  *(In +	27	) = *(In + 	28	);
  *(In +	28	) = *(In + 	29	);
  *(In +	29	) = *(In + 	30	);
  *(In +	30	) = *(In + 	31	);
  *(In +	31	) = *(In + 	32	);
  *(In +	32	) = *(In + 	33	);
  *(In +	33	) = *(In + 	34	);
  *(In +	34	) = *(In + 	35	);
  *(In +	35	) = *(In + 	36	);
  *(In +	36	) = *(In + 	37	);
  *(In +	37	) = *(In + 	38	);
  *(In +	38	) = *(In + 	39	);
  
}





//******************************************************************************
//   �������� ����������� ������ �� �����
//******************************************************************************
static void    vector32_mul (float *In, float *Out, float *Const){
  *(Out +  0) = *(In +  0) * *Const;
  *(Out +  1) = *(In +  1) * *Const;
  *(Out +  2) = *(In +  2) * *Const;
  *(Out +  3) = *(In +  3) * *Const;
  *(Out +  4) = *(In +  4) * *Const;
  *(Out +  5) = *(In +  5) * *Const;
  *(Out +  6) = *(In +  6) * *Const;
  *(Out +  7) = *(In +  7) * *Const;
  *(Out +  8) = *(In +  8) * *Const;
  *(Out +  9) = *(In +  9) * *Const;
  
  *(Out + 10) = *(In + 10) * *Const;
  *(Out + 11) = *(In + 11) * *Const;
  *(Out + 12) = *(In + 12) * *Const;
  *(Out + 13) = *(In + 13) * *Const;
  *(Out + 14) = *(In + 14) * *Const;
  *(Out + 15) = *(In + 15) * *Const;
  *(Out + 16) = *(In + 16) * *Const;
  *(Out + 17) = *(In + 17) * *Const;
  *(Out + 18) = *(In + 18) * *Const;
  *(Out + 19) = *(In + 19) * *Const;
  
  *(Out + 20) = *(In + 20) * *Const;
  *(Out + 21) = *(In + 21) * *Const;
  *(Out + 22) = *(In + 22) * *Const;
  *(Out + 23) = *(In + 23) * *Const;
  *(Out + 24) = *(In + 24) * *Const;
  *(Out + 25) = *(In + 25) * *Const;
  *(Out + 26) = *(In + 26) * *Const;
  *(Out + 27) = *(In + 27) * *Const;
  *(Out + 28) = *(In + 28) * *Const;
  *(Out + 29) = *(In + 29) * *Const;
  
  *(Out + 30) = *(In + 30) * *Const;
  *(Out + 31) = *(In + 31) * *Const;
  
}              




//******************************************************************************
//   ��������� ����������� � ������� In ������ Add 
//******************************************************************************
static void    vector32_add (float *In, float *Add){
  *(In +  0) += *(Add +  0);
  *(In +  1) += *(Add +  1);
  *(In +  2) += *(Add +  2);
  *(In +  3) += *(Add +  3);
  *(In +  4) += *(Add +  4);
  *(In +  5) += *(Add +  5);
  *(In +  6) += *(Add +  6);
  *(In +  7) += *(Add +  7);
  *(In +  8) += *(Add +  8);
  *(In +  9) += *(Add +  9);
  
  *(In + 10) += *(Add + 10);
  *(In + 11) += *(Add + 11);
  *(In + 12) += *(Add + 12);
  *(In + 13) += *(Add + 13);
  *(In + 14) += *(Add + 14);
  *(In + 15) += *(Add + 15);
  *(In + 16) += *(Add + 16);
  *(In + 17) += *(Add + 17);
  *(In + 18) += *(Add + 18);
  *(In + 19) += *(Add + 19);
  
  *(In + 20) += *(Add + 20);
  *(In + 21) += *(Add + 21);
  *(In + 22) += *(Add + 22);
  *(In + 23) += *(Add + 23);
  *(In + 24) += *(Add + 24);
  *(In + 25) += *(Add + 25);
  *(In + 26) += *(Add + 26);
  *(In + 27) += *(Add + 27);
  *(In + 28) += *(Add + 28);
  *(In + 29) += *(Add + 29);
  
  *(In + 30) += *(Add + 30);
  *(In + 31) += *(Add + 31);
}                            




//******************************************************************************
// ENF OF FILE
//******************************************************************************