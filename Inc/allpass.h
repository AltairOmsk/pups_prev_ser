/*
 * File: allpass.h
 *
 * Code generated for Simulink model 'allpass'.
 *
 * Model version                  : 1.2
 * Simulink Coder version         : 8.12 (R2017a) 16-Feb-2017
 * C/C++ source code generated on : Mon Oct 29 11:04:36 2018
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM 7
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef _ALLPASS_H_
#define _ALLPASS_H_

#include "stm32f4xx_hal.h"
#include "math.h"


#define SAMPLE_RATE             8000            // Hz
#define PI                      3.1415926535



/* Real-time Model Data Structure */
typedef struct {
  
    uint16_t Freq_Hz;
    uint16_t BW_Hz;
    int16_t  Gain_dBv;
    
    float Del1;
    float Del2;
    float  Tmp1;
    float  Tmp2;
    float  Tmp3;
    float  Tmp4;
    float  Tmp5;
  
    float delay3_state;                /* '<Root>/Delay3' */
    float delay2_state;                /* '<Root>/Delay2' */
    float k1;
    float k2;
    float gain;
}
ALLPASS_t;

/* Model entry point functions */
void allpass_convert_parameters(ALLPASS_t *rtM);                                // Из частоты полосы и усиления получить коэффициенты фильтра
void allpass_set_parameters(ALLPASS_t *rtM, float k1, float k2, float gain);
float allpass(ALLPASS_t *rtM, float input_sample);

#endif                                 /* RTW_HEADER_allpass_h_ */

