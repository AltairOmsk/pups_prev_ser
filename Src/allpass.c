/*
 * File: allpass.c
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

#include "allpass.h"

/* Model step function */
float allpass(ALLPASS_t *rtM, float input_sample)
{
float Out;
  
  rtM->Tmp4 = rtM->Del2 + rtM->Tmp1;
  rtM->Tmp5 = input_sample - rtM->Tmp4;
  Out  = (input_sample + rtM->Tmp4 + (rtM->Tmp5 * rtM->gain)) * 0.5;
  
  rtM->Tmp1 = (input_sample - rtM->Del2) * rtM->k2;
  rtM->Tmp2 = rtM->Tmp1 + input_sample;
  rtM->Tmp3 = (rtM->Tmp2 - rtM->Del1) * rtM->k1;

  rtM->Del1 = rtM->Tmp3 + rtM->Tmp2;
  rtM->Del2 = rtM->Del1 + rtM->Tmp3;
  
  
  return Out;
  
  
  
//    float rtY_Out;
//    float rtb_Sum10;
//    float rtb_Product4;
//
//    /* Product: '<Root>/Product3' incorporates:
//     *  Delay: '<Root>/Delay3'
//     *  Inport: '<Root>/Ik1'
//     *  Inport: '<Root>/In'
//     *  Sum: '<Root>/Sum11'
//     */
//    rtb_Sum10 = (input_sample-rtM->delay3_state)*rtM->k1;
//
//    /* Sum: '<Root>/Sum13' incorporates:
//     *  Delay: '<Root>/Delay3'
//     */
//    rtb_Product4 = rtb_Sum10+rtM->delay3_state;
//
//    /* Outport: '<Root>/Out' incorporates:
//     *  Gain: '<Root>/Gain1'
//     *  Inport: '<Root>/Gain'
//     *  Inport: '<Root>/In'
//     *  Product: '<Root>/Product5'
//     *  Sum: '<Root>/Sum14'
//     *  Sum: '<Root>/Sum15'
//     */
//    rtY_Out = ((input_sample-rtb_Product4)*rtM->gain+(input_sample+rtb_Product4))*
//            0.5;
//
//    /* Sum: '<Root>/Sum10' incorporates:
//     *  Inport: '<Root>/In'
//     */
//    rtb_Sum10 += input_sample;
//
//    /* Product: '<Root>/Product4' incorporates:
//     *  Delay: '<Root>/Delay2'
//     *  Inport: '<Root>/k2'
//     *  Sum: '<Root>/Sum9'
//     */
//    rtb_Product4 = (rtb_Sum10-rtM->delay2_state)*rtM->k2;
//
//    
//
//    /* Update for Delay: '<Root>/Delay2' incorporates:
//     *  Sum: '<Root>/Sum8'
//     */
//    rtM->delay2_state = rtb_Sum10+rtb_Product4;
//    
//    /* Update for Delay: '<Root>/Delay3' incorporates:
//     *  Delay: '<Root>/Delay2'
//     *  Sum: '<Root>/Sum12'
//     */
//    rtM->delay3_state = rtb_Product4+rtM->delay2_state;
//
//    return rtY_Out;
}



void allpass_convert_parameters(ALLPASS_t *rtM){

   // k1 = -cos(2 * PI * f) // ƒл€ 2 к√ц при Fs 8к должно быть =1, дл€ 1 к - 0,7
   // k2 = 1-sin(B) / cos (B)
   // gain = 10^(dBv/20)
  
    rtM->Del1 = 0;
    rtM->Del2 = 0;
    rtM->Tmp1 = 0;
    rtM->Tmp2 = 0;
    rtM->Tmp3 = 0;
    rtM->Tmp4 = 0;
    rtM->Tmp5 = 0;
  
    
    float k1 = -cos(PI * (  1/((SAMPLE_RATE/1)/(float)rtM->Freq_Hz)   ));
    rtM->k1 = k1;
    
    float B = PI * (1/((SAMPLE_RATE/1)/(float)rtM->BW_Hz));
    float k2 = (1 - sin(B)) / cos(B);
    rtM->k2 = k2;
    
    float gain = pow(10, (float)rtM->Gain_dBv/20);  
    rtM->gain = gain;
    
    
}


void allpass_set_parameters(ALLPASS_t *rtM, float k1, float k2, float gain)
{
    rtM->k1 = k1;
    rtM->k2 = k2;
    rtM->gain = gain;
}


