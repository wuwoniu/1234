
/**
  ******************************************************************************
  * @file    mc_parameters.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides definitions of HW parameters specific to the 
  *          configuration of the subsystem.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "parameters_conversion.h"

#include "r3_f4xx_pwm_curr_fdbk.h"
 
 
 
 
 

/**
  * @brief  Current sensor parameters Motor 2 - three shunt
  */
const R3_F4_Params_t R3_F4_ParamsM2 =
{

/* Dual MC parameters --------------------------------------------------------*/
  .Tw               = MAX_TWAIT2,                   
  .bFreqRatio       = FREQ_RATIO,                   
  .bIsHigherFreqTim = FREQ_RELATION2,               

  /* Current reading A/D Conversions initialization -----------------------------*/
  .bIaChannel       = MC_ADC_CHANNEL_3,
  .bIbChannel       = MC_ADC_CHANNEL_4,
  .bIcChannel       = MC_ADC_CHANNEL_5,
                                            
/* PWM generation parameters --------------------------------------------------*/
  .TIMx               = PWM_TIM8,
  .hDeadTime          = DEAD_TIME_COUNTS2,
  .bRepetitionCounter = REP_COUNTER2,
  .hTafter            = TW_AFTER2,
  .hTbefore           = TW_BEFORE2,
  
/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING2,
   
/* PWM Driving signals initialization ----------------------------------------*/
  .EmergencyStop = (FunctionalState) ENABLE,
};
   

/**
  * @brief  Current sensor parameters Motor 1 - three shunt
  */
const R3_F4_Params_t R3_F4_ParamsM1 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .Tw                       =	MAX_TWAIT,
  .bFreqRatio               =	FREQ_RATIO,          
  .bIsHigherFreqTim         =	FREQ_RELATION,       
                                                     
/* Current reading A/D Conversions initialization ----------------------------*/
  .bIaChannel              =	MC_ADC_CHANNEL_10,
  .bIbChannel              =	MC_ADC_CHANNEL_11,
  .bIcChannel              =	MC_ADC_CHANNEL_12,

/* PWM generation parameters --------------------------------------------------*/
  .TIMx                       =	PWM_TIM1,
  .bRepetitionCounter         =	REP_COUNTER,        
  .hTafter                    =	TW_AFTER,           
  .hTbefore                   =	TW_BEFORE,          
                                                    
/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs             =	(LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING, 
 

/* PWM Driving signals initialization ----------------------------------------*/
  .EmergencyStop                =	(FunctionalState) ENABLE,
};
   

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
