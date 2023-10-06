
/**
  ******************************************************************************
  * @file    mc_config.c 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Motor Control Subsystem components configuration and handler structures.
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
#include "main.h"
#include "mc_type.h"
#include "parameters_conversion.h"
#include "mc_parameters.h"
#include "mc_config.h"

#define OFFCALIBRWAIT_MS     0
#define OFFCALIBRWAIT_MS2    0     
#include "pqd_motor_power_measurement.h"

PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM1 =
{
  .wConvFact = PQD_CONVERSION_FACTOR
};
PQD_MotorPowMeas_Handle_t *pPQD_MotorPowMeasM1 = &PQD_MotorPowMeasM1; 

PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM2=
{
  .wConvFact = PQD_CONVERSION_FACTOR2
};
PQD_MotorPowMeas_Handle_t *pPQD_MotorPowMeasM2 = &PQD_MotorPowMeasM2; 
  
/**
  * @brief  PI / PID Speed loop parameters Motor 1
  */
PID_Handle_t PIDSpeedHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_SPEED_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_SPEED_KI_DEFAULT, 
  .wUpperIntegralLimit = (int32_t)IQMAX * (int32_t)SP_KIDIV,
  .wLowerIntegralLimit = -(int32_t)IQMAX * (int32_t)SP_KIDIV,
  .hUpperOutputLimit       = (int16_t)IQMAX, 
  .hLowerOutputLimit       = -(int16_t)IQMAX,
  .hKpDivisor          = (uint16_t)SP_KPDIV,
  .hKiDivisor          = (uint16_t)SP_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)SP_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)SP_KIDIV_LOG,
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  PI / PID Iq loop parameters Motor 1
  */
PID_Handle_t PIDIqHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_TORQUE_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_TORQUE_KI_DEFAULT,
  .wUpperIntegralLimit = (int32_t)INT16_MAX * TF_KIDIV,
  .wLowerIntegralLimit = (int32_t)-INT16_MAX * TF_KIDIV,   
  .hUpperOutputLimit       = INT16_MAX,     
  .hLowerOutputLimit       = -INT16_MAX,           
  .hKpDivisor          = (uint16_t)TF_KPDIV,       
  .hKiDivisor          = (uint16_t)TF_KIDIV,       
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG,       
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG,        
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  PI / PID Id loop parameters Motor 1
  */
PID_Handle_t PIDIdHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_FLUX_KP_DEFAULT, 
  .hDefKiGain          = (int16_t)PID_FLUX_KI_DEFAULT, 
  .wUpperIntegralLimit = (int32_t)INT16_MAX * TF_KIDIV, 
  .wLowerIntegralLimit = (int32_t)-INT16_MAX * TF_KIDIV,
  .hUpperOutputLimit       = INT16_MAX,                 
  .hLowerOutputLimit       = -INT16_MAX,                
  .hKpDivisor          = (uint16_t)TF_KPDIV,          
  .hKiDivisor          = (uint16_t)TF_KIDIV,          
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG,       
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG,       
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  FluxWeakeningCtrl component parameters Motor 1
  */
FW_Handle_t FW_M1 =
{
  .hMaxModule             = MAX_MODULE, 
  .hDefaultFW_V_Ref       = (int16_t)FW_VOLTAGE_REF,  
  .hDemagCurrent          = ID_DEMAG, 
  .wNominalSqCurr         = ((int32_t)NOMINAL_CURRENT*(int32_t)NOMINAL_CURRENT),
  .hVqdLowPassFilterBW    = M1_VQD_SW_FILTER_BW_FACTOR,   
  .hVqdLowPassFilterBWLOG = M1_VQD_SW_FILTER_BW_FACTOR_LOG  
};

/**
  * @brief  PI Flux Weakening control parameters Motor 1
  */
PID_Handle_t PIDFluxWeakeningHandle_M1 =
{
  .hDefKpGain          = (int16_t)FW_KP_GAIN,          
  .hDefKiGain          = (int16_t)FW_KI_GAIN,           
  .wUpperIntegralLimit = 0,          
  .wLowerIntegralLimit = (int32_t)(-NOMINAL_CURRENT) * (int32_t)FW_KIDIV, 
  .hUpperOutputLimit       = 0,                          
  .hLowerOutputLimit       = -INT16_MAX,                     
  .hKpDivisor          = (uint16_t)FW_KPDIV,              
  .hKiDivisor          = (uint16_t)FW_KIDIV,   
  .hKpDivisorPOW2      = (uint16_t)FW_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)FW_KIDIV_LOG, 
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  FluxWeakeningCtrl component parameters Motor 2
  */
FW_Handle_t FW_M2 =
{
  .hMaxModule             = MAX_MODULE2,                
  .hDefaultFW_V_Ref       = (int16_t)FW_VOLTAGE_REF2,    
  .hDemagCurrent          = ID_DEMAG2,                   
  .wNominalSqCurr         = ((int32_t)NOMINAL_CURRENT2*(int32_t)NOMINAL_CURRENT2),
  .hVqdLowPassFilterBW    = M2_VQD_SW_FILTER_BW_FACTOR,    
  .hVqdLowPassFilterBWLOG = M2_VQD_SW_FILTER_BW_FACTOR_LOG 
};

/**
  * @brief  PI Flux Weakening control parameters Motor 2
  */
PID_Handle_t PIDFluxWeakeningHandle_M2 =
{
  .hDefKpGain          = (int16_t)FW_KP_GAIN2,           
  .hDefKiGain          = (int16_t)FW_KI_GAIN2,           
  .wUpperIntegralLimit = 0,                             
  .wLowerIntegralLimit = (int32_t)(-NOMINAL_CURRENT2) * (int32_t)FW_KIDIV2,
  .hUpperOutputLimit       = 0,                 
  .hLowerOutputLimit       = -INT16_MAX,           
  .hKpDivisor          = (uint16_t)FW_KPDIV2,             
  .hKiDivisor          = (uint16_t)FW_KIDIV2,        
  .hKpDivisorPOW2      = (uint16_t)FW_KPDIV_LOG2,         
  .hKiDivisorPOW2      = (uint16_t)FW_KIDIV_LOG2,         
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  SpeednTorque Controller parameters Motor 1
  */
SpeednTorqCtrl_Handle_t SpeednTorqCtrlM1 =
{
  .STCFrequencyHz =           		MEDIUM_FREQUENCY_TASK_RATE, 	 
  .MaxAppPositiveMecSpeed01Hz =	(uint16_t)(MAX_APPLICATION_SPEED/6), 
  .MinAppPositiveMecSpeed01Hz =	(uint16_t)(MIN_APPLICATION_SPEED/6), 
  .MaxAppNegativeMecSpeed01Hz =	(int16_t)(-MIN_APPLICATION_SPEED/6), 
  .MinAppNegativeMecSpeed01Hz =	(int16_t)(-MAX_APPLICATION_SPEED/6),
  .MaxPositiveTorque =				(int16_t)NOMINAL_CURRENT,		 
  .MinNegativeTorque =				-(int16_t)NOMINAL_CURRENT,       
  .ModeDefault =					DEFAULT_CONTROL_MODE,            
  .MecSpeedRef01HzDefault =		(int16_t)(DEFAULT_TARGET_SPEED_RPM/6),
  .TorqueRefDefault =				(int16_t)DEFAULT_TORQUE_COMPONENT,
  .IdrefDefault =					(int16_t)DEFAULT_FLUX_COMPONENT,                                                                     
};

RevUpCtrl_Handle_t RevUpControlM1 =
{
  .hRUCFrequencyHz         = MEDIUM_FREQUENCY_TASK_RATE,   
  .hStartingMecAngle       = (int16_t)((int32_t)(STARTING_ANGLE_DEG)* 65536/360),
  .bFirstAccelerationStage = FIRST_SLESS_ALGO_PHASE,   
  .hMinStartUpValidSpeed   = OBS_MINIMUM_SPEED, 
  .hMinStartUpFlySpeed     = (int16_t)(OBS_MINIMUM_SPEED/2),  
  .OTFStartupEnabled       = false,  
  .OTFPhaseParams         = {(uint16_t)500,                 
                                         0,                 
                             (int16_t)PHASE5_FINAL_CURRENT,
                             (void*)MC_NULL},
  .ParamsData             = {{(uint16_t)PHASE1_DURATION,(int16_t)(PHASE1_FINAL_SPEED_RPM/6),(int16_t)PHASE1_FINAL_CURRENT,&RevUpControlM1.ParamsData[1]},
                             {(uint16_t)PHASE2_DURATION,(int16_t)(PHASE2_FINAL_SPEED_RPM/6),(int16_t)PHASE2_FINAL_CURRENT,&RevUpControlM1.ParamsData[2]},
                             {(uint16_t)PHASE3_DURATION,(int16_t)(PHASE3_FINAL_SPEED_RPM/6),(int16_t)PHASE3_FINAL_CURRENT,&RevUpControlM1.ParamsData[3]},
                             {(uint16_t)PHASE4_DURATION,(int16_t)(PHASE4_FINAL_SPEED_RPM/6),(int16_t)PHASE4_FINAL_CURRENT,&RevUpControlM1.ParamsData[4]},
                             {(uint16_t)PHASE5_DURATION,(int16_t)(PHASE5_FINAL_SPEED_RPM/6),(int16_t)PHASE5_FINAL_CURRENT,(void*)MC_NULL},
                            },
};

PWMC_R3_F4_Handle_t PWM_Handle_M1=
{
  {
    .pFctGetPhaseCurrents              = &R3F4XX_GetPhaseCurrents,    
    .pFctSwitchOffPwm                  = &R3F4XX_SwitchOffPWM,             
    .pFctSwitchOnPwm                   = &R3F4XX_SwitchOnPWM,              
    .pFctCurrReadingCalib              = &R3F4XX_CurrentReadingCalibration,
    .pFctTurnOnLowSides                = &R3F4XX_TurnOnLowSides,           
    .pFctSetADCSampPointSect1          = &R3F4XX_SetADCSampPointSect1,     
    .pFctSetADCSampPointSect2          = &R3F4XX_SetADCSampPointSect2,     
    .pFctSetADCSampPointSect3          = &R3F4XX_SetADCSampPointSect3,     
    .pFctSetADCSampPointSect4          = &R3F4XX_SetADCSampPointSect4,     
    .pFctSetADCSampPointSect5          = &R3F4XX_SetADCSampPointSect5,     
    .pFctSetADCSampPointSect6          = &R3F4XX_SetADCSampPointSect6,              
    .pFctIsOverCurrentOccurred         = &R3F4XX_IsOverCurrentOccurred,    
    .pFctOCPSetReferenceVoltage        = MC_NULL,
    .pFctRLDetectionModeEnable         = &R3F4XX_RLDetectionModeEnable,    
    .pFctRLDetectionModeDisable        = &R3F4XX_RLDetectionModeDisable,   
    .pFctRLDetectionModeSetDuty        = &R3F4XX_RLDetectionModeSetDuty,   
    .hT_Sqrt3 = (PWM_PERIOD_CYCLES*SQRT3FACTOR)/16384u,   
    .hSector = 0,   
    .hCntPhA = 0,
    .hCntPhB = 0,
    .hCntPhC = 0,
    .SWerror = 0,
    .bTurnOnLowSidesAction = false, 
    .hOffCalibrWaitTimeCounter = 0, 
    .bMotor = 0,     
    .RLDetectionMode = false, 
    .hIa = 0, 
    .hIb = 0, 
    .hIc = 0, 
    .DTTest = 0,    
    .DTCompCnt = 0, 
    .hPWMperiod          = PWM_PERIOD_CYCLES,     
    .hOffCalibrWaitTicks = (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS)/ 1000),            
    .hDTCompCnt          = DTCOMPCNT, 
    .Ton                 = TON,                             
    .Toff                = TOFF                            
  },
  .wPhaseAOffset = 0,   
  .wPhaseBOffset = 0,  
  .wPhaseCOffset = 0,   
  .wADC1Channel = 0,    
  .wADC2Channel = 0,   
  .Half_PWMPeriod = PWM_PERIOD_CYCLES/2u,  
  .bSoFOC = 0,  
  .bIndex = 0,
  .wADCTriggerSet = 0,   
  .wADCTriggerUnSet = 0,
  .pParams_str = &R3_F4_ParamsM1,

};

/**
  * @brief  PI / PID Speed loop parameters Motor 2
  */
PID_Handle_t PIDSpeedHandle_M2 =
{
  .hDefKpGain          = (int16_t)PID_SPEED_KP_DEFAULT2,     
  .hDefKiGain          = (int16_t)PID_SPEED_KI_DEFAULT2,      
  .wUpperIntegralLimit = (int32_t)IQMAX2 * (int32_t)SP_KIDIV2,  
  .wLowerIntegralLimit = -(int32_t)IQMAX2 * (int32_t)SP_KIDIV2, 
  .hUpperOutputLimit       = (int16_t)IQMAX2,                     
  .hLowerOutputLimit       = -(int16_t)IQMAX2,                
  .hKpDivisor          = (uint16_t)SP_KPDIV2,                 
  .hKiDivisor          = (uint16_t)SP_KIDIV2,                 
  .hKpDivisorPOW2      = (uint16_t)SP_KPDIV_LOG2,             
  .hKiDivisorPOW2      = (uint16_t)SP_KIDIV_LOG2,             
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  PI / PID Iq loop parameters Motor 2
  */
PID_Handle_t PIDIqHandle_M2 =
{
  .hDefKpGain          = (int16_t)PID_TORQUE_KP_DEFAULT2,  
  .hDefKiGain          = (int16_t)PID_TORQUE_KI_DEFAULT2,  
  .wUpperIntegralLimit = (int32_t)INT16_MAX * TF_KIDIV2,   
  .wLowerIntegralLimit = (int32_t)-INT16_MAX * TF_KIDIV2,  
  .hUpperOutputLimit       = INT16_MAX,                    
  .hLowerOutputLimit       = -INT16_MAX,                   
  .hKpDivisor          = (uint16_t)TF_KPDIV2,              
  .hKiDivisor          = (uint16_t)TF_KIDIV2,              
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG2,          
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG2,          
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  PI / PID Id loop parameters Motor 2
  */
PID_Handle_t PIDIdHandle_M2 =
{
  .hDefKpGain          = (int16_t)PID_FLUX_KP_DEFAULT2,    
  .hDefKiGain          = (int16_t)PID_FLUX_KI_DEFAULT2,    
  .wUpperIntegralLimit = (int32_t)INT16_MAX * TF_KIDIV2,   
  .wLowerIntegralLimit = (int32_t)-INT16_MAX * TF_KIDIV2,  
  .hUpperOutputLimit       = INT16_MAX,                    
  .hLowerOutputLimit       = -INT16_MAX,                   
  .hKpDivisor          = (uint16_t)TF_KPDIV2,              
  .hKiDivisor          = (uint16_t)TF_KIDIV2,              
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG2,          
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG2,          
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  SpeednTorque Controller parameters Motor 2
  */
SpeednTorqCtrl_Handle_t SpeednTorqCtrlM2 =
{
  .STCFrequencyHz =           		MEDIUM_FREQUENCY_TASK_RATE2, 	         
  .MaxAppPositiveMecSpeed01Hz =	(uint16_t)(MAX_APPLICATION_SPEED2/6),  
  .MinAppPositiveMecSpeed01Hz =	(uint16_t)(MIN_APPLICATION_SPEED2/6),     
  .MaxAppNegativeMecSpeed01Hz =	(int16_t)(-MIN_APPLICATION_SPEED2/6),  
  .MinAppNegativeMecSpeed01Hz =	(int16_t)(-MAX_APPLICATION_SPEED2/6),
  .MaxPositiveTorque =				(int16_t)NOMINAL_CURRENT2,		   
  .MinNegativeTorque =				-(int16_t)NOMINAL_CURRENT2,              
  .ModeDefault =					DEFAULT_CONTROL_MODE2,                  
  .MecSpeedRef01HzDefault =		(int16_t)(DEFAULT_TARGET_SPEED_RPM2/6),   
  .TorqueRefDefault =				(int16_t)DEFAULT_TORQUE_COMPONENT2,      
  .IdrefDefault =					(int16_t)DEFAULT_FLUX_COMPONENT2,     
};

RevUpCtrl_Handle_t RevUpControlM2 =
{
  .hRUCFrequencyHz         = MEDIUM_FREQUENCY_TASK_RATE2,   
  .hStartingMecAngle       = (int16_t)((int32_t)(STARTING_ANGLE_DEG2)* 65536/360), 
  .bFirstAccelerationStage = FIRST_SLESS_ALGO_PHASE2,       
  .hMinStartUpValidSpeed   = OBS_MINIMUM_SPEED2,            
  .hMinStartUpFlySpeed     = (int16_t)((OBS_MINIMUM_SPEED2)/2),   
  .OTFStartupEnabled       = false, 
  .OTFPhaseParams         = {(uint16_t)500,                
                                         0,                
                             (int16_t)PHASE5_FINAL_CURRENT2, 
                             (void*)MC_NULL},
  .ParamsData             = {{(uint16_t)PHASE1_DURATION2,(int16_t)(PHASE1_FINAL_SPEED_RPM2/6),(int16_t)PHASE1_FINAL_CURRENT2,&RevUpControlM2.ParamsData[1]},
                             {(uint16_t)PHASE2_DURATION2,(int16_t)(PHASE2_FINAL_SPEED_RPM2/6),(int16_t)PHASE2_FINAL_CURRENT2,&RevUpControlM2.ParamsData[2]},
                             {(uint16_t)PHASE3_DURATION2,(int16_t)(PHASE3_FINAL_SPEED_RPM2/6),(int16_t)PHASE3_FINAL_CURRENT2,&RevUpControlM2.ParamsData[3]},
                             {(uint16_t)PHASE4_DURATION2,(int16_t)(PHASE4_FINAL_SPEED_RPM2/6),(int16_t)PHASE4_FINAL_CURRENT2,&RevUpControlM2.ParamsData[4]},
                             {(uint16_t)PHASE5_DURATION2,(int16_t)(PHASE5_FINAL_SPEED_RPM2/6),(int16_t)PHASE5_FINAL_CURRENT2,(void*)MC_NULL},
                            },
  
};

PWMC_R3_F4_Handle_t PWM_Handle_M2=
{
  {
    .pFctGetPhaseCurrents              = &R3F4XX_GetPhaseCurrents,    
    .pFctSwitchOffPwm                  = &R3F4XX_SwitchOffPWM,             
    .pFctSwitchOnPwm                   = &R3F4XX_SwitchOnPWM,              
    .pFctCurrReadingCalib              = &R3F4XX_CurrentReadingCalibration,
    .pFctTurnOnLowSides                = &R3F4XX_TurnOnLowSides,         
    .pFctSetADCSampPointSect1          = &R3F4XX_SetADCSampPointSect1,     
    .pFctSetADCSampPointSect2          = &R3F4XX_SetADCSampPointSect2,     
    .pFctSetADCSampPointSect3          = &R3F4XX_SetADCSampPointSect3,     
    .pFctSetADCSampPointSect4          = &R3F4XX_SetADCSampPointSect4,     
    .pFctSetADCSampPointSect5          = &R3F4XX_SetADCSampPointSect5,     
    .pFctSetADCSampPointSect6          = &R3F4XX_SetADCSampPointSect6,          
    .pFctIsOverCurrentOccurred         = &R3F4XX_IsOverCurrentOccurred,    
    .pFctOCPSetReferenceVoltage        = MC_NULL,
    .pFctRLDetectionModeEnable         = &R3F4XX_RLDetectionModeEnable,    
    .pFctRLDetectionModeDisable        = &R3F4XX_RLDetectionModeDisable,   
    .pFctRLDetectionModeSetDuty        = &R3F4XX_RLDetectionModeSetDuty,   
    .hT_Sqrt3 = (PWM_PERIOD_CYCLES2*SQRT3FACTOR)/16384u,  
    .hSector = 0,   
    .hCntPhA = 0,
    .hCntPhB = 0,
    .hCntPhC = 0,
    .SWerror = 0,
    .bTurnOnLowSidesAction = false, 
    .hOffCalibrWaitTimeCounter = 0, 
    .bMotor = M2,     
    .RLDetectionMode = false,
    .hIa = 0, 
    .hIb = 0,
    .hIc = 0, 
    .DTTest = 0,    
    .DTCompCnt = 0, 
  
  
    .hPWMperiod          = PWM_PERIOD_CYCLES2,    
    .hOffCalibrWaitTicks = (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS2)/ 1000),      
    .hDTCompCnt          = DTCOMPCNT2,                       
    .Ton                 = TON2,                              
    .Toff                = TOFF2                             
  },
  .wPhaseAOffset = 0,   
  .wPhaseBOffset = 0,   
  .wPhaseCOffset = 0,   
  .wADC1Channel = 0,    
  .wADC2Channel = 0,    
  .Half_PWMPeriod = PWM_PERIOD_CYCLES2/2u,
  .bSoFOC = 0, 
  .bIndex = 0,
  .wADCTriggerSet = 0,  
  .wADCTriggerUnSet = 0, 
  .pParams_str = &R3_F4_ParamsM2,

};

/**
  * @brief  SpeedNPosition sensor parameters Motor 2 - HALL
  */
HALL_Handle_t HALL_M2 =
{
  ._Super = {
    .bElToMecRatio                     =	POLE_PAIR_NUM2,            
    .hMaxReliableMecSpeed01Hz          =	(uint16_t)(1.15*MAX_APPLICATION_SPEED2/6),
    .hMinReliableMecSpeed01Hz          =	(uint16_t)(MIN_APPLICATION_SPEED2/6),
    .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS2,           
    .hMaxReliableMecAccel01HzP         =	65535,                             
    .hMeasurementFrequency             =	TF_REGULATION_RATE2,                  
  }, 
   .SensorPlacement     = HALL_SENSORS_PLACEMENT2,
  .PhaseShift          = (int16_t)(HALL_PHASE_SHIFT2 * 65536/360),
  .SpeedSamplingFreqHz = MEDIUM_FREQUENCY_TASK_RATE2,
  .SpeedBufferSize     = HALL_AVERAGING_FIFO_DEPTH2,  
 .TIMClockFreq       = HALL_TIM_CLK2,         
 .TIMx                = HALL_TIM4,   

 .H1Port             =  M2_HALL_H1_GPIO_Port, 
 .H1Pin              =  M2_HALL_H1_Pin,       
 .H2Port             =  M2_HALL_H2_GPIO_Port, 
 .H2Pin              =  M2_HALL_H2_Pin,       
 .H3Port             =  M2_HALL_H3_GPIO_Port, 
 .H3Pin              =  M2_HALL_H3_Pin,       													 
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - HALL
  */

HALL_Handle_t HALL_M1 =
{
  ._Super = {
    .bElToMecRatio                     =	POLE_PAIR_NUM,               
    .hMaxReliableMecSpeed01Hz          =	(uint16_t)(1.15*MAX_APPLICATION_SPEED/6),
    .hMinReliableMecSpeed01Hz          =	(uint16_t)(MIN_APPLICATION_SPEED/6),
    .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS,            
    .hMaxReliableMecAccel01HzP         =	65535,                             
    .hMeasurementFrequency             =	TF_REGULATION_RATE,                 
  }, 
  .SensorPlacement     = HALL_SENSORS_PLACEMENT,
  .PhaseShift          = (int16_t)(HALL_PHASE_SHIFT * 65536/360),
  .SpeedSamplingFreqHz = MEDIUM_FREQUENCY_TASK_RATE,
  .SpeedBufferSize     = HALL_AVERAGING_FIFO_DEPTH, 
 .TIMClockFreq       = HALL_TIM_CLK,         
 .TIMx                = HALL_TIM2,        

 .H1Port             =  M1_HALL_H1_GPIO_Port, 
 .H1Pin              =  M1_HALL_H1_Pin,       
 .H2Port             =  M1_HALL_H2_GPIO_Port, 
 .H2Pin              =  M1_HALL_H2_Pin,       
 .H3Port             =  M1_HALL_H3_GPIO_Port, 
 .H3Pin              =  M1_HALL_H3_Pin,       									 
};

/**
  * Virtual temperature sensor parameters Motor 1
  */
NTC_Handle_t TempSensorParamsM1 =
{
  .bSensorType = VIRTUAL_SENSOR,
  .hExpectedTemp_d = 555, 
  .hExpectedTemp_C = M1_VIRTUAL_HEAT_SINK_TEMPERATURE_VALUE,
};

/**
  * Virtual temperature sensor parameters Motor 2
  */
NTC_Handle_t TempSensorParamsM2 =
{
  .bSensorType = VIRTUAL_SENSOR,
  .hExpectedTemp_d = 555,
  .hExpectedTemp_C = M2_VIRTUAL_HEAT_SINK_TEMPERATURE_VALUE,
};

/* Bus voltage sensor value filter buffer */
uint16_t RealBusVoltageSensorFilterBufferM1[M1_VBUS_SW_FILTER_BW_FACTOR];

/**
  * Bus voltage sensor parameters Motor 1
  */
RDivider_Handle_t RealBusVoltageSensorParamsM1 =
{
  ._Super                =
  {
    .SensorType          = REAL_SENSOR,                 
    .ConversionFactor    = (uint16_t)(ADC_REFERENCE_VOLTAGE / BUS_ADC_CONV_RATIO),                                                   
  },
  
  .VbusRegConv =
  {
    .regADC = ADC1,
    .channel = MC_ADC_CHANNEL_13,
    .samplingTime = M1_VBUS_SAMPLING_TIME,   
  },
  .LowPassFilterBW       =  M1_VBUS_SW_FILTER_BW_FACTOR,  
  .OverVoltageThreshold  = OVERVOLTAGE_THRESHOLD_d,   
  .UnderVoltageThreshold =  UNDERVOLTAGE_THRESHOLD_d,  
  .aBuffer = RealBusVoltageSensorFilterBufferM1,
};

/**
  * Virtual bus voltage sensor parameters Motor 2
  */
VirtualBusVoltageSensor_Handle_t VirtualBusVoltageSensorParamsM2 =
{
  ._Super =
  {
    .SensorType       = VIRTUAL_SENSOR,                 
    .ConversionFactor = 500,                            
  },

  .ExpectedVbus_d = 1+(NOMINAL_BUS_VOLTAGE_V2 * 65536) / 500,
};

UI_Handle_t UI_Params =
{
	      .bDriveNum = 0,
};

/** RAMP for Motor1.
  *
  */
RampExtMngr_Handle_t RampExtMngrHFParamsM1 =
{
  .FrequencyHz = TF_REGULATION_RATE 
};

/**
  * @brief  CircleLimitation Component parameters Motor 1 - Base Component
  */
CircleLimitation_Handle_t CircleLimitationM1 =
{
  .MaxModule          = MAX_MODULE,        	
  .Circle_limit_table = MMITABLE,        	
  .Start_index        = START_INDEX, 		
};
/** RAMP for Motor2.
  *
  */
RampExtMngr_Handle_t RampExtMngrHFParamsM2 =
{
  .FrequencyHz = TF_REGULATION_RATE2 
};

/**
  * @brief  CircleLimitation Component parameters Motor 2 - Base Component
  */
CircleLimitation_Handle_t CircleLimitationM2 =
{
  .MaxModule          = MAX_MODULE2,     	
  .Circle_limit_table = MMITABLE2,       	
  .Start_index        = START_INDEX2,    	
};

UFCP_Handle_t pUSART =
{
    ._Super.RxTimeout = 0, 

    .USARTx              = USART3,                
       
};

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/

