
/**
  ******************************************************************************
  * @file    parameters_conversion.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file includes the proper Parameter conversion on the base
  *          of stdlib for the first drive
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PARAMETERS_CONVERSION_H
#define __PARAMETERS_CONVERSION_H

#include "pmsm_motor_parameters.h"
#include "parameters_conversion_f4xx.h"
#include "mc_math.h"

#define ADC_REFERENCE_VOLTAGE  3.30

/************************* CONTROL FREQUENCIES & DELAIES **********************/
#define TF_REGULATION_RATE 	(uint16_t) ((uint16_t)(PWM_FREQUENCY)/REGULATION_EXECUTION_RATE)
#define REP_COUNTER 			(uint16_t) ((REGULATION_EXECUTION_RATE *2u)-1u)

#define MEDIUM_FREQUENCY_TASK_RATE	(uint16_t)SPEED_LOOP_FREQUENCY_HZ

#define INRUSH_CURRLIMIT_DELAY_COUNTS  (uint16_t)(INRUSH_CURRLIMIT_DELAY_MS * \
                                  ((uint16_t)SPEED_LOOP_FREQUENCY_HZ)/1000u -1u)

#define SYS_TICK_FREQUENCY          2000
#define UI_TASK_FREQUENCY_HZ        10
#define SERIAL_COM_TIMEOUT_INVERSE  25
#define SERIAL_COM_ATR_TIME_MS 20

#define MF_TASK_OCCURENCE_TICKS  (SYS_TICK_FREQUENCY/SPEED_LOOP_FREQUENCY_HZ)-1u
#define UI_TASK_OCCURENCE_TICKS  (SYS_TICK_FREQUENCY/UI_TASK_FREQUENCY_HZ)-1u
#define SERIALCOM_TIMEOUT_OCCURENCE_TICKS (SYS_TICK_FREQUENCY/SERIAL_COM_TIMEOUT_INVERSE)-1u
#define SERIALCOM_ATR_TIME_TICKS (uint16_t)(((SYS_TICK_FREQUENCY * SERIAL_COM_ATR_TIME_MS) / 1000u) - 1u)

#define TF_REGULATION_RATE2 	(uint16_t) ((uint16_t)(PWM_FREQUENCY2)/REGULATION_EXECUTION_RATE2)
#define REP_COUNTER2 			(uint16_t) ((REGULATION_EXECUTION_RATE2 *2u)-1u)

#define MEDIUM_FREQUENCY_TASK_RATE2	(uint16_t)SPEED_LOOP_FREQUENCY_HZ2

#define INRUSH_CURRLIMIT_DELAY_COUNTS2  (uint16_t)(INRUSH_CURRLIMIT_DELAY_MS2 * \
                                  ((uint16_t)SPEED_LOOP_FREQUENCY_HZ)/1000u -1u)

#define MF_TASK_OCCURENCE_TICKS2  (SYS_TICK_FREQUENCY/MEDIUM_FREQUENCY_TASK_RATE2)-1u

/************************* OBSERVER + PLL PARAMETERS **************************/
#define MAX_BEMF_VOLTAGE  (uint16_t)((MAX_APPLICATION_SPEED * 1.2 *\
                           MOTOR_VOLTAGE_CONSTANT*SQRT_2)/(1000u*SQRT_3))

/*max phase voltage, 0-peak Volts*/
#define MAX_VOLTAGE (int16_t)((ADC_REFERENCE_VOLTAGE/2)/BUS_ADC_CONV_RATIO) 

#define MAX_CURRENT (ADC_REFERENCE_VOLTAGE/(2*RSHUNT*AMPLIFICATION_GAIN))

#define C1 (int32_t)((((int16_t)F1)*RS)/(LS*TF_REGULATION_RATE))
#define C2 (int32_t) GAIN1
#define C3 (int32_t)((((int16_t)F1)*MAX_BEMF_VOLTAGE)/(LS*MAX_CURRENT*TF_REGULATION_RATE))
#define C4 (int32_t) GAIN2
#define C5 (int32_t)((((int16_t)F1)*MAX_VOLTAGE)/(LS*MAX_CURRENT*TF_REGULATION_RATE))

#define PERCENTAGE_FACTOR    (uint16_t)(VARIANCE_THRESHOLD*128u)      
#define OBS_MINIMUM_SPEED        (uint16_t) (OBS_MINIMUM_SPEED_RPM/6u)
#define HFI_MINIMUM_SPEED        (uint16_t) (HFI_MINIMUM_SPEED_RPM/6u)
#define MAX_BEMF_VOLTAGE2  (uint16_t)((MAX_APPLICATION_SPEED2 * 1.2 *\
                           MOTOR_VOLTAGE_CONSTANT2*SQRT_2)/(1000u*SQRT_3))

#define MAX_VOLTAGE2 (int16_t)(500/2) /* Virtual sensor conversion factor */

#define MAX_CURRENT2 (ADC_REFERENCE_VOLTAGE/(2*RSHUNT*AMPLIFICATION_GAIN2))

#define C12 (int32_t)((((int16_t)F12)*RS2)/(LS2*TF_REGULATION_RATE2))
#define C22 (int32_t) GAIN12
#define C32 (int32_t)((((int16_t)F12)*MAX_BEMF_VOLTAGE2)/(LS2*MAX_CURRENT2*TF_REGULATION_RATE2))
#define C42 (int32_t) GAIN22
#define C52 (int32_t)((((int16_t)F12)*MAX_VOLTAGE2)/(LS2*MAX_CURRENT2*TF_REGULATION_RATE2))

#define PERCENTAGE_FACTOR2    (uint16_t)(VARIANCE_THRESHOLD2*128u)      
#define OBS_MINIMUM_SPEED2        (uint16_t) (OBS_MINIMUM_SPEED_RPM2/6u)
#define HFI_MINIMUM_SPEED2        (uint16_t) (HFI_MINIMUM_SPEED_RPM2/6u)

/*********************** OBSERVER + CORDIC PARAMETERS *************************/
#define CORD_C1 (int32_t)((((int16_t)CORD_F1)*RS)/(LS*TF_REGULATION_RATE))
#define CORD_C2 (int32_t) CORD_GAIN1
#define CORD_C3 (int32_t)((((int16_t)CORD_F1)*MAX_BEMF_VOLTAGE)/(LS*MAX_CURRENT\
                                                           *TF_REGULATION_RATE))
#define CORD_C4 (int32_t) CORD_GAIN2
#define CORD_C5 (int32_t)((((int16_t)CORD_F1)*MAX_VOLTAGE)/(LS*MAX_CURRENT*\
                                                          TF_REGULATION_RATE))
#define CORD_PERCENTAGE_FACTOR    (uint16_t)(CORD_VARIANCE_THRESHOLD*128u)      
#define CORD_MINIMUM_SPEED        (uint16_t) (CORD_MINIMUM_SPEED_RPM/6u)
#define CORD_C12 (int32_t)((((int16_t)CORD_F12)*RS2)/(LS2*TF_REGULATION_RATE2))
#define CORD_C22 (int32_t) CORD_GAIN12
#define CORD_C32 (int32_t)((((int16_t)CORD_F12)*MAX_BEMF_VOLTAGE2)/(LS2*MAX_CURRENT2\
                                                           *TF_REGULATION_RATE2))
#define CORD_C42 (int32_t) CORD_GAIN22
#define CORD_C52 (int32_t)((((int16_t)CORD_F12)*MAX_VOLTAGE2)/(LS2*MAX_CURRENT2*\
                                                          TF_REGULATION_RATE2))
#define CORD_PERCENTAGE_FACTOR2    (uint16_t)(CORD_VARIANCE_THRESHOLD2*128u)      
#define CORD_MINIMUM_SPEED2        (uint16_t) (CORD_MINIMUM_SPEED_RPM2/6u)

/**************************   VOLTAGE CONVERSIONS  ****************************/
#define BUS_ADC_CONV_RATIO       VBUS_PARTITIONING_FACTOR

#define OVERVOLTAGE_THRESHOLD_d   (uint16_t)(OV_VOLTAGE_THRESHOLD_V*65535/\
                                  (ADC_REFERENCE_VOLTAGE/VBUS_PARTITIONING_FACTOR))
#define UNDERVOLTAGE_THRESHOLD_d  (uint16_t)((UD_VOLTAGE_THRESHOLD_V*65535)/\
                                  ((uint16_t)(ADC_REFERENCE_VOLTAGE/\
                                                           BUS_ADC_CONV_RATIO)))
#define INT_SUPPLY_VOLTAGE          (uint16_t)(65536/ADC_REFERENCE_VOLTAGE)

#define DELTA_TEMP_THRESHOLD        (OV_TEMPERATURE_THRESHOLD_C- T0_C)
#define DELTA_V_THRESHOLD           (dV_dT * DELTA_TEMP_THRESHOLD)
#define OV_TEMPERATURE_THRESHOLD_d  ((V0_V + DELTA_V_THRESHOLD)*INT_SUPPLY_VOLTAGE)

#define DELTA_TEMP_HYSTERESIS        (OV_TEMPERATURE_HYSTERESIS_C)
#define DELTA_V_HYSTERESIS           (dV_dT * DELTA_TEMP_HYSTERESIS)
#define OV_TEMPERATURE_HYSTERESIS_d  (DELTA_V_HYSTERESIS*INT_SUPPLY_VOLTAGE)
#define BUS_ADC_CONV_RATIO2       VBUS_PARTITIONING_FACTOR2

#define OVERVOLTAGE_THRESHOLD_d2   (uint16_t)(OV_VOLTAGE_THRESHOLD_V2*65535/\
                                  (ADC_REFERENCE_VOLTAGE/VBUS_PARTITIONING_FACTOR2))
#define UNDERVOLTAGE_THRESHOLD_d2  (uint16_t)((UD_VOLTAGE_THRESHOLD_V2*65535)/\
                                  ((uint16_t)(ADC_REFERENCE_VOLTAGE/\
                                                           BUS_ADC_CONV_RATIO2)))
#define INT_SUPPLY_VOLTAGE2          (uint16_t)(65536/ADC_REFERENCE_VOLTAGE)

#define DELTA_TEMP_THRESHOLD2        (OV_TEMPERATURE_THRESHOLD_C2- T0_C2)
#define DELTA_V_THRESHOLD2           (dV_dT2 * DELTA_TEMP_THRESHOLD2)
#define OV_TEMPERATURE_THRESHOLD_d2  ((V0_V2 + DELTA_V_THRESHOLD2)*INT_SUPPLY_VOLTAGE2)

#define DELTA_TEMP_HYSTERESIS2        (OV_TEMPERATURE_HYSTERESIS_C2)
#define DELTA_V_HYSTERESIS2           (dV_dT2 * DELTA_TEMP_HYSTERESIS2)
#define OV_TEMPERATURE_HYSTERESIS_d2  (DELTA_V_HYSTERESIS2*INT_SUPPLY_VOLTAGE2)

/*************** Encoder Alignemnt ************************/
    
/* Encoder alignment */
#define ALIGNMENT_ANGLE_S16      (int16_t)  (ALIGNMENT_ANGLE_DEG*65536u/360u)
#define ALIGNMENT_ANGLE_S162      (int16_t)  (ALIGNMENT_ANGLE_DEG2*65536u/360u)

/*************** Timer for PWM generation & currenst sensing parameters  ******/
#define PWM_PERIOD_CYCLES (uint16_t)(ADV_TIM_CLK_MHz*\
                                      (unsigned long long)1000000u/((uint16_t)(PWM_FREQUENCY)))
#define PWM_PERIOD_CYCLES2 (uint16_t)(ADV_TIM_CLK_MHz*\
                                      (unsigned long long)1000000u/((uint16_t)(PWM_FREQUENCY2)))

#define DEADTIME_NS  SW_DEADTIME_NS
#define DEADTIME_NS2  SW_DEADTIME_NS2

#define DEAD_TIME_ADV_TIM_CLK_MHz (ADV_TIM_CLK_MHz * TIM_CLOCK_DIVIDER)
#define DEAD_TIME_COUNTS_1  (DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/1000uL)

#if (DEAD_TIME_COUNTS_1 <= 255)
#define DEAD_TIME_COUNTS (uint16_t) DEAD_TIME_COUNTS_1
#elif (DEAD_TIME_COUNTS_1 <= 508)
#define DEAD_TIME_COUNTS (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/2) /1000uL) + 128)
#elif (DEAD_TIME_COUNTS_1 <= 1008)
#define DEAD_TIME_COUNTS (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/8) /1000uL) + 320)
#elif (DEAD_TIME_COUNTS_1 <= 2015)
#define DEAD_TIME_COUNTS (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/16) /1000uL) + 384)
#else
#define DEAD_TIME_COUNTS 510
#endif
#define DEAD_TIME_ADV_TIM_CLK_MHz2 (ADV_TIM_CLK_MHz2 * TIM_CLOCK_DIVIDER2)
#define DEAD_TIME_COUNTS2_1  (DEAD_TIME_ADV_TIM_CLK_MHz2 * DEADTIME_NS2/1000uL)

#if (DEAD_TIME_COUNTS2_1 <= 255)
#define DEAD_TIME_COUNTS2 (uint16_t) DEAD_TIME_COUNTS2_1
#elif (DEAD_TIME_COUNTS2_1 <= 508)
#define DEAD_TIME_COUNTS2 (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz2 * DEADTIME_NS2/2) /1000uL) + 128)
#elif (DEAD_TIME_COUNTS2_1 <= 1008)
#define DEAD_TIME_COUNTS2 (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz2 * DEADTIME_NS2/8) /1000uL) + 320)
#elif (DEAD_TIME_COUNTS2_1 <= 2015)
#define DEAD_TIME_COUNTS2 (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz2 * DEADTIME_NS2/16) /1000uL) + 384)
#else
#define DEAD_TIME_COUNTS2 510
#endif

#define DTCOMPCNT (uint16_t)((DEADTIME_NS * ADV_TIM_CLK_MHz) / 2000)
#define TON_NS  500
#define TOFF_NS 500
#define TON  (uint16_t)((TON_NS * ADV_TIM_CLK_MHz)  / 2000)
#define TOFF (uint16_t)((TOFF_NS * ADV_TIM_CLK_MHz) / 2000)
#define DTCOMPCNT2 (uint16_t)((DEADTIME_NS2 * ADV_TIM_CLK_MHz2) / 2000)
#define TON_NS2  500
#define TOFF_NS2 500
#define TON2  (uint16_t)((TON_NS2 * ADV_TIM_CLK_MHz2)  / 2000)
#define TOFF2 (uint16_t)((TOFF_NS2 * ADV_TIM_CLK_MHz2) / 2000)

#define MAX_TNTR_NS TRISE_NS
#define MAX_TNTR_NS2 TRISE_NS2

#define SAMPLING_TIME (uint16_t)(((uint16_t)(SAMPLING_TIME_NS) * ADV_TIM_CLK_MHz)/1000uL) 
#define TRISE (uint16_t)((((uint16_t)(TRISE_NS)) * ADV_TIM_CLK_MHz)/1000uL)
#define TDEAD (uint16_t)((DEADTIME_NS * ADV_TIM_CLK_MHz)/1000uL)

#define TMIN (((uint16_t)(((DEADTIME_NS+((uint16_t)(TRISE_NS))+\
			 ((uint16_t)(SAMPLING_TIME_NS+TRIG_CONV_LATENCY_NS)))*ADV_TIM_CLK_MHz)/1000ul))+1)
#define HTMIN (uint16_t)(TMIN >> 1)
#define CHTMIN (uint16_t)(TMIN/(REGULATION_EXECUTION_RATE*2))
#define TAFTER ((uint16_t)(((DEADTIME_NS+((uint16_t)(TRISE_NS)))\
					   *ADV_TIM_CLK_MHz)/1000ul))

#define TBEFORE (((uint16_t)(((((uint16_t)(SAMPLING_TIME_NS+TRIG_CONV_LATENCY_NS)))\
                                           *ADV_TIM_CLK_MHz)/1000ul))+1)
                                           
#define SAMPLING_TIME2 (uint16_t)(((uint16_t)(SAMPLING_TIME_NS2) * ADV_TIM_CLK_MHz2)/1000uL) 
#define TRISE2 (uint16_t)((((uint16_t)(TRISE_NS2)) * ADV_TIM_CLK_MHz2)/1000uL)
#define TDEAD2 (uint16_t)((DEADTIME_NS2 * ADV_TIM_CLK_MHz2)/1000uL)

#define TMIN2 (((uint16_t)(((DEADTIME_NS2+((uint16_t)(TRISE_NS2))+\
			 ((uint16_t)(SAMPLING_TIME_NS2+TRIG_CONV_LATENCY_NS)))*ADV_TIM_CLK_MHz2)/1000ul))+1)
#define HTMIN2 (uint16_t)(TMIN2 >> 1)
#define CHTMIN2 (uint16_t)(TMIN2/(REGULATION_EXECUTION_RATE2*2))
#define TSAMPLE2 SAMPLING_TIME2
#define TAFTER2 ((uint16_t)(((DEADTIME_NS2+((uint16_t)(TRISE_NS2)))\
												   *ADV_TIM_CLK_MHz2)/1000ul))
#define TBEFORE2 (((uint16_t)(((((uint16_t)(SAMPLING_TIME_NS2+TRIG_CONV_LATENCY_NS)))\
											*ADV_TIM_CLK_MHz2)/1000ul))+1)

#if (TRISE_NS > SAMPLING_TIME_NS)
#define MAX_TRTS (2 * TRISE)
#else
#define MAX_TRTS (2 * SAMPLING_TIME)
#endif

#define TNOISE (uint16_t)((((uint16_t)(TNOISE_NS)) * ADV_TIM_CLK_MHz)/1000uL)

#define MAX_TNTR_NS TRISE_NS
#if (TRISE_NS2 > SAMPLING_TIME_NS2)
#define MAX_TRTS2 (2 * TRISE2)
#else
#define MAX_TRTS2 (2 * SAMPLING_TIME2)
#endif

#define TNOISE2 (uint16_t)((((uint16_t)(TNOISE_NS2)) * ADV_TIM_CLK_MHz2)/1000uL)

#define MAX_TNTR_NS2 TRISE_NS2

#define TW_AFTER ((uint16_t)(((DEADTIME_NS+MAX_TNTR_NS)*ADV_TIM_CLK_MHz)/1000ul))
#define TW_AFTER2 ((uint16_t)(((DEADTIME_NS2+MAX_TNTR_NS2)*ADV_TIM_CLK_MHz2)/1000ul))

/*************** PI divisor  ***************/
#define SP_KPDIV_LOG LOG2(16)
#define SP_KIDIV_LOG LOG2(256)
#define SP_KDDIV_LOG LOG2(16)
#define TF_KPDIV_LOG LOG2(4096)
#define TF_KIDIV_LOG LOG2(16384)
#define TF_KDDIV_LOG LOG2(8192)
#define FW_KPDIV_LOG LOG2(32768)
#define FW_KIDIV_LOG LOG2(32768)
#define PLL_KPDIV     16384
#define PLL_KPDIV_LOG LOG2(PLL_KPDIV)
#define PLL_KIDIV     65535
#define PLL_KIDIV_LOG LOG2(PLL_KIDIV)
#define F1_LOG LOG2(16384)
#define F2_LOG LOG2(16384)
#define STO_FIFO_DEPTH_DPP_LOG LOG2(64)
#define CORD_FIFO_DEPTH_DPP_LOG LOG2(64)
#define HFI_PID_KPDIV_LOG LOG2(16384)
#define HFI_PID_KIDIV_LOG LOG2(32768)
#define SP_KPDIV_LOG2 LOG2(16)
#define SP_KIDIV_LOG2 LOG2(256)
#define SP_KDDIV_LOG2 LOG2(16)
#define TF_KPDIV_LOG2 LOG2(1024)
#define TF_KIDIV_LOG2 LOG2(8192)
#define TF_KDDIV_LOG2 LOG2(8192)
#define FW_KPDIV_LOG2 LOG2(32768)
#define FW_KIDIV_LOG2 LOG2(32768)
#define PLL_KPDIV2     16384
#define PLL_KPDIV_LOG2 LOG2(PLL_KPDIV2)
#define PLL_KIDIV2     65535
#define PLL_KIDIV_LOG2 LOG2(PLL_KIDIV2)
#define F1_LOG2 LOG2(4096)
#define F2_LOG2 LOG2(8192)
#define STO_FIFO_DEPTH_DPP_LOG2 LOG2(64)
#define CORD_FIFO_DEPTH_DPP_LOG2 LOG2(64)
#define HFI_PID_KPDIV_LOG2 LOG2(16384)
#define HFI_PID_KIDIV_LOG2 LOG2(32768)
 
/* USER CODE BEGIN virtual temperature */

#define M1_VIRTUAL_HEAT_SINK_TEMPERATURE_VALUE   25u
#define M1_TEMP_SW_FILTER_BW_FACTOR      250u
#define M1_VQD_SW_FILTER_BW_FACTOR       128u
#define M1_VQD_SW_FILTER_BW_FACTOR_LOG LOG2(M1_VQD_SW_FILTER_BW_FACTOR)

#define M2_VIRTUAL_HEAT_SINK_TEMPERATURE_VALUE   25u
#define M2_TEMP_SW_FILTER_BW_FACTOR     250u
#define M2_VQD_SW_FILTER_BW_FACTOR      128u
#define M2_VQD_SW_FILTER_BW_FACTOR_LOG LOG2(M2_VQD_SW_FILTER_BW_FACTOR)
/* USER CODE END virtual temperature */
 
#define FLUX_WEAKENING
#define FLUX_WEAKENING2
 
#define PQD_CONVERSION_FACTOR (int32_t)(( 1000 * 3 * ADC_REFERENCE_VOLTAGE ) /\
             ( 1.732 * RSHUNT * AMPLIFICATION_GAIN ))
#define PQD_CONVERSION_FACTOR2 (int32_t)(( 1000 * 3 * ADC_REFERENCE_VOLTAGE ) /\
             ( 1.732 * RSHUNT2 * AMPLIFICATION_GAIN2 ))

#define PWM_FREQUENCY_CHECK_RATIO   (PWM_FREQUENCY*10000u/PWM_FREQUENCY2)

#define MAGN_FREQ_RATIO   (1*10000u)

#if (PWM_FREQUENCY_CHECK_RATIO != MAGN_FREQ_RATIO)
#error "The two motor PWM frequencies should be integer multiple"  
#endif

#define USART_IRQHandler        USART3_IRQHandler

/****** Prepares the UI configurations according the MCconfxx settings ********/
#define COM_ENABLE | OPT_COM

#define DAC_ENABLE
#define DAC_OP_ENABLE

/* Motor 1 settings */
#define FW_ENABLE | UI_CFGOPT_FW

#define DIFFTERM_ENABLE
/* Motor 2 settings */
#define FW_ENABLE2 | UI_CFGOPT_FW

#define DIFFTERM_ENABLE2

/* Sensors setting */

#define MAIN_SCFG UI_SCODE_HALL

#define AUX_SCFG 0x0

#define MAIN_SCFG2 UI_SCODE_HALL

#define AUX_SCFG2 0x0

#define PLLTUNING_ENABLE
#define PLLTUNING_ENABLE2

#define UI_CFGOPT_PFC_ENABLE

/******************************************************************************* 
  * UI configurations settings. It can be manually overwritten if special 
  * configuartion is required. 
*******************************************************************************/

/* Specific options of UI */
#define UI_CONFIG_M1 ( UI_CFGOPT_NONE DAC_OP_ENABLE FW_ENABLE DIFFTERM_ENABLE \
  | (MAIN_SCFG << MAIN_SCFG_POS) | (AUX_SCFG << AUX_SCFG_POS) | UI_CFGOPT_SETIDINSPDMODE PLLTUNING_ENABLE UI_CFGOPT_PFC_ENABLE | UI_CFGOPT_PLLTUNING)

/* Specific options of UI, Motor 2 */
#define UI_CONFIG_M2 ( UI_CFGOPT_NONE DAC_OP_ENABLE FW_ENABLE DIFFTERM_ENABLE2 \
  | (MAIN_SCFG2 << MAIN_SCFG_POS) | (AUX_SCFG2 << AUX_SCFG_POS) | UI_CFGOPT_SETIDINSPDMODE PLLTUNING_ENABLE2 )

#define DIN_ACTIVE_LOW Bit_RESET
#define DIN_ACTIVE_HIGH Bit_SET

 /*** Temporary bridge between workbench data model****/
#define PWM_TIM1	TIM1
#define PWM_TIM8  TIM8
#define HALL_TIM2 TIM2
#define HALL_TIM3 TIM3
#define HALL_TIM4 TIM4
#define HALL_TIM5 TIM5

#define ENC_TIM2 TIM2
#define ENC_TIM3 TIM3
#define ENC_TIM4 TIM4
#define ENC_TIM5 TIM5

#define DOUT_ACTIVE_HIGH   DOutputActiveHigh
#define DOUT_ACTIVE_LOW    DOutputActiveLow

/**********  AUXILIARY HALL TIMER MOTOR 1 *************/
#define M1_HALL_TIM_PERIOD 65535
#define M1_HALL_IC_FILTER  15
#define SPD_TIM_M1_IRQHandler TIM2_IRQHandler
/**********  AUXILIARY HALL TIMER MOTOR 2 *************/
#define M2_HALL_TIM_PERIOD 65535
#define M2_HALL_IC_FILTER  15
#define SPD_TIM_M2_IRQHandler TIM4_IRQHandler

#define START_INDEX     57
#define MAX_MODULE      31128   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*95%
#define MMITABLE {\
32613,32310,32016,31872,31589,31314,31046,30784,30529,30404,\
30158,29919,29684,29456,29343,29122,28906,28695,28488,28285,\
28186,27990,27798,27610,27425,27245,27155,26980,26808,26639,\
26473,26392,26230,26072,25917,25764,25614,25540,25394,25250,\
25109,24970,24901,24766,24633,24501,24372,24245,24182,24058,\
23936,23816,23697,23580,23522,23408,23295,23184,23075,23021,\
22913,22808,22703,22600,22499,22449,22349,22251,22154,22059,\
21964\
}

#define START_INDEX2     57
#define MAX_MODULE2      31128   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*95%
#define MMITABLE2 {\
32613,32310,32016,31872,31589,31314,31046,30784,30529,30404,\
30158,29919,29684,29456,29343,29122,28906,28695,28488,28285,\
28186,27990,27798,27610,27425,27245,27155,26980,26808,26639,\
26473,26392,26230,26072,25917,25764,25614,25540,25394,25250,\
25109,24970,24901,24766,24633,24501,24372,24245,24182,24058,\
23936,23816,23697,23580,23522,23408,23295,23184,23075,23021,\
22913,22808,22703,22600,22499,22449,22349,22251,22154,22059,\
21964\
}

#endif /*__PARAMETERS_CONVERSION_H*/

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
