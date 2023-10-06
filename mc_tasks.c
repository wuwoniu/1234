
/**
  ******************************************************************************
  * @file    mc_tasks.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implementes tasks definition
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
#include "mc_type.h"
#include "mc_math.h"
#include "mc_config.h"
#include "regular_conversion_manager.h"
#include "mc_interface.h"
#include "mc_tuning.h"
#include "digital_output.h"
#include "state_machine.h"
#include "pwm_common.h"

#include "mc_tasks.h"
#include "parameters_conversion.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private define */
/* Private define ------------------------------------------------------------*/

#define CHARGE_BOOT_CAP_MS  10
#define CHARGE_BOOT_CAP_MS2 10
#define OFFCALIBRWAIT_MS     0
#define OFFCALIBRWAIT_MS2    0
#define STOPPERMANENCY_MS  400
#define STOPPERMANENCY_MS2 400
#define CHARGE_BOOT_CAP_TICKS  (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS)/ 1000)
#define CHARGE_BOOT_CAP_TICKS2 (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS2)/ 1000)
#define OFFCALIBRWAITTICKS     (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS)/ 1000)
#define OFFCALIBRWAITTICKS2    (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS2)/ 1000)
#define STOPPERMANENCY_TICKS   (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS)/ 1000)
#define STOPPERMANENCY_TICKS2  (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS2)/ 1000)

/* Un-Comment this macro define in order to activate the smooth
   braking action on over voltage */
/* #define  MC.SMOOTH_BRAKING_ACTION_ON_OVERVOLTAGE */

/* USER CODE END Private define */
#define VBUS_TEMP_ERR_MASK ~(0 | 0 | MC_OVER_TEMP)
#define VBUS_TEMP_ERR_MASK2 ~(MC_OVER_VOLT | MC_UNDER_VOLT | MC_OVER_TEMP)

/* Private variables----------------------------------------------------------*/
FOCVars_t FOCVars[NBR_OF_MOTORS];
MCI_Handle_t Mci[NBR_OF_MOTORS];
MCI_Handle_t * oMCInterface[NBR_OF_MOTORS];
MCT_Handle_t MCT[NBR_OF_MOTORS];
STM_Handle_t STM[NBR_OF_MOTORS];
SpeednTorqCtrl_Handle_t *pSTC[NBR_OF_MOTORS];
PID_Handle_t *pPIDSpeed[NBR_OF_MOTORS];
PID_Handle_t *pPIDIq[NBR_OF_MOTORS];
PID_Handle_t *pPIDId[NBR_OF_MOTORS];
RDivider_Handle_t *pBusSensorM1;
VirtualBusVoltageSensor_Handle_t *pBusSensorM2;
NTC_Handle_t *pTemperatureSensor[NBR_OF_MOTORS];
PWMC_Handle_t * pwmcHandle[NBR_OF_MOTORS];
DOUT_handle_t *pR_Brake[NBR_OF_MOTORS];
DOUT_handle_t *pOCPDisabling[NBR_OF_MOTORS];
PQD_MotorPowMeas_Handle_t *pMPM[NBR_OF_MOTORS];
CircleLimitation_Handle_t *pCLM[NBR_OF_MOTORS];
FW_Handle_t *pFW[NBR_OF_MOTORS];     /* only if M1 or M2 has FW */
RampExtMngr_Handle_t *pREMNG[NBR_OF_MOTORS];   /*!< Ramp manager used to modify the Iq ref
                                                    during the start-up switch over.*/

static volatile uint16_t hMFTaskCounterM1 = 0;
static volatile uint16_t hBootCapDelayCounterM1 = 0;
static volatile uint16_t hStopPermanencyCounterM1 = 0;
static volatile uint16_t hMFTaskCounterM2 = 0;
static volatile uint16_t hBootCapDelayCounterM2 = 0;
static volatile uint16_t hStopPermanencyCounterM2 = 0;

uint8_t bMCBootCompleted = 0;

/* USER CODE BEGIN Private Variables */

/* USER CODE END Private Variables */

/* Private functions ---------------------------------------------------------*/
static void TSK_MediumFrequencyTaskM1(void);
static void FOC_Clear(uint8_t bMotor);
static void FOC_InitAdditionalMethods(uint8_t bMotor);
static void FOC_CalcCurrRef(uint8_t bMotor);
static uint16_t FOC_CurrController(uint8_t bMotor);
void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount);
bool TSK_ChargeBootCapDelayHasElapsedM1(void);
static void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount);
static bool TSK_StopPermanencyTimeHasElapsedM1(void);

void TSK_SafetyTask_PWMOFF(uint8_t motor);

static void TSK_MediumFrequencyTaskM2(void);
void TSK_SetChargeBootCapDelayM2(uint16_t hTickCount);
bool TSK_ChargeBootCapDelayHasElapsedM2(void);
static void TSK_SetStopPermanencyTimeM2(uint16_t SysTickCount);
static bool TSK_StopPermanencyTimeHasElapsedM2(void);

#define FOC_ARRAY_LENGTH 2
static void *FOC_array[FOC_ARRAY_LENGTH]={MC_NULL,MC_NULL};
static uint8_t FOC_array_head = 0; // Next obj to be executed
static uint8_t FOC_array_tail = 0; // Last arrived

void UI_Scheduler(void);

/* USER CODE BEGIN Private Functions */

/* USER CODE END Private Functions */
/**
  * @brief  It initializes the whole MC core according to user defined
  *         parameters.
  * @param  pMCIList pointer to the vector of MCInterface objects that will be
  *         created and initialized. The vector must have length equal to the
  *         number of motor drives.
  * @param  pMCTList pointer to the vector of MCTuning objects that will be
  *         created and initialized. The vector must have length equal to the
  *         number of motor drives.
  * @retval None
  */
void MCboot( MCI_Handle_t* pMCIList[NBR_OF_MOTORS],MCT_Handle_t* pMCTList[NBR_OF_MOTORS] )
{
  /* USER CODE BEGIN MCboot 0 */

  /* USER CODE END MCboot 0 */

  bMCBootCompleted = 0;
  pCLM[M1] = &CircleLimitationM1;
  pFW[M1] = &FW_M1; /* only if M1 has FW */

  pCLM[M2] = &CircleLimitationM2;
  pFW[M2] = &FW_M2; /* only if M2 has FW */

  /**********************************************************/
  /*    PWM and current sensing component initialization    */
  /**********************************************************/
  pwmcHandle[M1] = &PWM_Handle_M1._Super;
  R3F4XX_Init(&PWM_Handle_M1);
  pwmcHandle[M2] = &PWM_Handle_M2._Super;
  R3F4XX_Init(&PWM_Handle_M2); 
  /* USER CODE BEGIN MCboot 1 */

  /* USER CODE END MCboot 1 */

  /**************************************/
  /*    Start timers synchronously      */
  /**************************************/
  startTimers();    

  /**************************************/
  /*    State machine initialization    */
  /**************************************/
  STM_Init(&STM[M1]);
  
  /******************************************************/
  /*   PID component initialization: speed regulation   */
  /******************************************************/
  PID_HandleInit(&PIDSpeedHandle_M1);
  
  /******************************************************/
  /*   Main speed sensor component initialization       */
  /******************************************************/
  pPIDSpeed[M1] = &PIDSpeedHandle_M1;
  pSTC[M1] = &SpeednTorqCtrlM1;
  HALL_Init (&HALL_M1);
  
  /******************************************************/
  /*   Speed & torque component initialization          */
  /******************************************************/
  STC_Init(pSTC[M1],pPIDSpeed[M1], &HALL_M1._Super);
  
  /********************************************************/
  /*   PID component initialization: current regulation   */
  /********************************************************/
  PID_HandleInit(&PIDIqHandle_M1);
  PID_HandleInit(&PIDIdHandle_M1);
  pPIDIq[M1] = &PIDIqHandle_M1;
  pPIDId[M1] = &PIDIdHandle_M1;
  
  /********************************************************/
  /*   Bus voltage sensor component initialization        */
  /********************************************************/
  pBusSensorM1 = &RealBusVoltageSensorParamsM1;
  RVBS_Init(pBusSensorM1);
  
  /*************************************************/
  /*   Power measurement component initialization  */
  /*************************************************/
  pMPM[M1] = &PQD_MotorPowMeasM1;
  pMPM[M1]->pVBS = &(pBusSensorM1->_Super);
  pMPM[M1]->pFOCVars = &FOCVars[M1];
  
  /*******************************************************/
  /*   Temperature measurement component initialization  */
  /*******************************************************/
  NTC_Init(&TempSensorParamsM1);    
  pTemperatureSensor[M1] = &TempSensorParamsM1;
    
  /*******************************************************/
  /*   Flux weakening component initialization           */
  /*******************************************************/
  PID_HandleInit(&PIDFluxWeakeningHandle_M1);
  FW_Init(pFW[M1],pPIDSpeed[M1],&PIDFluxWeakeningHandle_M1);   
             

  pREMNG[M1] = &RampExtMngrHFParamsM1;
  REMNG_Init(pREMNG[M1]);

  FOC_Clear(M1);
  FOCVars[M1].bDriveInput = EXTERNAL;
  FOCVars[M1].Iqdref = STC_GetDefaultIqdref(pSTC[M1]);
  FOCVars[M1].UserIdref = STC_GetDefaultIqdref(pSTC[M1]).qI_Component2;
  oMCInterface[M1] = & Mci[M1];
  MCI_Init(oMCInterface[M1], &STM[M1], pSTC[M1], &FOCVars[M1]);
  MCI_ExecSpeedRamp(oMCInterface[M1],
  STC_GetMecSpeedRef01HzDefault(pSTC[M1]),0); /*First command to STC*/
  pMCIList[M1] = oMCInterface[M1];
  MCT[M1].pPIDSpeed = pPIDSpeed[M1];
  MCT[M1].pPIDIq = pPIDIq[M1];
  MCT[M1].pPIDId = pPIDId[M1];
  MCT[M1].pPIDFluxWeakening = &PIDFluxWeakeningHandle_M1; /* only if M1 has FW */
  MCT[M1].pPWMnCurrFdbk = pwmcHandle[M1];
  MCT[M1].pRevupCtrl = MC_NULL;              /* only if M1 is not sensorless*/
  MCT[M1].pSpeedSensorMain = (SpeednPosFdbk_Handle_t *) &HALL_M1; 
  MCT[M1].pSpeedSensorAux = MC_NULL;
  MCT[M1].pSpeedSensorVirtual = MC_NULL;
  MCT[M1].pSpeednTorqueCtrl = pSTC[M1];
  MCT[M1].pStateMachine = &STM[M1];
  MCT[M1].pTemperatureSensor = (NTC_Handle_t *) pTemperatureSensor[M1];
  MCT[M1].pBusVoltageSensor = &(pBusSensorM1->_Super);
  MCT[M1].pBrakeDigitalOutput = MC_NULL;   /* brake is defined, oBrakeM1*/
  MCT[M1].pNTCRelay = MC_NULL;             /* relay is defined, oRelayM1*/
  MCT[M1].pMPM =  (MotorPowMeas_Handle_t*)pMPM[M1];
  MCT[M1].pFW = pFW[M1];
  MCT[M1].pFF = MC_NULL;
  MCT[M1].pSCC = MC_NULL;
  MCT[M1].pOTT = MC_NULL;
  pMCTList[M1] = &MCT[M1];

  /******************************************************/
  /*   Motor 2 features initialization                  */
  /******************************************************/
  STM_Init(&STM[M2]);
  pSTC[M2] = &SpeednTorqCtrlM2;
  PID_HandleInit(&PIDSpeedHandle_M2);
  pPIDSpeed[M2] = &PIDSpeedHandle_M2;
  HALL_Init (&HALL_M2);
  STC_Init(pSTC[M2],pPIDSpeed[M2], &HALL_M2._Super);
 
  PID_HandleInit(&PIDIqHandle_M2);
  PID_HandleInit(&PIDIdHandle_M2);
  pPIDIq[M2] = &PIDIqHandle_M2;
  pPIDId[M2] = &PIDIdHandle_M2;
  pBusSensorM2 = &VirtualBusVoltageSensorParamsM2; /* powerboard configuration: Rdivider or Virtual*/
  VVBS_Init(pBusSensorM2);
  
  //Power Measurement M2
  pMPM[M2] = &PQD_MotorPowMeasM2;
  pMPM[M2]->pVBS = &(pBusSensorM2->_Super);
  pMPM[M2]->pFOCVars = &FOCVars[M2];

  NTC_Init(&TempSensorParamsM2);  
  pTemperatureSensor[M2] = &TempSensorParamsM2;

  PID_HandleInit(&PIDFluxWeakeningHandle_M2);
  FW_Init(pFW[M2],pPIDSpeed[M2],&PIDFluxWeakeningHandle_M2);    /* only if M2 has FW */
  pREMNG[M2] = &RampExtMngrHFParamsM2;
  REMNG_Init(pREMNG[M2]);
  FOC_Clear(M2);
  FOCVars[M2].bDriveInput = EXTERNAL;
  FOCVars[M2].Iqdref = STC_GetDefaultIqdref(pSTC[M2]);
  FOCVars[M2].UserIdref = STC_GetDefaultIqdref(pSTC[M2]).qI_Component2;
  oMCInterface[M2] = &Mci[M2];
  MCI_Init(oMCInterface[M2], &STM[M2], pSTC[M2], &FOCVars[M2]);
  MCI_ExecSpeedRamp(oMCInterface[M2],
  STC_GetMecSpeedRef01HzDefault(pSTC[M2]),0); /*First command to STC*/
  pMCIList[M2] = oMCInterface[M2];
  MCT[M2].pPIDSpeed = pPIDSpeed[M2];
  MCT[M2].pPIDIq = pPIDIq[M2];
  MCT[M2].pPIDId = pPIDId[M2];
  MCT[M2].pPIDFluxWeakening = &PIDFluxWeakeningHandle_M2; /* only if M2 has FW */
  MCT[M2].pPWMnCurrFdbk = pwmcHandle[M2];
  MCT[M2].pRevupCtrl = MC_NULL;              /* only if M2 is not sensorless*/
  MCT[M2].pSpeedSensorMain = (SpeednPosFdbk_Handle_t *) &HALL_M2;
  MCT[M2].pSpeedSensorAux = MC_NULL;
  MCT[M2].pSpeedSensorVirtual = MC_NULL;
  MCT[M2].pSpeednTorqueCtrl = pSTC[M2];
  MCT[M2].pStateMachine = &STM[M2];
  MCT[M2].pTemperatureSensor = (NTC_Handle_t *) pTemperatureSensor[M2];
  MCT[M2].pBusVoltageSensor = &(pBusSensorM2->_Super);
  MCT[M2].pBrakeDigitalOutput = MC_NULL;   /* brake is defined, oBrakeM2*/
  MCT[M2].pNTCRelay = MC_NULL;             /* relay is defined, oRelayM2*/
  MCT[M2].pMPM = (MotorPowMeas_Handle_t*)pMPM[M2];
  MCT[M2].pFW = pFW[M2];
  MCT[M2].pFF = MC_NULL;
  MCT[M2].pSCC = MC_NULL;
  pMCTList[M2] = &MCT[M2];
 
 

  /* USER CODE BEGIN MCboot 2 */

  /* USER CODE END MCboot 2 */

  bMCBootCompleted = 1;
}

/**
 * @brief Runs all the Tasks of the Motor Control cockpit
 *
 * This function is to be called periodically at least at the Medium Frequency task
 * rate (It is typically called on the Systick interrupt). Exact invokation rate is 
 * the Speed regulator execution rate set in the Motor Contorl Workbench.
 *
 * The following tasks are executed in this order:
 *
 * - Medium Frequency Tasks of each motors
 * - Safety Task
 * - Power Factor Correction Task (if enabled)
 * - User Interface task. 
 */
void MC_RunMotorControlTasks(void)
{
  if ( bMCBootCompleted ) {
    /* ** Medium Frequency Tasks ** */
    MC_Scheduler();

    /* Safety task is run after Medium Frequency task so that  
     * it can overcome actions they initiated if needed. */
    TSK_SafetyTask();
    

    /* ** User Interface Task ** */
    UI_Scheduler();
  }
}

/**
 * @brief  Executes the Medium Frequency Task functions for each drive instance. 
 *
 * It is to be clocked at the Systick frequency.
 */
void MC_Scheduler(void)
{
/* USER CODE BEGIN MC_Scheduler 0 */

/* USER CODE END MC_Scheduler 0 */

  if (bMCBootCompleted == 1)
  {    
    if(hMFTaskCounterM1 > 0u)
    {
      hMFTaskCounterM1--;
    }
    else
    {
      TSK_MediumFrequencyTaskM1();
      /* USER CODE BEGIN MC_Scheduler 1 */

      /* USER CODE END MC_Scheduler 1 */
      hMFTaskCounterM1 = MF_TASK_OCCURENCE_TICKS;
    }
    if(hMFTaskCounterM2 > 0u)
    {
      hMFTaskCounterM2--;
    }
    else
    {
      TSK_MediumFrequencyTaskM2();
      /* USER CODE BEGIN MC_Scheduler MediumFrequencyTask M2 */

      /* USER CODE END MC_Scheduler MediumFrequencyTask M2 */
      hMFTaskCounterM2 = MF_TASK_OCCURENCE_TICKS2;
    }
    if(hBootCapDelayCounterM1 > 0u)
    {
      hBootCapDelayCounterM1--;
    }
    if(hStopPermanencyCounterM1 > 0u)
    {
      hStopPermanencyCounterM1--;
    }
    if(hBootCapDelayCounterM2 > 0u)
    {
      hBootCapDelayCounterM2--;
    }
    if(hStopPermanencyCounterM2 > 0u)
    {
      hStopPermanencyCounterM2--;
    }
  }
  else
  {
  }
  /* USER CODE BEGIN MC_Scheduler 2 */

  /* USER CODE END MC_Scheduler 2 */
}

/**
  * @brief Executes medium frequency periodic Motor Control tasks
  *
  * This function performs some of the control duties on Motor 1 according to the 
  * present state of its state machine. In particular, duties requiring a periodic 
  * execution at a medium frequency rate (such as the speed controller for instance) 
  * are executed here.
  */
void TSK_MediumFrequencyTaskM1(void)
{
  /* USER CODE BEGIN MediumFrequencyTask M1 0 */

  /* USER CODE END MediumFrequencyTask M1 0 */

  State_t StateM1;
  int16_t wAux = 0;

  (void) HALL_CalcAvrgMecSpeed01Hz( &HALL_M1, &wAux );
  PQD_CalcElMotorPower( pMPM[M1] );

  StateM1 = STM_GetState( &STM[M1] );

  switch ( StateM1 )
  {
  case IDLE_START:
    R3F4XX_TurnOnLowSides( pwmcHandle[M1] );
    TSK_SetChargeBootCapDelayM1( CHARGE_BOOT_CAP_TICKS );
    STM_NextState( &STM[M1], CHARGE_BOOT_CAP );
    break;

  case CHARGE_BOOT_CAP:
    if ( TSK_ChargeBootCapDelayHasElapsedM1() )
    {
      PWMC_CurrentReadingCalibr( pwmcHandle[M1], CRC_START );

      /* USER CODE BEGIN MediumFrequencyTask M1 Charge BootCap elapsed */

      /* USER CODE END MediumFrequencyTask M1 Charge BootCap elapsed */

      STM_NextState(&STM[M1],OFFSET_CALIB);
    }
    break;

  case OFFSET_CALIB:
    if ( PWMC_CurrentReadingCalibr( pwmcHandle[M1], CRC_EXEC ) )
    {
      STM_NextState( &STM[M1], CLEAR );
    }
    break;

  case CLEAR:
    HALL_Clear( &HALL_M1 );

    if ( STM_NextState( &STM[M1], START ) == true )
    {
      FOC_Clear( M1 );

      R3F4XX_SwitchOnPWM( pwmcHandle[M1] );
    }
    break;

  case START:
    {
      STM_NextState( &STM[M1], START_RUN ); /* only for sensored*/
    }
    break;
    
  case START_RUN:
    {
      /* USER CODE BEGIN MediumFrequencyTask M1 1 */

      /* USER CODE END MediumFrequencyTask M1 1 */      
	  FOC_InitAdditionalMethods(M1);
      FOC_CalcCurrRef( M1 );
      STM_NextState( &STM[M1], RUN );
    }
    STC_ForceSpeedReferenceToCurrentSpeed( pSTC[M1] ); /* Init the reference speed to current speed */
    MCI_ExecBufferedCommands( oMCInterface[M1] ); /* Exec the speed ramp after changing of the speed sensor */
	
    break;

  case RUN:
    /* USER CODE BEGIN MediumFrequencyTask M1 2 */

    /* USER CODE END MediumFrequencyTask M1 2 */

    MCI_ExecBufferedCommands( oMCInterface[M1] );
    FOC_CalcCurrRef( M1 );
 
 
    /* USER CODE BEGIN MediumFrequencyTask M1 3 */

    /* USER CODE END MediumFrequencyTask M1 3 */
    break;

  case ANY_STOP:
    R3F4XX_SwitchOffPWM( pwmcHandle[M1] );
    FOC_Clear( M1 );
    MPM_Clear( (MotorPowMeas_Handle_t*) pMPM[M1] );
    TSK_SetStopPermanencyTimeM1( STOPPERMANENCY_TICKS );

    /* USER CODE BEGIN MediumFrequencyTask M1 4 */

    /* USER CODE END MediumFrequencyTask M1 4 */

    STM_NextState( &STM[M1], STOP );
    break;

  case STOP:
    if ( TSK_StopPermanencyTimeHasElapsedM1() )
    {
      STM_NextState( &STM[M1], STOP_IDLE );
    }
    break;

  case STOP_IDLE:
    /* USER CODE BEGIN MediumFrequencyTask M1 5 */

    /* USER CODE END MediumFrequencyTask M1 5 */
    STM_NextState( &STM[M1], IDLE );
    break;

  default:
    break;
  }

  /* USER CODE BEGIN MediumFrequencyTask M1 6 */

  /* USER CODE END MediumFrequencyTask M1 6 */
}

/**
  * @brief  It re-initializes the current and voltage variables. Moreover
  *         it clears qd currents PI controllers, voltage sensor and SpeednTorque
  *         controller. It must be called before each motor restart.
  *         It does not clear speed sensor.
  * @param  bMotor related motor it can be M1 or M2
  * @retval none
  */
void FOC_Clear(uint8_t bMotor)
{
  /* USER CODE BEGIN FOC_Clear 0 */

  /* USER CODE END FOC_Clear 0 */
  
  Curr_Components Inull = {(int16_t)0, (int16_t)0};
  Volt_Components Vnull = {(int16_t)0, (int16_t)0};

  FOCVars[bMotor].Iab = Inull;
  FOCVars[bMotor].Ialphabeta = Inull;
  FOCVars[bMotor].Iqd = Inull;
  FOCVars[bMotor].Iqdref = Inull;
  FOCVars[bMotor].hTeref = (int16_t)0;
  FOCVars[bMotor].Vqd = Vnull;
  FOCVars[bMotor].Valphabeta = Vnull;
  FOCVars[bMotor].hElAngle = (int16_t)0;

  PID_SetIntegralTerm(pPIDIq[bMotor], (int32_t)0);
  PID_SetIntegralTerm(pPIDId[bMotor], (int32_t)0);

  STC_Clear(pSTC[bMotor]);

  PWMC_SwitchOffPWM(pwmcHandle[bMotor]);

  if (pFW[bMotor])
  {
    FW_Clear(pFW[bMotor]);
  }
  /* USER CODE BEGIN FOC_Clear 1 */

  /* USER CODE END FOC_Clear 1 */
}

/**
  * @brief  Use this method to initialize additional methods (if any) in
  *         START_TO_RUN state
  * @param  bMotor related motor it can be M1 or M2
  * @retval none
  */
void FOC_InitAdditionalMethods(uint8_t bMotor)
{
  /* USER CODE BEGIN FOC_InitAdditionalMethods 0 */

  /* USER CODE END FOC_InitAdditionalMethods 0 */
}

/**
  * @brief  It computes the new values of Iqdref (current references on qd
  *         reference frame) based on the required electrical torque information
  *         provided by oTSC object (internally clocked).
  *         If implemented in the derived class it executes flux weakening and/or
  *         MTPA algorithm(s). It must be called with the periodicity specified
  *         in oTSC parameters
  * @param  bMotor related motor it can be M1 or M2
  * @retval none
  */
void FOC_CalcCurrRef(uint8_t bMotor)
{
  Curr_Components IqdTmp;
    
  /* USER CODE BEGIN FOC_CalcCurrRef 0 */

  /* USER CODE END FOC_CalcCurrRef 0 */
  if(FOCVars[bMotor].bDriveInput == INTERNAL)
  {
    FOCVars[bMotor].hTeref = STC_CalcTorqueReference(pSTC[bMotor]);
    FOCVars[bMotor].Iqdref.qI_Component1 = FOCVars[bMotor].hTeref;

    if (pFW[bMotor])
    {
      IqdTmp.qI_Component1 = FOCVars[bMotor].Iqdref.qI_Component1;
      IqdTmp.qI_Component2 = FOCVars[bMotor].UserIdref; 
      FOCVars[bMotor].Iqdref = FW_CalcCurrRef(pFW[bMotor],IqdTmp); 
    }
  }
  /* USER CODE BEGIN FOC_CalcCurrRef 1 */

  /* USER CODE END FOC_CalcCurrRef 1 */
}

/**
  * @brief  It set a counter intended to be used for counting the delay required
  *         for drivers boot capacitors charging of motor 1
  * @param  hTickCount number of ticks to be counted
  * @retval void
  */
void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount)
{
   hBootCapDelayCounterM1 = hTickCount;
}

/**
  * @brief  Use this function to know whether the time required to charge boot
  *         capacitors of motor 1 has elapsed
  * @param  none
  * @retval bool true if time has elapsed, false otherwise
  */
bool TSK_ChargeBootCapDelayHasElapsedM1(void)
{
  bool retVal = false;
  if (hBootCapDelayCounterM1 == 0)
  {
    retVal = true;
  }
  return (retVal);
}

/**
  * @brief  It set a counter intended to be used for counting the permanency
  *         time in STOP state of motor 1
  * @param  hTickCount number of ticks to be counted
  * @retval void
  */
void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount)
{
  hStopPermanencyCounterM1 = hTickCount;
}

/**
  * @brief  Use this function to know whether the permanency time in STOP state
  *         of motor 1 has elapsed
  * @param  none
  * @retval bool true if time is elapsed, false otherwise
  */
bool TSK_StopPermanencyTimeHasElapsedM1(void)
{
  bool retVal = false;
  if (hStopPermanencyCounterM1 == 0)
  {
    retVal = true;
  }
  return (retVal);
}

#if defined (CCMRAM_ENABLED)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief Executes medium frequency periodic Motor Control tasks
  *
  * This function performs some of the control duties on Motor 2 according to the 
  * present state of its state machine. In particular, duties requiring a periodic 
  * execution at a medium frequency rate (such as the speed controller for instance) 
  * are executed here.
  */
void TSK_MediumFrequencyTaskM2(void)
{
  /* USER CODE BEGIN MediumFrequencyTask M2 0 */

  /* USER CODE END MediumFrequencyTask M2 0 */
  State_t StateM2;
  int16_t wAux = 0;

  (void) HALL_CalcAvrgMecSpeed01Hz( &HALL_M2, &wAux );
  PQD_CalcElMotorPower( pMPM[M2] );

  StateM2 = STM_GetState( &STM[M2] );

  switch ( StateM2 )
  {
  case IDLE_START:
    R3F4XX_TurnOnLowSides( pwmcHandle[M2] );
    TSK_SetChargeBootCapDelayM2( CHARGE_BOOT_CAP_TICKS2 );
    STM_NextState( &STM[M2], CHARGE_BOOT_CAP );
    break;

  case CHARGE_BOOT_CAP:
    if ( TSK_ChargeBootCapDelayHasElapsedM2() )
    {
      PWMC_CurrentReadingCalibr( pwmcHandle[M2], CRC_START );

      /* USER CODE BEGIN MediumFrequencyTask M2 Charge BootCap elapsed */

      /* USER CODE END MediumFrequencyTask M2 Charge BootCap elapsed */

      STM_NextState( &STM[M2], OFFSET_CALIB );
    }
    break;

  case OFFSET_CALIB:
    if ( PWMC_CurrentReadingCalibr( pwmcHandle[M2], CRC_EXEC ) )
    {
      STM_NextState( &STM[M2], CLEAR );
    }
    break;

  case CLEAR:
    HALL_Clear( &HALL_M2 );

    if ( STM_NextState( &STM[M2], START ) == true )
    {
      FOC_Clear(M2);
      R3F4XX_SwitchOnPWM( pwmcHandle[M2] );
    }
    break;

  case START:
    {
      STM_NextState( &STM[M2], START_RUN ); 
    }
    break;

  case START_RUN:
    {
      /* USER CODE BEGIN MediumFrequencyTask M2 1 */

      /* USER CODE END MediumFrequencyTask M2 1 */ 

      FOC_InitAdditionalMethods( M2 );
      FOC_CalcCurrRef( M2 );

      STM_NextState(&STM[M2], RUN);
    }
    STC_ForceSpeedReferenceToCurrentSpeed(pSTC[M2]); /* Init the reference speed to current speed */
    MCI_ExecBufferedCommands(oMCInterface[M2]); /* Exec the speed ramp after changing of the speed sensor */
    break;

  case RUN:
    /* USER CODE BEGIN MediumFrequencyTask M2 2 */

    /* USER CODE END MediumFrequencyTask M2 2 */

    MCI_ExecBufferedCommands( oMCInterface[M2] );
    FOC_CalcCurrRef( M2 );
 
 
    /* USER CODE BEGIN MediumFrequencyTask M2 3 */

    /* USER CODE END MediumFrequencyTask M2 3 */
    break;

  case ANY_STOP:
    R3F4XX_SwitchOffPWM( pwmcHandle[M2] );
    FOC_Clear( M2 );
    MPM_Clear( (MotorPowMeas_Handle_t*) pMPM[M2] );
    TSK_SetStopPermanencyTimeM2( STOPPERMANENCY_TICKS2 );

    /* USER CODE BEGIN MediumFrequencyTask M2 4 */

    /* USER CODE END MediumFrequencyTask M2 4 */

    STM_NextState(&STM[M2], STOP);
    break;

  case STOP:
    if ( TSK_StopPermanencyTimeHasElapsedM2() )
    {
      STM_NextState( &STM[M2], STOP_IDLE );
    }
    break;

  case STOP_IDLE:
    /* USER CODE BEGIN MediumFrequencyTask M2 5 */

    /* USER CODE END MediumFrequencyTask M2 5 */
    STM_NextState( &STM[M2], IDLE );
    break;

  default:
    break;
  }

  /* USER CODE BEGIN MediumFrequencyTask M2 6 */

  /* USER CODE END MediumFrequencyTask M2 6 */  
}

/**
  * @brief  It set a counter intended to be used for counting the delay required
  *         for drivers boot capacitors charging of motor 2
  * @param  hTickCount number of ticks to be counted
  * @retval void
  */
void TSK_SetChargeBootCapDelayM2(uint16_t hTickCount)
{
   hBootCapDelayCounterM2 = hTickCount;
}

/**
  * @brief  Use this function to know whether the time required to charge boot
  *         capacitors of motor 2 has elapsed
  * @param  none
  * @retval bool true if time has elapsed, false otherwise
  */
bool TSK_ChargeBootCapDelayHasElapsedM2(void)
{
  bool retVal = false;
  if (hBootCapDelayCounterM2 == 0)
  {
    retVal = true;
  }
  return (retVal);
}

/**
  * @brief  It set a counter intended to be used for counting the permanency
  *         time in STOP state of motor 2
  * @param  hTickCount number of ticks to be counted
  * @retval void
  */
void TSK_SetStopPermanencyTimeM2(uint16_t hTickCount)
{
  hStopPermanencyCounterM2 = hTickCount;
}

/**
  * @brief  Use this function to know whether the permanency time in STOP state
  *         of motor 2 has elapsed
  * @param  none
  * @retval bool true if time is elapsed, false otherwise
  */
bool TSK_StopPermanencyTimeHasElapsedM2(void)
{
  bool retVal = false;
  if (hStopPermanencyCounterM2 == 0)
  {
    retVal = true;
  }
  return (retVal);
}
 

#if defined (CCMRAM_ENABLED)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Executes the Motor Control duties that require a high frequency rate and a precise timing
  *
  *  This is mainly the FOC current control loop. It is executed depending on the state of the Motor Control 
  * subsystem (see the state machine(s)).
  *
  * @retval Number of the  motor instance which FOC loop was executed.
  */
uint8_t TSK_HighFrequencyTask(void)
{
  /* USER CODE BEGIN HighFrequencyTask 0 */

  /* USER CODE END HighFrequencyTask 0 */
  
  uint8_t bMotorNbr = 0;
  uint16_t hFOCreturn;
 

  bMotorNbr = *((uint8_t*)(FOC_array[FOC_array_head]));
  if (bMotorNbr == M1)
  {

  HALL_CalcElAngle (&HALL_M1);  
  /* USER CODE BEGIN HighFrequencyTask DUALDRIVE_1 */

  /* USER CODE END HighFrequencyTask DUALDRIVE_1 */
    hFOCreturn = FOC_CurrController(M1);
  /* USER CODE BEGIN HighFrequencyTask DUALDRIVE_2 */

  /* USER CODE END HighFrequencyTask DUALDRIVE_2 */
  }
  else /* bMotorNbr != M1 */
  {
   HALL_CalcElAngle (&HALL_M2);  
  /* USER CODE BEGIN HighFrequencyTask DUALDRIVE_3 */

  /* USER CODE END HighFrequencyTask DUALDRIVE_3 */
    hFOCreturn = FOC_CurrController(M2);
  /* USER CODE BEGIN HighFrequencyTask DUALDRIVE_4 */

  /* USER CODE END HighFrequencyTask DUALDRIVE_4 */
  }
  if(hFOCreturn == MC_FOC_DURATION)
  {
    STM_FaultProcessing(&STM[bMotorNbr], MC_FOC_DURATION, 0);
  }
  else
  {
    if (bMotorNbr == M1)
    {
    }
    else // bMotorNbr != M1
    {
  
    }    
  }
  FOC_array_head++;
  if (FOC_array_head == FOC_ARRAY_LENGTH)
  {
    FOC_array_head = 0;
  }
  /* USER CODE BEGIN HighFrequencyTask 1 */

  /* USER CODE END HighFrequencyTask 1 */

  return bMotorNbr;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief It executes the core of FOC drive that is the controllers for Iqd
  *        currents regulation. Reference frame transformations are carried out
  *        accordingly to the active speed sensor. It must be called periodically
  *        when new motor currents have been converted
  * @param this related object of class CFOC.
  * @retval int16_t It returns MC_NO_FAULTS if the FOC has been ended before
  *         next PWM Update event, MC_FOC_DURATION otherwise
  */
inline uint16_t FOC_CurrController(uint8_t bMotor)
{
  Curr_Components Iab, Ialphabeta, Iqd;
  Volt_Components Valphabeta, Vqd;
  int16_t hElAngle;
  uint16_t hCodeError;

  hElAngle = SPD_GetElAngle(STC_GetSpeedSensor(pSTC[bMotor]));
  PWMC_GetPhaseCurrents(pwmcHandle[bMotor], &Iab);
  Ialphabeta = MCM_Clarke(Iab);
  Iqd = MCM_Park(Ialphabeta, hElAngle);
  Vqd.qV_Component1 = PI_Controller(pPIDIq[bMotor],
            (int32_t)(FOCVars[bMotor].Iqdref.qI_Component1) - Iqd.qI_Component1);

  Vqd.qV_Component2 = PI_Controller(pPIDId[bMotor],
            (int32_t)(FOCVars[bMotor].Iqdref.qI_Component2) - Iqd.qI_Component2);
  FOCVars[bMotor].Vqd = Vqd;
  Vqd = Circle_Limitation(pCLM[bMotor], Vqd);
  Valphabeta = MCM_Rev_Park(Vqd, hElAngle);
  hCodeError = PWMC_SetPhaseVoltage(pwmcHandle[bMotor], Valphabeta);
  FOCVars[bMotor].Iab = Iab;
  FOCVars[bMotor].Ialphabeta = Ialphabeta;
  FOCVars[bMotor].Iqd = Iqd;
  FOCVars[bMotor].Valphabeta = Valphabeta;
  FOCVars[bMotor].hElAngle = hElAngle;
  if (pFW[bMotor])
  {
    FW_DataProcess(pFW[bMotor], Vqd);
  }
  return(hCodeError);
}

/**
  * @brief  Executes safety checks (e.g. bus voltage and temperature) for all drive instances. 
  *
  * Faults flags are updated here.
  */
void TSK_SafetyTask(void)
{
  /* USER CODE BEGIN TSK_SafetyTask 0 */

  /* USER CODE END TSK_SafetyTask 0 */
  if (bMCBootCompleted == 1)
  {  
    TSK_SafetyTask_PWMOFF(M1);
    /* Second drive */
    TSK_SafetyTask_PWMOFF(M2);
    /* User conversion execution */
    RCM_ExecUserConv ();
  /* USER CODE BEGIN TSK_SafetyTask 1 */

  /* USER CODE END TSK_SafetyTask 1 */
  }
}

/**
  * @brief  Safety task implementation if  MC.ON_OVER_VOLTAGE == TURN_OFF_PWM
  * @param  bMotor Motor reference number defined
  *         \link Motors_reference_number here \endlink
  * @retval None
  */
void TSK_SafetyTask_PWMOFF(uint8_t bMotor)
{
  /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 0 */

  /* USER CODE END TSK_SafetyTask_PWMOFF 0 */
  
  uint16_t CodeReturn = MC_NO_ERROR;
  uint16_t errMask[NBR_OF_MOTORS] = {VBUS_TEMP_ERR_MASK, VBUS_TEMP_ERR_MASK2};

  CodeReturn |= errMask[bMotor] & NTC_CalcAvTemp(pTemperatureSensor[bMotor]); /* check for fault if FW protection is activated. It returns MC_OVER_TEMP or MC_NO_ERROR */
  CodeReturn |= PWMC_CheckOverCurrent(pwmcHandle[bMotor]);                    /* check for fault. It return MC_BREAK_IN or MC_NO_FAULTS 
                                                                                 (for STM32F30x can return MC_OVER_VOLT in case of HW Overvoltage) */
  if(bMotor == M1)
  {
    CodeReturn |=  errMask[bMotor] &RVBS_CalcAvVbus(pBusSensorM1);
  }

  STM_FaultProcessing(&STM[bMotor], CodeReturn, ~CodeReturn); /* Update the STM according error code */
  switch (STM_GetState(&STM[bMotor])) /* Acts on PWM outputs in case of faults */
  {
  case FAULT_NOW:
    PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
    FOC_Clear(bMotor);
    MPM_Clear((MotorPowMeas_Handle_t*)pMPM[bMotor]);
    /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 1 */

    /* USER CODE END TSK_SafetyTask_PWMOFF 1 */
    break;
  case FAULT_OVER:
    PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
	/* USER CODE BEGIN TSK_SafetyTask_PWMOFF 2 */

    /* USER CODE END TSK_SafetyTask_PWMOFF 2 */
    break;
  default:
    break;
  }
  /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 3 */

  /* USER CODE END TSK_SafetyTask_PWMOFF 3 */
}

#if defined (CCMRAM_ENABLED)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief Reserves FOC execution on ADC ISR half a PWM period in advance
  *
  *  This function is called by TIMx_UP_IRQHandler in case of dual MC and
  * it allows to reserve half PWM period in advance the FOC execution on
  * ADC ISR
  * @param  pDrive Pointer on the FOC Array
  */
void TSK_DualDriveFIFOUpdate(void *pDrive)
{
  FOC_array[FOC_array_tail] = pDrive;
  FOC_array_tail++;
  if (FOC_array_tail == FOC_ARRAY_LENGTH)
  {
    FOC_array_tail = 0;
  }
}

/**
  * @brief  This function returns the reference of the MCInterface relative to
  *         the selected drive.
  * @param  bMotor Motor reference number defined
  *         \link Motors_reference_number here \endlink
  * @retval MCI_Handle_t * Reference to MCInterface relative to the selected drive.
  *         Note: it can be MC_NULL if MCInterface of selected drive is not
  *         allocated.
  */
MCI_Handle_t * GetMCI(uint8_t bMotor)
{
  MCI_Handle_t * retVal = MC_NULL;
  if ((oMCInterface != MC_NULL) && (bMotor < NBR_OF_MOTORS))
  {
    retVal = oMCInterface[bMotor];
  }
  return retVal;
}

/**
  * @brief  This function returns the reference of the MCTuning relative to
  *         the selected drive.
  * @param  bMotor Motor reference number defined
  *         \link Motors_reference_number here \endlink
  * @retval MCT_Handle_t motor control tuning handler for the selected drive.
  *         Note: it can be MC_NULL if MCInterface of selected drive is not
  *         allocated.
  */
MCT_Handle_t* GetMCT(uint8_t bMotor)
{
  MCT_Handle_t* retVal = MC_NULL;
  if (bMotor < NBR_OF_MOTORS)
  {
    retVal = &MCT[bMotor];
  }
  return retVal;
}

/**
  * @brief  Puts the Motor Control subsystem in in safety conditions on a Hard Fault
  *
  *  This function is to be executed when a general hardware failure has been detected  
  * by the microcontroller and is used to put the system in safety condition.
  */
void TSK_HardwareFaultTask(void)
{
  /* USER CODE BEGIN TSK_HardwareFaultTask 0 */

  /* USER CODE END TSK_HardwareFaultTask 0 */
  
  R3F4XX_SwitchOffPWM(pwmcHandle[M1]);
  STM_FaultProcessing(&STM[M1], MC_SW_ERROR, 0);
  R3F4XX_SwitchOffPWM(pwmcHandle[M2]);
  STM_FaultProcessing(&STM[M2], MC_SW_ERROR, 0);
  /* USER CODE BEGIN TSK_HardwareFaultTask 1 */

  /* USER CODE END TSK_HardwareFaultTask 1 */
}
 /**
  * @brief  Locks GPIO pins used for Motor Control to prevent accidental reconfiguration 
  */
void mc_lock_pins (void)
{
LL_GPIO_LockPin(M2_PWM_UL_GPIO_Port, M2_PWM_UL_Pin);
LL_GPIO_LockPin(M2_PWM_VL_GPIO_Port, M2_PWM_VL_Pin);
LL_GPIO_LockPin(M2_PWM_WL_GPIO_Port, M2_PWM_WL_Pin);
LL_GPIO_LockPin(M2_PWM_UH_GPIO_Port, M2_PWM_UH_Pin);
LL_GPIO_LockPin(M2_PWM_VH_GPIO_Port, M2_PWM_VH_Pin);
LL_GPIO_LockPin(M2_OCP_GPIO_Port, M2_OCP_Pin);
LL_GPIO_LockPin(M2_PWM_WH_GPIO_Port, M2_PWM_WH_Pin);
LL_GPIO_LockPin(M2_HALL_H1_GPIO_Port, M2_HALL_H1_Pin);
LL_GPIO_LockPin(M2_HALL_H2_GPIO_Port, M2_HALL_H2_Pin);
LL_GPIO_LockPin(M2_HALL_H3_GPIO_Port, M2_HALL_H3_Pin);
LL_GPIO_LockPin(M1_CURR_AMPL_W_GPIO_Port, M1_CURR_AMPL_W_Pin);
LL_GPIO_LockPin(M2_CURR_AMPL_W_GPIO_Port, M2_CURR_AMPL_W_Pin);
LL_GPIO_LockPin(M1_HALL_H2_GPIO_Port, M1_HALL_H2_Pin);
LL_GPIO_LockPin(M1_HALL_H3_GPIO_Port, M1_HALL_H3_Pin);
LL_GPIO_LockPin(M1_HALL_H1_GPIO_Port, M1_HALL_H1_Pin);
LL_GPIO_LockPin(M1_PWM_UH_GPIO_Port, M1_PWM_UH_Pin);
LL_GPIO_LockPin(M1_PWM_VH_GPIO_Port, M1_PWM_VH_Pin);
LL_GPIO_LockPin(M1_OCP_GPIO_Port, M1_OCP_Pin);
LL_GPIO_LockPin(M1_PWM_WH_GPIO_Port, M1_PWM_WH_Pin);
LL_GPIO_LockPin(M1_PWM_VL_GPIO_Port, M1_PWM_VL_Pin);
LL_GPIO_LockPin(M1_PWM_WL_GPIO_Port, M1_PWM_WL_Pin);
LL_GPIO_LockPin(M1_PWM_UL_GPIO_Port, M1_PWM_UL_Pin);
LL_GPIO_LockPin(M2_CURR_AMPL_V_GPIO_Port, M2_CURR_AMPL_V_Pin);
LL_GPIO_LockPin(M2_CURR_AMPL_U_GPIO_Port, M2_CURR_AMPL_U_Pin);
LL_GPIO_LockPin(M1_CURR_AMPL_V_GPIO_Port, M1_CURR_AMPL_V_Pin);
LL_GPIO_LockPin(M1_BUS_VOLTAGE_GPIO_Port, M1_BUS_VOLTAGE_Pin);
LL_GPIO_LockPin(M1_CURR_AMPL_U_GPIO_Port, M1_CURR_AMPL_U_Pin);
}

/* USER CODE BEGIN mc_task 0 */

/* USER CODE END mc_task 0 */

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
