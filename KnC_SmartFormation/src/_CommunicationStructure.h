/*
 * _CommunicationStructure.h
 *
 * Created: 2021-11-26 AM 00:05:00
 *  Author: 4N Corporation, bgyu
 */ 


#ifndef _COMMUNICATIONSTRUCTURE_H_
#define _COMMUNICATIONSTRUCTURE_H_

#pragma pack(1)

#define	_MAX_CHANNEL					4
#define	_MAX_SEQUENCE_STEP				255
#define	_MAX_PULSE_STEP					10
#define _MUX_MAX_CHANNEL				8		//2023.04.24 LJK
#define MAX_DCIR_10MS					15		//2024.10.14 LJK DCIR 10ms 
#define MAX_DCIR_SEC					65		//2024.10.14 LJK DCIR Sec 

#define	CALIBRATION_MAX_RANGE		(_MAX_CHANNEL*4)		// Voltage 0, Discharge Current 4, Charge Current 8, Voltage2 12
#define	CALIBRATION_MAX_STEP		11

#define __DAC   0
#define __ADC   1

#define _NOR_STEP_RECORD	1
#define _END_STEP_RECORD	2
enum
{
	_PAUSE_NONE = 0,	
	_PAUSE_ON,	
	_RESUME_ON,
	_RESUME_NEXT_STEP,
};
enum
{
	_EEPRom_NONE = 0,
	_EEPRom_READ_ALL,
	_EEPRom_WRITE_ALL,
	#ifdef SUPPORT_BLACK_OUT
  _EEPRom_WRITE_PAUSE_SYS_RESUME_INFO,
  _EEPRom_WRITE_CLEAR_PAUSE_INFO,
  #endif
	_HARDWARE_INITIALIZE,
};
// State
enum
{
	_STATE_NONE = 0,
	_STATE_REST,
	_STATE_CHARGE,
	_STATE_DISCHARGE,
	_STATE_BALANCE,
	_STATE_CONTACT,
	_STATE_PULSE,
	_STATE_DCIR,
};

enum
{
	_NOW_STATE_REST = 0,
	_NOW_STATE_CHARGE,
	_NOW_STATE_DISCHARGE,
};

enum
{
	_PULSE_RECTANGLE = 0,
	_PULSE_TRIANGLE,
};

//Rest Mode 2024.09.05 (부사장님 요청사항)
enum
{
	_REST_TOLERANCE_MODE	= 200,
	_REST_ABSOLUTE_MODE		= 250,
	_REST_TOL_ABS_MODE		= 251,
	_REST_NONE_MODE			= 252,

};

enum
{
	_MODE_FORMATION_LAST_STEP = 0,
	_MODE_LOGICAL_REST,
	_MODE_PHYSICAL_REST,
	_MODE_CC,
	_MODE_CCCV,
	_MODE_CPCV,
	_MODE_CRCV,
	_MODE_2POINT_PULSE_CHARGE,
	_MODE_2POINT_PULSE_DISCHARGE,
	_MODE_10POINT_PULSE_CHARGE,
	_MODE_10POINT_PULSE_DISCHARGE,
	_MODE_CHARGE_CONTACT,
	_MODE_DISCHARGE_CONTACT,
	_MODE_10ms_DCIR,
	_MODE_Sec_DCIR,		//LJK 2024.10.14 유부사장님 요청사항	
	_MODE_NONE,
};

enum
{
	_PULSE_MODE_CCCV = 0,
	_PULSE_MODE_CPCV,
	_PULSE_MODE_CRCV,
	_PULSE_MODE_CCCV_DP,
	_PULSE_MODE_CPCV_DP,
	_PULSE_MODE_CCCV_CP,	//LJK 2024.07.29 유부사장님 요청사항
	_PULSE_MODE_CPCV_CP,	//LJK 2024.07.29 유부사장님 요청사항
	_PULSE_MODE_PCCV_DP2,	//bygu 2025.06.16
	_PULSE_MODE_PPCV_DP2,	//bygu 2025.06.16
	_PULSE_MODE_PRCV_CP2,	//bygu 2025.06.16
	_PULSE_MODE_NONE,
};

// Step 현재 및 종료 조건
enum
{
	_STEP_END_CONDITION_TIME=0,
	_STEP_END_CONDITION_CV_TIME,
	_STEP_END_CONDITION_CHARGE_VOLTAGE,
	_STEP_END_CONDITION_DISCHARGE_VOLTAGE,
	_STEP_END_CONDITION_CHARGE_CURRENT,
	_STEP_END_CONDITION_DISCHARGE_CURRENT,
	_STEP_END_CONDITION_CHARGE_CAPACITY,
	_STEP_END_CONDITION_DISCHARGE_CAPACITY,
	_STEP_END_CONDITION_CHARGE_WATTHOUR,
	_STEP_END_CONDITION_DISCHARGE_WATTHOUR,
	_STEP_END_CONDITION_SOC,
	_STEP_END_CONDITION_DOD,
	_STEP_END_CONDITION_CONTACT_RESISTANCE,
	_STEP_END_CONDITION_SOFT_CURRENT_MODE,
	_STEP_END_CONDITION_HIGH_TEMPERATURE,	//20221230 Temp Condition 추가
	_STEP_END_CONDITION_LOW_TEMERATURE,
};

enum
{
	_STEP_STOP = 0,
	_STEP_TESTING,
	_STEP_END_TIME,
	_STEP_END_CV_TIME,
	_STEP_END_CHARGE_VOLTAGE,
	_STEP_END_DISCHARGE_VOLTAGE,	// 5
	_STEP_END_CHARGE_CURRENT,
	_STEP_END_DISCHARGE_CURRENT,	// 7
	_STEP_END_CHARGE_CAPACITY,
	_STEP_END_DISCHARGE_CAPACITY,
	_STEP_END_CHARGE_WATTHOUR,
	_STEP_END_DISCHARGE_WATTHOUR,
	_STEP_END_SOC,
	_STEP_END_DOD,
	_STEP_END_INDEX_MAX,
	_STEP_END_BY_USER,
	_STEP_END_HIGH_TEMPERATURE,	//20230104 추가
	_STEP_END_LOW_TEMPERATURE,
	_TEST_END_CONTACT_OVER_RESISTANCE,
	_FORMATION_END,
	_STEP_END_PAUSE,	//2023.03.20 일시정지 추가
//	_STEP_END_INDEX_MAX	이상부터 Error 종료 조건
	_STEP_TEST_EMPTY,	// Test Step 없음
	_STEP_TESTING_NOW,	// Step Test 중...
	_STEP_ALARM_UVP = 0x60,
	_STEP_ALARM_OVP,
	_STEP_ALARM_UCP,
	_STEP_ALARM_OCP,
	_STEP_ALARM_UWP,
	_STEP_ALARM_OWP,
	_STEP_ALARM_URP,
	_STEP_ALARM_ORP,
	_STEP_ALARM_CHARGE_VOLTAGE_DOWN,
	_STEP_ALARM_DISCHARGE_VOLTAGE_UP,
	_STEP_ALARM_MINUS_VOLTAGE,
	_STEP_ALARM_VOLTAGE_FIX_ERROR,
	_STEP_ALARM_VSENSOR_OPEN,			// 0x6c
	_STEP_ALARM_VOLTAGE_BALANCE_ERROR,
	_STEP_ALARM_VSENSOR_MINUS,
	_STEP_ALARM_VSENSOR_OVER,
	_STEP_ALARM_ISENSOR_VOLTAGE_MINUS,	//0x70
	_STEP_ALARM_ISENSOR_VOLTAGE_OVER,
	_STEP_ALARM_FAN,
	_STEP_ALARM_AIR_TEMP,
	_STEP_ALARM_HEAT_SINK_TEMP,
	_STEP_ALARM_MY_CHANNEL,
	_STEP_ALARM_OTHER_CHANNEL,
	_STEP_ALARM_VOLTAGE_FAST_DOWN,
	_STEP_ALARM_VOLTAGE_FAST_UP,
	_STEP_DAC_ERROR,
	_STEP_ALARM_REST_OVP,			//LJK 2023.06.22 유부사장 추가사항
	_STEP_ALARM_REST_UVP,			//LJK 2023.06.22 유부사장 추가사항	
	_STEP_ALARM_CELL_TEMP_ERROR,	//LJK 2024.07.01 
	_STEP_ALARM_REVERSE_CELL,       //20250908 역전압 알람 추가
	#ifdef SUPPORT_BLACK_OUT
	_STEP_ALARM_BLACKOUT,          //20251212 swcho blackout 추가
	_STEP_ALARM_EMG,               //20251212 swcho EMG 추가
	_STEP_ALARM_SMOKE,               //20251212 swcho EMG 추가
	#endif
	_STEP_END_CONDITION_NONE,
};

// Mask Bits
enum
{
	_STEP_ALARM_MASK_UVP = 0,
	_STEP_ALARM_MASK_OVP,
	_STEP_ALARM_MASK_UCP,
	_STEP_ALARM_MASK_OCP,
	_STEP_ALARM_MASK_UWP,
	_STEP_ALARM_MASK_OWP,
	_STEP_ALARM_MASK_URP,
	_STEP_ALARM_MASK_ORP,
	_STEP_ALARM_MASK_CHARGE_VOLTAGE_DOWN,
	_STEP_ALARM_MASK_DISCHARGE_VOLTAGE_UP,
	_STEP_ALARM_MASK_MINUS_VOLTAGE,
	_STEP_ALARM_MASK_VOLTAGE_FIX_ERROR,
	_STEP_ALARM_MASK_VSENSOR_OPEN,
	_STEP_ALARM_MASK_VOLTAGE_BALANCE_ERROR,
	_STEP_ALARM_MASK_VSENSOR_MINUS,
	_STEP_ALARM_MASK_VSENSOR_OVER,
	_STEP_ALARM_MASK_ISENSOR_VOLTAGE_MINUS,
	_STEP_ALARM_MASK_ISENSOR_VOLTAGE_OVER,
	_STEP_ALARM_MASK_FAN,
	_STEP_ALARM_MASK_AIR_TEMP,
	_STEP_ALARM_MASK_HEAT_SINK_TEMP,
	_STEP_ALARM_MASK_MY_CHANNEL,
	_STEP_ALARM_MASK_OTHER_CHANNEL,
	_STEP_ALARM_MASK_VOLTAGE_FAST_DOWN,
	_STEP_ALARM_MASK_VOLTAGE_FAST_UP,
	_STEP_ALARM_MASK_REST_OVP,			//LJK 2023.06.22 유부사장 추가사항
	_STEP_ALARM_MASK_REST_UVP,			//LJK 2023.06.22 유부사장 추가사항	
	_STEP_ALARM_MASK_CELL_TEMP_ERROR,	//LJK 2024.07.01
	_STEP_ALARM_MASK_REVERSE_CELL,	    //20250908 역전압 알람 추가
	#ifdef SUPPORT_BLACK_OUT
	_STEP_ALARM_MASK_BLACKOUT,          //20251212 swcho blackout 추가
	_STEP_ALARM_MASK_EMG,
	_STEP_ALARM_MASK_SMOKE,
	#endif
};

#ifdef SUPPORT_BLACK_OUT
enum 
{ 
  _DELAY_NONE = 0,
  _DELAY_PAUSE_ON,
  _DELAY_RESUME_ON,
  _DELAY_CLEAR,
};
#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Machine Alarm Bit On...
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
enum SystemErrorFlag
{
	SystemErrorFlagPcdaPcCommunicationRxBufferOver=0,
	SystemErrorFlagPcCommunicationParityEtc,
	SystemErrorFlagPcCommunicationNoneStx,
	SystemErrorFlagPcCommunicationSumCheck,
	SystemErrorFlagPcCommunicationRecvSize,
	SystemErrorFlagPcCommunicationPdcaTx,
	SystemErrorFlagPcCommunicationPdcaRx,
	SystemErrorFlagPcCommunicationTxQueueFull,
	SystemErrorFlagPcCommunicationTxQueueSizeOver,
	SystemErrorFlagPcCommunicationSumCheck2,
	SystemErrorFlagPcCommunicationReceiveContError,
	SystemErrorFlagPcCommunicationCommandSize,
	SystemErrorFlagAdcTimeOver,
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Communication Command
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
enum PcCommunicationCommandList
{
	Pc_Comm_GetRunStopAllChannel = 0,
	Pc_Comm_RunControlBitsChannel,
	Pc_Comm_StopControlBitsChannel,
	Pc_Comm_ResetStepSequence,
	Pc_Comm_SetOneStepSequence,
	Pc_Comm_GetStepSum32,
	
	Pc_Comm_Get_EEPROM_Pointer200,			
	Pc_Comm_Set_EEPROM_Pointer200,	
	Pc_Comm_GetLastOneSecondCondition,
	Pc_Comm_GetDataRecordOne,
	Pc_Comm_DeleteDataRecordOne,
	
	Pc_Comm_DeleteDataRecordAll,			
	Pc_Comm_MaskAlarmCondition,
	Pc_Comm_EEPRomControl,
	Pc_Comm_SetGetGpIo,
	Pc_Comm_SetGpIo,
	
	Pc_Comm_GetGpIo,						
	Pc_Comm_SetHardWareInitialize,
	Pc_Comm_GetSystemInformationData,
	Pc_Comm_SetOneStep,
	Pc_Comm_GetOneStep,
	
	Pc_Comm_SetCalibrationData,
	Pc_Comm_GetCalibrationData,
	Pc_Comm_SetCalibrationEEPRom,
	Pc_Comm_SetChargePower,
	Pc_Comm_GetChargePowerFet,
	
	Pc_Comm_SetGate,	
	Pc_Comm_SetDac,
	Pc_Comm_StartCalibrationMode,
	Pc_Comm_GetCalibrationAdc,
	Pc_Comm_AllChannelReset,
	
	Pc_Comm_GetDacValue,
	Pc_Comm_GetVIV2,
	Pc_Comm_SetV2ModeCurrent,
	Pc_Comm_SetV2ModeVoltage,
	Pc_Comm_ClearLoopCounterAll,
	
	Pc_Comm_SetStartMode,
	Pc_Comm_GetStepEndData,
	Pc_Comm_InitSystmeInformationData,		
	PC_Comm_SendTemperature,			// 20221230 Temperature 모니용
	Pc_Comm_PauseResume,				// 20230320 Pause, Resume LJK
	
	Pc_Comm_GetAirTemp,					// 2023.05.12	AirTemp
	Pc_Comm_GetHeatSink,				// 2023.05.12	HeatSink
	Pc_Comm_SetCellTemp,				// 2024.07.01	Cell Temp Limit
	
	PC_Comm_SetVoltage,					// 2025.04.14 Alarm Test
	PC_Comm_SetCurrent,					// 2025.04.14 Alarm Test
	PC_Comm_SetResistance,				// 2025.04.23 Alarm Test
	PC_Comm_SetWatt,					// 2025.04.23 Alarm Test
	PC_Comm_SetHeatSink,				// 2025.04.23 Alarm Test
	PC_Comm_SetAirTemp,					// 2025.04.23 Alarm Test

	#ifdef SUPPORT_PROTECTION_CONDITION
	PC_Comm_SetProtection,              // 20250805_jschoi
	#endif
  
  #ifdef SUPPORT_BLACK_OUT
  Pc_Comm_SetResumeStepSequence,
  #endif
};

enum _SystemParameterIndexList
{
	_FW_Version = 0,
	_SystemErrorFlag,
	_SystemErrorFlagCounter,
	_SdramFail,
	_SdramErrorAddress,
	_SdramErrorData,
	_DacWriteFatalError,
	_DacWriteRetryError,
	_AdcReadCompleteError,
	_AdcReadCompleteErrorCode,
	_CPUResetCause,				//2023.04.13 LJK CPU Reset 정보  pm_412.h 참조, main.c : reset_cause_get_causes()
	_SystemParameterIndexLast,
};


typedef struct
{
	float fChargeVoltage;
	float fDisChargeVoltage;
	float fChargeCurrent;
	float fDisChargeCurrent;
	double fChargeCapacity;		// DCIR시는 변화된 전압
	double fDisChargeCapacity;	// DCIR시는 적분된 전류
	double fChargeWattHour;
	double fDisChargeWattHour;
} _RESULT_DATA;

typedef struct
{
	unsigned int uiCalibrationDate[_MAX_CHANNEL];
	float fContactSpec[2][_MAX_CHANNEL];			// [0:Charge,1:Discharge][0:CH1~3:CH4]
	float fContactCalibration[2][_MAX_CHANNEL];	// [0:Charge,1:Discharge][0:CH1~3:CH4]
	S16 s16Dac_ChargePower[_MAX_CHANNEL/2][CALIBRATION_MAX_STEP];
	unsigned char ucMaxStep[CALIBRATION_MAX_RANGE];
	int iAdc[CALIBRATION_MAX_RANGE][CALIBRATION_MAX_STEP];
	S16 s16Dac[CALIBRATION_MAX_RANGE][CALIBRATION_MAX_STEP];
	float fReferance[CALIBRATION_MAX_RANGE][CALIBRATION_MAX_STEP];
	char cCalibrator[_MAX_CHANNEL][17];
} _EEPROM_DATA;

/*
CALIBRATION_MAX_RANGE
	0	Ch1 Voltage
	1	Ch2 Voltage
	2	Ch3 Voltage
	3	Ch4 Voltage
	4	Ch1 Discharge Current
	5	Ch1 Charge Current
	6	Ch2 Discharge Current
	7	Ch2 Charge Current
	8	Ch3 Discharge Current
	9	Ch3 Charge Current
	10	Ch4 Discharge Current
	11	Ch4 Charge Current
	12	Ch1 Voltage2
	13	Ch2 Voltage2
	14	Ch3 Voltage2
	15	Ch4 Voltage2
*/

typedef struct
{
	unsigned char ucLoopIndex;
	unsigned short usLoopCountMax;
	unsigned short usLoopCountNow;
	unsigned char ucState;
	unsigned char ucMode;
	float fSettingVoltage;
	float fSettingCurrent;	// Pulse시 Discharge Voltage
	U32 u32StepEndConditionsEnable;
	U8 ucSocDodIndex;
	unsigned int uiTestSecond32;
	unsigned int uiCV_msSecond32;
	unsigned int uiDataRecordSecond32;
	_RESULT_DATA StepEndCondition;
	short shStepEndTemperatureHigh;		//20230104 추가
	short shStepEndTemperatureLow;
	unsigned char ucPulseMax;	// Pulse Point #
	unsigned char ucPulseMode;	// _PULSE_MODE_CCCV...
	unsigned char ucPulseKind[_MAX_PULSE_STEP];	// Rectangle/Triangle
	U16 u16PulseTime1ms[_MAX_PULSE_STEP];	// 1ms Pulse 유지 시간들
	float fPulseCurrent[_MAX_PULSE_STEP];		// DCIR의 경우 0=Start 1초동안 전류, Pulse, +값은 Chargae, -값은 DisCharge
	float fPulseChargeUp1Sec;		// 2Point Pulse시 1초당 Charge Current Up Delta Up
	float fPulseChargeCurrentMax;	// Charge Delta Up 절대치 최대
	float fPulseDisChargeUp1Sec;	// 2Point Pulse시 1초당 DisCharge Current Up Delta Down
	float fPulseDisChargeCurrentMax;// DisCharge Delta Up 절대치 최대
} _FORMATIONM_STEP_SEQUENCE;

typedef struct
{
	_FORMATIONM_STEP_SEQUENCE stSequence[_MAX_CHANNEL][_MAX_SEQUENCE_STEP];
} _FORMATIONM_STEP_SEQUENCE_STRUCTURE;

typedef struct
{
	unsigned short usLoopCountNow;
	unsigned char ucEndCondition;		// 종료된 조건
	unsigned int uiTotalTestmsSecond32;	// 총시간
	unsigned int uiCV_msSecond32;		// CV 시간
	_RESULT_DATA StepEndCondition;
} _FORMATIONM_STEP_END_DATA;

typedef struct
{
	_FORMATIONM_STEP_END_DATA stStepEndData[_MAX_CHANNEL][_MAX_SEQUENCE_STEP];
} _FORMATIONM_STEP_END_DATA_STRUCTURE;

/*
typedef struct
{
	unsigned char ucStepNow;
	unsigned int uiNowmsSecond32;		// 현재시간
	unsigned int uiCV_msSecond32;		// CV 시간
	_RESULT_DATA StepEndCondition;
} _FORMATIONM_STEP_RECORD_DATA;
*/

typedef struct
{
	unsigned char ucRcodrdType;		// Record : 1, Step End : 2
	unsigned short usLoopCountNow;	// 반복 카운트
	unsigned char ucEndCondition;	// 종료된 조건
	unsigned char ucStepNow;		// 스탭 번호
	unsigned int uiNowmsSecond32;	// REC 현재시간,  END 총시간
	unsigned int uiCV_msSecond32;	// CV 시간
	_RESULT_DATA StepEndCondition;
} _FORMATIONM_STEP_RECORD_DATA;



#define		_FORMATIONM_STEP_RECORD_MAX		(60*60*18)
typedef struct
{
	_FORMATIONM_STEP_RECORD_DATA stRecord[_MAX_CHANNEL][_FORMATIONM_STEP_RECORD_MAX];
} _FORMATIONM_STEP_RECORD_DATA_STRUCTURE;

#define		MAX_1Sec_BUFFER		1000	// 건들면 안됨

typedef struct
{
	int iVoltage[_MAX_CHANNEL][MAX_1Sec_BUFFER];
	int iCurrent[_MAX_CHANNEL][MAX_1Sec_BUFFER];
	int iVoltage2[_MAX_CHANNEL][MAX_1Sec_BUFFER];
} _1SEC_BUFFER_STRUCTURE;

typedef struct
{
	float fDCIR_Current[_MAX_CHANNEL][MAX_DCIR_SEC];
	unsigned short usDCIR_Count[_MAX_CHANNEL];
} _DCIR_SEC_BUFFER_STRUCTURE;


// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
typedef struct
{
	unsigned char ucPcCommRing_TxBuffer[512];
	unsigned char ucPcCommLastIndex;
	unsigned char ucPcCommRingIndex;
	unsigned char ucPcCommReturnValue;
} CPU_TX_QUEUE_BUFFER;

typedef struct
{
	unsigned char ucKpuCommand_TxBuffer[512];
} CPU_COMMAND_QUEUE_BUFFER;


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Avr32 Flash User Page Area ==> Write
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
enum RetryCounterValueList					// OK
{
	RTY_UVP = 0,
	RTY_OVP,
	RTY_UCP,
	RTY_OCP,
	RTY_UWP,
	RTY_OWP,
	RTY_URP,
	RTY_ORP,
	RTY_CHARGE_VOLTATGE_DOWN_ERROR,
	RTY_DISCHARGE_VOLTATGE_UP_ERROR,
	RTY_VOLTAGE_BALANCE_ERROR,
	RTY_VSENSOR_OPEN_ERROR,
	RTY_VOLTAGE_FIX_ERROR,
	RTY_VOLTAGE_MINUS_ERROR,
	RTY_FAN,
	RTY_AIR_TEMP,
	RTY_HEAT_SINK_TEMP,
	RTY_COMMUNICATION_ERROR,
	RTY_MAIN_POWER_ALARM,
	RTY_CELL_TEMP_ERROR,
	RTY_VOLTAGE_FAST_DOWN,
	RTY_VOLTAGE_FAST_UP,
	RTY_REST_OVP,
	RTY_REST_UVP,	
	#ifdef SUPPORT_BLACK_OUT
  RTY_BLACKOUT,
  RTY_EMG,
  RTY_SMOKE, 
  #endif
	RTY_MAX_CNT,							// MAX CNT
};

typedef struct 
{
	unsigned int  uiMachineInfo[16];
} MCU_A_TEST_INFORMATION_DATA;



//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Safety Condition
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef SUPPORT_PROTECTION_CONDITION
typedef struct
{
    float fVoltLowerOffset;
    float fVoltUpperOffset;
    float fCurrLowerOffset;
    float fCurrUpperOffset;
    float fPowerLowerOffset;
    float fPowerUpperOffset;
    float fResistLowerOffset;
    float fResistUpperOffset;
    float fReverseCellVolt;
    float fAirTemp;
    float fHeatSinkTemp;
    float fCellTemp;
    
} _PROTECTION_CONDITION_TYPE;
#endif

#ifdef SUPPORT_BLACK_OUT
typedef struct
{
  unsigned char ucIsPauseDataValid[_MAX_CHANNEL];
  unsigned char ucIsPauseDataWritten[_MAX_CHANNEL];

  unsigned int  uiStepTimeNow[_MAX_CHANNEL];
  unsigned int uiStepCV_TimeNow[_MAX_CHANNEL];
  
  unsigned char ucStartStepIndex[_MAX_CHANNEL];
  unsigned char ucPauseStatus[_MAX_CHANNEL];
  
  double dblChargeCapacity[_MAX_CHANNEL];
  double dblDisChargeCapacity[_MAX_CHANNEL];
  double dblChargeWattHour[_MAX_CHANNEL];
  double dblDisChargeWattHour[_MAX_CHANNEL];

  //PULSE
  unsigned int uiPauseStepTimeNow[_MAX_CHANNEL];
  unsigned short u16PausePulseTime1msNow[_MAX_CHANNEL];
  unsigned short u16PulseTime1msNow[_MAX_CHANNEL];
  float fPulseRefCurrent[_MAX_CHANNEL];
  float fPulseCurrentPre[_MAX_CHANNEL];
  unsigned char ucPulseIndex[_MAX_CHANNEL];

  unsigned char checkSum;
} PAUSE_INFO_DATA;
#endif


// Pin Control
#define PDA_HIGH		(*(U32*)0xFFFF2054)
#define PDA_LOW			(*(U32*)0xFFFF2058)
#define PDB_HIGH		(*(U32*)0xFFFF2254)
#define PDB_LOW			(*(U32*)0xFFFF2258)
#define PDC_HIGH		(*(U32*)0xFFFF2454)
#define PDC_LOW			(*(U32*)0xFFFF2458)
#define PDD_HIGH		(*(U32*)0xFFFF2654)
#define PDD_LOW			(*(U32*)0xFFFF2658)

#define PDA_PIN_VALUE	(*(U32*)0xFFFF2060)
#define PDB_PIN_VALUE	(*(U32*)0xFFFF2264)
#define PDC_PIN_VALUE	(*(U32*)0xFFFF2468)
#define PDD_PIN_VALUE	(*(U32*)0xFFFF2660)

// AVR : GPIO Pin Map
#define AVR_32_CLK2							(0xA0000000|AVR32_PIN_PB30)
#define _PORT_B_AVR_32_CLK2					(1<<30)

#define _CH1_MC								(0xA0000000|AVR32_PIN_PB31)
#define		_PORT_B_CH1_MC						(1<<31)
#define _CH1_MC_ON							PDB_LOW = _PORT_B_CH1_MC
#define _CH1_MC_OFF							PDB_HIGH = _PORT_B_CH1_MC

#define _CH2_MC								(0xA0000000|AVR32_PIN_PB03)
#define		_PORT_B_CH2_MC						(1<<03)
#define _CH2_MC_ON							PDB_LOW = _PORT_B_CH2_MC
#define _CH2_MC_OFF							PDB_HIGH = _PORT_B_CH2_MC

#define _CH3_MC								(0xA0000000|AVR32_PIN_PC02)
#define		_PORT_C_CH3_MC						(1<<02)
#define _CH3_MC_ON							PDC_LOW = _PORT_C_CH3_MC
#define _CH3_MC_OFF							PDC_HIGH = _PORT_C_CH3_MC

#define _CH4_MC								(0xA0000000|AVR32_PIN_PC03)
#define		_PORT_C_CH4_MC						(1<<03)
#define _CH4_MC_ON							PDC_LOW = _PORT_C_CH4_MC
#define _CH4_MC_OFF							PDC_HIGH = _PORT_C_CH4_MC


#define _ALARM_LED							(0xA0000000|AVR32_PIN_PB01)
#define		_PORT_B_ALARM_LED					(1<<01)
#define _RUN_LED							(0xA0000000|AVR32_PIN_PB02)
#define		_PORT_B_RUN_LED						(1<<02)

#define _FAN_OFF_H							(0xA0000000|AVR32_PIN_PC01)
#define		_PORT_C_FAN_OFF_H					(1<<01)

#define	_BOARD_AIR_TEMPERATURE				(0xA0000000|AVR32_PIN_PA04)
#define		_PORT_A_BOARD_AIR_TEMPERATURE		(1<<04)
#define	_DAC_TEMPERATURE					(0xA0000000|AVR32_PIN_PA05)
#define		_PORT_A_DAC_TEMPERATURE				(1<<05)
#define	_HEAT_SINK_TEMPERATURE				(0xA0000000|AVR32_PIN_PA06)
#define		_PORT_A_HEAT_SINK_TEMPERATURE		(1<<06)

#define	_HEAT_SINK_TEMPERATURE_A0			(0xA0000000|AVR32_PIN_PA07)
#define		_PORT_A_HEAT_SINK_TEMPERATURE_A0	(1<<7)
#define	_HEAT_SINK_TEMPERATURE_A1			(0xA0000000|AVR32_PIN_PA08)
#define		_PORT_A_HEAT_SINK_TEMPERATURE_A1	(1<<8)
#define	_HEAT_SINK_TEMPERATURE_A2			(0xA0000000|AVR32_PIN_PA09)
#define		_PORT_A_HEAT_SINK_TEMPERATURE_A2	(1<<9)


#define _CH1_DICHARGE0_CHARGE1				(0xA0000000|AVR32_PIN_PA11)
#define		_PORT_A_CH1_DICHARGE0_CHARGE1		(1<<11)
#define _CH1_MODE_CHARGE					PDA_HIGH = _PORT_A_CH1_DICHARGE0_CHARGE1
#define _CH1_MODE_DISCHARGE					PDA_LOW = _PORT_A_CH1_DICHARGE0_CHARGE1

#define _CH1_DISCHARGE_ENABLE				(0xA0000000|AVR32_PIN_PA12)
#define		_PORT_A_CH1_DISCHARGE_ENABLE		(1<<12)
#define _CH1_DISCHARGE_ENABLE_ON			PDA_HIGH = _PORT_A_CH1_DISCHARGE_ENABLE
#define _CH1_DISCHARGE_ENABLE_OFF			PDA_LOW = _PORT_A_CH1_DISCHARGE_ENABLE

#define _CH1_CHARGE_ENABLE					(0xA0000000|AVR32_PIN_PA13)
#define		_PORT_A_CH1_CHARGE_ENABLE			(1<<13)
#define _CH1_CHARGE_ENABLE_ON			PDA_HIGH = _PORT_A_CH1_CHARGE_ENABLE
#define _CH1_CHARGE_ENABLE_OFF			PDA_LOW = _PORT_A_CH1_CHARGE_ENABLE


#define _CH2_DICHARGE0_CHARGE1				(0xA0000000|AVR32_PIN_PA14)
#define		_PORT_A_CH2_DICHARGE0_CHARGE1		(1<<14)
#define _CH2_MODE_CHARGE					PDA_HIGH = _PORT_A_CH2_DICHARGE0_CHARGE1
#define _CH2_MODE_DISCHARGE					PDA_LOW = _PORT_A_CH2_DICHARGE0_CHARGE1

#define _CH2_DISCHARGE_ENABLE				(0xA0000000|AVR32_PIN_PA15)
#define		_PORT_A_CH2_DISCHARGE_ENABLE		(1<<15)
#define _CH2_DISCHARGE_ENABLE_ON			PDA_HIGH = _PORT_A_CH2_DISCHARGE_ENABLE
#define _CH2_DISCHARGE_ENABLE_OFF			PDA_LOW = _PORT_A_CH2_DISCHARGE_ENABLE

#define _CH2_CHARGE_ENABLE					(0xA0000000|AVR32_PIN_PA16)
#define		_PORT_A_CH2_CHARGE_ENABLE			(1<<16)
#define _CH2_CHARGE_ENABLE_ON			PDA_HIGH = _PORT_A_CH2_CHARGE_ENABLE
#define _CH2_CHARGE_ENABLE_OFF			PDA_LOW = _PORT_A_CH2_CHARGE_ENABLE


#define _CH3_DICHARGE0_CHARGE1				(0xA0000000|AVR32_PIN_PA19)
#define		_PORT_A_CH3_DICHARGE0_CHARGE1		(1<<19)
#define _CH3_MODE_CHARGE					PDA_HIGH = _PORT_A_CH3_DICHARGE0_CHARGE1
#define _CH3_MODE_DISCHARGE					PDA_LOW = _PORT_A_CH3_DICHARGE0_CHARGE1

#define _CH3_DISCHARGE_ENABLE				(0xA0000000|AVR32_PIN_PA20)
#define		_PORT_A_CH3_DISCHARGE_ENABLE		(1<<20)
#define _CH3_DISCHARGE_ENABLE_ON			PDA_HIGH = _PORT_A_CH3_DISCHARGE_ENABLE
#define _CH3_DISCHARGE_ENABLE_OFF			PDA_LOW = _PORT_A_CH3_DISCHARGE_ENABLE

#define _CH3_CHARGE_ENABLE					(0xA0000000|AVR32_PIN_PA21)
#define		_PORT_A_CH3_CHARGE_ENABLE			(1<<21)
#define _CH3_CHARGE_ENABLE_ON			PDA_HIGH = _PORT_A_CH3_CHARGE_ENABLE
#define _CH3_CHARGE_ENABLE_OFF			PDA_LOW = _PORT_A_CH3_CHARGE_ENABLE


#define _CH4_DICHARGE0_CHARGE1				(0xA0000000|AVR32_PIN_PA22)
#define		_PORT_A_CH4_DICHARGE0_CHARGE1		(1<<22)
#define _CH4_MODE_CHARGE					PDA_HIGH = _PORT_A_CH4_DICHARGE0_CHARGE1
#define _CH4_MODE_DISCHARGE					PDA_LOW = _PORT_A_CH4_DICHARGE0_CHARGE1

#define _CH4_DISCHARGE_ENABLE				(0xA0000000|AVR32_PIN_PA23)
#define		_PORT_A_CH4_DISCHARGE_ENABLE		(1<<23)
#define _CH4_DISCHARGE_ENABLE_ON			PDA_HIGH = _PORT_A_CH4_DISCHARGE_ENABLE
#define _CH4_DISCHARGE_ENABLE_OFF			PDA_LOW = _PORT_A_CH4_DISCHARGE_ENABLE

#define _CH4_CHARGE_ENABLE					(0xA0000000|AVR32_PIN_PA24)
#define		_PORT_A_CH4_CHARGE_ENABLE			(1<<24)
#define _CH4_CHARGE_ENABLE_ON			PDA_HIGH = _PORT_A_CH4_CHARGE_ENABLE
#define _CH4_CHARGE_ENABLE_OFF			PDA_LOW = _PORT_A_CH4_CHARGE_ENABLE


#define _FAN_ERROR_INPUT_					(0xA0000000|AVR32_PIN_PA25)
#define		_PORT_A_FAN_ERROR_INPUT_			(1<<25)

#define _CH_ID_00							(0xA0000000|AVR32_PIN_PB18)
#define		_PORT_B_CH_ID_00					(1<<18)
#define _CH_ID_01							(0xA0000000|AVR32_PIN_PB19)
#define		_PORT_B_CH_ID_01					(1<<19)
#define _CH_ID_02							(0xA0000000|AVR32_PIN_PB20)
#define		_PORT_B_CH_ID_02					(1<<20)
#define _CH_ID_03							(0xA0000000|AVR32_PIN_PB21)
#define		_PORT_B_CH_ID_03					(1<<21)
#define _CH_ID_04							(0xA0000000|AVR32_PIN_PB25)
#define		_PORT_B_CH_ID_04					(1<<25)
#define _CH_ID_05							(0xA0000000|AVR32_PIN_PB26)
#define		_PORT_B_CH_ID_05					(1<<26)

#define _CH1_CHARGE_PULLUP					(0xA0000000|AVR32_PIN_PB27)	//2023.04.04 LJK
#define _CH2_CHARGE_PULLUP					(0xA0000000|AVR32_PIN_PB28)	//2023.04.04 LJK
#define _CH3_CHARGE_PULLUP					(0xA0000000|AVR32_PIN_PB29)	//2023.04.04 LJK
#define _CH4_CHARGE_PULLUP					(0xA0000000|AVR32_PIN_PC00)	//2023.04.04 LJK
/*
#define _CH1_LED							(0xA0000000|AVR32_PIN_PB27)
#define		_PORT_B_CH1_LED						(1<<27)
#define _CH2_LED							(0xA0000000|AVR32_PIN_PB28)
#define		_PORT_B_CH2_LED						(1<<28)
#define _CH3_LED							(0xA0000000|AVR32_PIN_PB29)
#define		_PORT_B_CH3_LED						(1<<29)
#define _CH4_LED							(0xA0000000|AVR32_PIN_PC00)
#define		_PORT_C_CH4_LED						(1<<00)
*/

#define _GATE1								(0xA0000000|AVR32_PIN_PD04)
#define		_PORT_D_GATE1						(1<<04)
#define _GATE2								(0xA0000000|AVR32_PIN_PD16)
#define		_PORT_D_GATE2						(1<<16)
#define _GATE3								(0xA0000000|AVR32_PIN_PD19)
#define		_PORT_D_GATE3						(1<<19)
#define _GATE4								(0xA0000000|AVR32_PIN_PD22)
#define		_PORT_D_GATE4						(1<<22)
#define _GATE5								(0xA0000000|AVR32_PIN_PD23)
#define		_PORT_D_GATE5						(1<<23)
#define _GATE6								(0xA0000000|AVR32_PIN_PC10)
#define		_PORT_C_GATE6						(1<<10)
#define _GATE7								(0xA0000000|AVR32_PIN_PC12)
#define		_PORT_C_GATE7						(1<<12)

#define RS_485_RXD_2						(0xA0000000|AVR32_PIN_PB16)
#define RS_485_TXD_2						(0xA0000000|AVR32_PIN_PB17)
#define RS_485_RTS_2						(0xA0000000|AVR32_PIN_PB14)


#define _DEBUG0						(0xA0000000|AVR32_PIN_PA29)
#define		_PORT_A_DEBUG0				(1<<29)
#define DEBUG_TP0_H					PDA_HIGH = _PORT_A_DEBUG0
#define DEBUG_TP0_L					PDA_LOW = _PORT_A_DEBUG0
#define _DEBUG1						(0xA0000000|AVR32_PIN_PA26)
#define		_PORT_A_DEBUG1				(1<<26)
#define DEBUG_TP1_H					PDA_HIGH = _PORT_A_DEBUG1
#define DEBUG_TP1_L					PDA_LOW = _PORT_A_DEBUG1
#define _DEBUG2						(0xA0000000|AVR32_PIN_PA27)
#define		_PORT_A_DEBUG2				(1<<27)
#define DEBUG_TP2_H					PDA_HIGH = _PORT_A_DEBUG2
#define DEBUG_TP2_L					PDA_LOW = _PORT_A_DEBUG2


#define  CAPTAIN_TP_H			DEBUG_TP0_H
#define  CAPTAIN_TP_L			DEBUG_TP0_L

#ifdef SUPPORT_BLACK_OUT
#define _EMG_CHECK_ ( gpio_get_pin_value(_DEBUG0&0xFFF))
#define _BLACKOUT_CHECK_ ( gpio_get_pin_value(_DEBUG1&0xFFF))
#define _SMOKE_CHECK_ ( gpio_get_pin_value(_DEBUG2&0xFFF))

#define  ADC_TP_H				0
#define  ADC_TP_L				0
#define  ADC_ERROR_TP_H			0
#define  ADC_ERROR_TP_L			0
#else
#define  ADC_TP_H				DEBUG_TP1_H
#define  ADC_TP_L				DEBUG_TP1_L
#define  ADC_ERROR_TP_H			DEBUG_TP2_H
#define  ADC_ERROR_TP_L			DEBUG_TP2_L
#endif

#define ADD_SDRAM_CHECK_ENALBE	(0xA0003027)
#define ADD_SDRAM_CHECK_DISABLE (0xA0003028)
#define ADD_SDRAM_CHECK_STATUS	(0xA0003029)
#define ADD_FAN_CONTROL_DIRECT	(0xA0003030)
#define ADD_FAN_OFF_DELAY		(0xA0003031)

#define _DAC_SDI					(0xA0000000|AVR32_PIN_PB04)
#define		_PORT_B_DAC_SDI				(1<<04)
#define _DAC_SDO					(0xA0000000|AVR32_PIN_PB05)
#define		_PORT_B_DAC_SDO				(1<<05)
#define _DAC_SCLK					(0xA0000000|AVR32_PIN_PB06)
#define		_PORT_B_DAC_SCLK			(1<<06)
#define _DAC_SYNC_					(0xA0000000|AVR32_PIN_PB07)
#define		_PORT_B_DAC_SYNC_			(1<<07)
#define _DAC_LDAC					(0xA0000000|AVR32_PIN_PB08)
#define		_PORT_B_DAC_LDAC			(1<<8)
#define _DAC_RESET_					(0xA0000000|AVR32_PIN_PC04)
#define		_PORT_C_DAC_RESET_			(1<<04)

#define _ADC_DOUTB					(0xA0000000|AVR32_PIN_PB09)
#define		_PORT_B_ADC_DOUTB		(1<<9)

#define _ADC_DOUTA					(0xA0000000|AVR32_PIN_PB11)
#define		_PORT_B_ADC_DOUTA			(1<<11)
#define _ADC_SCK					(0xA0000000|AVR32_PIN_PB12)
#define		_PORT_B_ADC_SCK				(1<<12)
#define _ADC_CS_					(0xA0000000|AVR32_PIN_PB13)
#define		_PORT_B_ADC_CS_				(1<<13)
#define _ADC_FIRST_DATA				(0xA0000000|AVR32_PIN_PB15)
#define		_PORT_B_ADC_FIRST_DATA		(1<<15)
#define _ADC_CONVST					(0xA0000000|AVR32_PIN_PA10)
#define		_PORT_A_ADC_CONVST			(1<<10)
#define _ADC_RESET					(0xA0000000|AVR32_PIN_PD30)
#define		_PORT_D_ADC_RESET			(1<<30)
#define _ADC_BUSY					(0xA0000000|AVR32_PIN_PA29)
#define		_PORT_A_ADC_BUSY			(1<<29)

#define _POWER_DAC_SDI				(0xA0000000|AVR32_PIN_PC06)
#define		_PORT_C_POWER_DAC_SDI		(1<<06)
#define _POWER_DAC_SCK				(0xA0000000|AVR32_PIN_PC07)
#define		_PORT_C_POWER_DAC_SCK		(1<<07)
#define _POWER_DAC_SDO				(0xA0000000|AVR32_PIN_PC08)
#define		_PORT_C_POWER_DAC_SDO		(1<<08)
#define _POWER_DAC_LAT				(0xA0000000|AVR32_PIN_PB23)
#define		_PORT_B_POWER_DAC_LAT		(1<<23)
#define _POWER_DAC_CS_				(0xA0000000|AVR32_PIN_PB22)
#define		_PORT_B_POWER_DAC_CS_		(1<<22)

#define _EEPROM_WP					(0xA0000000|AVR32_PIN_PC05)
#define		_PORT_C_EEPROM_WP			(1<<05)
#define _EEPROM_SCL256				(0xA0000000|AVR32_PIN_PD29)
#define		_PORT_D_EEPROM_SCL256		(1<<29)
#define _EEPROM_SDA256				(0xA0000000|AVR32_PIN_PC13)
#define		_PORT_C_EEPROM_SDA256		(1<<13)

#define	_LDAC_ENABLE					{PDB_LOW = _PORT_B_DAC_LDAC; PDB_LOW = _PORT_B_DAC_LDAC; PDB_HIGH = _PORT_B_DAC_LDAC;}
#define	_DAC_MODE_WRITE					0x3

#define _DAC_RESET_ENABLE			{ PDC_LOW = _PORT_C_DAC_RESET_; PDC_LOW = _PORT_C_DAC_RESET_; PDC_LOW = _PORT_C_DAC_RESET_; PDC_HIGH = _PORT_C_DAC_RESET_; }

#define _ADC_RESET_ENABLE			{ PDD_HIGH = _PORT_D_ADC_RESET; PDD_HIGH = _PORT_D_ADC_RESET; PDD_HIGH = _PORT_D_ADC_RESET; PDD_HIGH = _PORT_D_ADC_RESET; PDD_LOW = _PORT_D_ADC_RESET; }

#define _CH1_CH2_POWER_REF			(0xA0002008)
#define _CH3_CH4_POWER_REF			(0xA0002009)

#define _CH1_ADC_VOLTAGE_READ		(0xA000200a)
#define _CH1_ADC_CURRENT_READ		(0xA000200b)

#define _CH1_VOLTAGE2_MODE			(0xA0000000|AVR32_PIN_PC09)
#define _CH2_VOLTAGE2_MODE			(0xA0000000|AVR32_PIN_PD25)	//2023.04.04 LJK
#define _CH3_VOLTAGE2_MODE			(0xA0000000|AVR32_PIN_PD26) //2023.04.04 LJK
#define _CH4_VOLTAGE2_MODE			(0xA0000000|AVR32_PIN_PA28)	//2023.04.04 LJK

#define	I_TR_Soft0_Fast1			(0xA0000000|AVR32_PIN_PC11)
#define		_PORT_C_I_TR_Soft0_Fast1	(1<<11)
#define I_TR_Soft0_Fast1_OFF	PDC_LOW	= _PORT_C_I_TR_Soft0_Fast1
#define I_TR_Soft0_Fast1_ON		PDC_HIGH =_PORT_C_I_TR_Soft0_Fast1	//220517 smy
#pragma pack()
#endif /* _COMMUNICATIONSTRUCTURE_H_ */