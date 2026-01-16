/*
 * GlobalData.c
 *
 * Created: 2021-11-26 AM 00:05:00
 *  Author: 4N Corporation, bgyu
 */ 


#include "KnC_IncludeFile.h"

struct spi_device DAC_SPI_DEVICE = {
	//! Board specific select id
	.id = DAC_SPI_NPCS
};

struct spi_device ADC_SPI_DEVICE = {
	//! Board specific select id
	.id = ADC_SPI_NPCS
};

unsigned char m_ucMcuNumber;
unsigned char m_ucStepIndexNow[_MAX_CHANNEL];
unsigned char m_ucStartStepIndex[_MAX_CHANNEL];
unsigned char m_bChannelRunning[_MAX_CHANNEL];
unsigned char m_bStepRunning[_MAX_CHANNEL];

unsigned char m_bRunSequence[_MAX_CHANNEL];
U8 m_bContinueNextSequence[_MAX_CHANNEL];
unsigned char m_ucStopSequence[_MAX_CHANNEL];
unsigned char m_ucPauseSequence[_MAX_CHANNEL];

unsigned char m_ucResumeSequence[_MAX_CHANNEL];
unsigned char m_ucPauseStatus[_MAX_CHANNEL];
unsigned int m_uiPauseTimeNow[_MAX_CHANNEL];
unsigned char m_ucResumeStatus[_MAX_CHANNEL];
unsigned char m_ucCompletStatus[_MAX_CHANNEL];

unsigned int m_ui1msCounter;
unsigned int m_uiFanTime;

unsigned int m_uiSystemInformation[_SystemParameterIndexLast];
U8  m_ucLedStatus;
int m_iPbcHz;
U8 m_ucPcCommPDCA_TxBuffer[PC_COMM_TX_PDCA_BUFFER_SIZE];
U8 m_ucPcCommPDCA_RxBuffer[PC_COMM_RX_PDCA_BUFFER_SIZE];

unsigned int m_uiAlarmMask[_MAX_CHANNEL];

U16 m_u16SafeErrorNumbers[_MAX_CHANNEL][32];

unsigned int m_uiSdramCheckEnalbe;
unsigned int m_uiSdramCheckPassFail;

unsigned int m_ui1SecRingCountVoltage[_MAX_CHANNEL];
unsigned int m_ui1SecRingCountCurrent[_MAX_CHANNEL];
unsigned int m_ui1SecRingCountVoltage2[_MAX_CHANNEL];
float m_fVoltage1Sec[_MAX_CHANNEL];
float m_fVoltage1SecPre[_MAX_CHANNEL];
float m_fVoltage2_1Sec[_MAX_CHANNEL];
float m_fCurrent1Sec[_MAX_CHANNEL];
int m_iCurrent1SecTotal[_MAX_CHANNEL];
int m_iVoltage1SecTotal[_MAX_CHANNEL];
int m_iVoltage2_1SecTotal[_MAX_CHANNEL];
int m_i1msVoltage[_MAX_CHANNEL];
int m_i1msCurrent[_MAX_CHANNEL];
int m_i1msVoltage2[_MAX_CHANNEL];
float m_fRecordVoltage[_MAX_CHANNEL];
float m_fRecordCurrent[_MAX_CHANNEL];
float m_fRecordVoltage2[_MAX_CHANNEL];

unsigned int m_uiRecordRingCountVoltage[_MAX_CHANNEL];
unsigned int m_uiRecordRingCountCurrent[_MAX_CHANNEL];
unsigned int m_uiRecordRingCountVoltage2[_MAX_CHANNEL];
unsigned int m_uiRecordSecond[_MAX_CHANNEL];
double m_f3SecVoltage[_MAX_CHANNEL][3];
double m_f3SecCurrent[_MAX_CHANNEL][3];
double m_f3SecVoltage2[_MAX_CHANNEL][3];

unsigned int m_uiStepTimeNow[_MAX_CHANNEL];
unsigned int m_uiStepCV_TimeNow[_MAX_CHANNEL];

float m_fChargeVoltage[_MAX_CHANNEL];
float m_fDisChargeVoltage[_MAX_CHANNEL];
float m_fChargeCurrent[_MAX_CHANNEL];
float m_fDisChargeCurrent[_MAX_CHANNEL];
double m_dblChargeCapacity[_MAX_CHANNEL];
double m_dblDisChargeCapacity[_MAX_CHANNEL];
double m_dblChargeWattHour[_MAX_CHANNEL];
double m_dblDisChargeWattHour[_MAX_CHANNEL];

U32 u32RecordHead[_MAX_CHANNEL];
U32 u32RecordTail[_MAX_CHANNEL];

_FORMATIONM_STEP_END_DATA m_DataNow[_MAX_CHANNEL];

unsigned int m_uiStepDownloadSum[_MAX_CHANNEL];
char m_bPreMcOnOff[_MAX_CHANNEL];
char m_bMcOnOffNow[_MAX_CHANNEL];
char m_bMcOff[_MAX_CHANNEL];
unsigned int m_uiMcChangeDelay[_MAX_CHANNEL];
unsigned char m_bMcControlAllComplete[_MAX_CHANNEL];
unsigned short m_usReadyToStartSequenceCounter[_MAX_CHANNEL];
unsigned char m_bReadyToStartSequence[_MAX_CHANNEL];
unsigned char m_bNewStepSetting[_MAX_CHANNEL];
unsigned short m_usScalerAdcRead;
unsigned int m_uiAdcReadCycleInterruptCount;
int m_iVoltage[_MAX_CHANNEL];
int m_iCurrent[_MAX_CHANNEL];
int m_iV[_MAX_CHANNEL];
int m_iC[_MAX_CHANNEL];
int m_iAdcReadComplete;
int m_iNowVoltageAdc[_MAX_CHANNEL];
int m_iNowCurrentAdc[_MAX_CHANNEL];
float m_fNowVoltage[_MAX_CHANNEL];
float m_fNowCurrent[_MAX_CHANNEL];
float m_fNowVoltage2[_MAX_CHANNEL];
char m_bVoltage2Mode[_MAX_CHANNEL];
U32 m_uiVoltage2ModeAddress[_MAX_CHANNEL] = {_CH1_VOLTAGE2_MODE, _CH2_VOLTAGE2_MODE, _CH3_VOLTAGE2_MODE, _CH4_VOLTAGE2_MODE};
U32 m_uiChargePullupAddress[_MAX_CHANNEL] = {_CH1_CHARGE_PULLUP, _CH2_CHARGE_PULLUP, _CH3_CHARGE_PULLUP, _CH4_CHARGE_PULLUP};
float m_fVoltage[_MAX_CHANNEL];
float m_fCurrent[_MAX_CHANNEL];
float m_fVoltage2[_MAX_CHANNEL];
float m_fWatt[_MAX_CHANNEL];
float m_fResister[_MAX_CHANNEL];
int m_iCaptainClockCountT1Over;
_FORMATIONM_STEP_SEQUENCE* m_pSeqNow[_MAX_CHANNEL];
_FORMATIONM_STEP_SEQUENCE* m_pSeqNext[_MAX_CHANNEL];
unsigned char m_ucPreState[_MAX_CHANNEL];
unsigned char m_ucNowState[_MAX_CHANNEL];
unsigned char m_ucNowRestChargeDisCharge[_MAX_CHANNEL];
unsigned char m_ucFirstStep[_MAX_CHANNEL];
unsigned char m_ucFirstStep2[_MAX_CHANNEL];
unsigned char m_ucNowMode[_MAX_CHANNEL];
unsigned char m_ucNowPulseMode[_MAX_CHANNEL];
unsigned char m_ucPulseNextState[_MAX_CHANNEL];
S16 m_s16DacVoltageNext[_MAX_CHANNEL];
S16 m_s16DacCurrentNext[_MAX_CHANNEL];
U8 m_ucNextCurrentDelay[_MAX_CHANNEL];		// bgyu 20250529

char m_bNewStepStartReady[_MAX_CHANNEL];
float m_fCvVoltageLow[_MAX_CHANNEL];
float m_fCvVoltageHigh[_MAX_CHANNEL];
float m_fStepStartVoltage[_MAX_CHANNEL];
char m_bChagedOrDisChargedNow[_MAX_CHANNEL];
float m_fAlarmVoltageLow[_MAX_CHANNEL];
float m_fAlarmVoltageHigh[_MAX_CHANNEL];
//2024.09.09 REST TOL_ABS
unsigned char m_ucREST_MODE[_MAX_CHANNEL];
float m_fAlarmTol_VoltageLow[_MAX_CHANNEL];
float m_fAlarmTol_VoltageHigh[_MAX_CHANNEL];

float m_fErrorCurrentLow[_MAX_CHANNEL];
float m_fErrorCurrentHigh[_MAX_CHANNEL];
float m_fAlarmWattLow[_MAX_CHANNEL];
float m_fAlarmWattHigh[_MAX_CHANNEL];
float m_fAlarmResisterLow[_MAX_CHANNEL];
float m_fAlarmResisterHigh[_MAX_CHANNEL];
unsigned char m_ucEEPRomControl;

_FORMATIONM_STEP_RECORD_DATA_STRUCTURE* m_pFORMATION_STEP_RECORD_DATA_STRUCTURE;
_FORMATIONM_STEP_SEQUENCE_STRUCTURE* m_pFORMATION_STEP_SEQUENCE_STRUCTURE;
_FORMATIONM_STEP_END_DATA_STRUCTURE* m_pFORMATION_STEP_END_DATA_STRUCTURE;
_EEPROM_DATA m_pEEPROM_DATA;
_1SEC_BUFFER_STRUCTURE *m_p1SecBuffer;
_DCIR_SEC_BUFFER_STRUCTURE *m_pSecDcirBuffer;	//2024.10.14
char* m_pSDRAM_TAIL;

U8 m_ucPulseIndex[_MAX_CHANNEL];
U8 m_ucPulseKind[_MAX_CHANNEL];
U16 m_u16PulseTime1ms[_MAX_CHANNEL];
U16 m_u16PulseTime1msNow[_MAX_CHANNEL];
float m_fPulseCurrentPre[_MAX_CHANNEL];
float m_fPulseCurrent[_MAX_CHANNEL];
float m_fPulseChargeCurrentPreStep[_MAX_CHANNEL];		//bgyu 2025.06.16
float m_fPulseDisChargeCurrentPreStep[_MAX_CHANNEL];	//bgyu 2025.06.16
U8 m_bNewPulseLoad[_MAX_CHANNEL];
U8 m_bStepStarted[_MAX_CHANNEL];
U8 m_ucCalculateMode[_MAX_CHANNEL];

float m_fSoc[_MAX_CHANNEL];
float m_fDod[_MAX_CHANNEL];

S16 m_s16PulseCurrentDac1[_MAX_CHANNEL];
S16 m_s16PulseCurrentDac2[_MAX_CHANNEL];
float m_fStartPulseCurrent[_MAX_CHANNEL];
float m_fPulsePer1msDelta[_MAX_CHANNEL];
float m_fPulseTriangleCpCr1[_MAX_CHANNEL];
float m_fPulseTriangleCpCr2[_MAX_CHANNEL];
float m_fDCIR_Voltage1[_MAX_CHANNEL];
float m_fDCIR_Voltage2[_MAX_CHANNEL][MAX_DCIR_10MS];
float m_fDCIR_Current[_MAX_CHANNEL][MAX_DCIR_10MS];
U8 m_bDCIR_StepStop[_MAX_CHANNEL];
U8 m_ucChargePowerControl;
U8 m_bChargePowerControlOne;
U8 m_bFanOff;
U8 m_bFanError;
U8 m_bAirTemperaturError;
U8 m_bHeatSinkTemperaturError[_MAX_CHANNEL];	//LJK 2024.07.16
U8 m_bVoltageDeltaError[2][_MAX_CHANNEL];
U8 m_bLedRunStatus;

avr32_gpio_port_t *pGpioLedPortAlarm = (avr32_gpio_port_t *)&AVR32_GPIO.port[(_ALARM_LED&0xFFF) >> 5];
avr32_gpio_port_t *pGpioLedPortRun = (avr32_gpio_port_t *)&AVR32_GPIO.port[(_RUN_LED&0xFFF) >> 5];

int m_iDacReadBack;
short m_s16DacCurrent[_MAX_CHANNEL];
U8 m_bDacCurrentSoftUp[_MAX_CHANNEL];
U8 m_bSoftCurrentMode[_MAX_CHANNEL];
U8 m_bAlarmParameterSet[_MAX_CHANNEL];

U8 m_bHardWareInit;		//HardWare Init Flag
unsigned int m_uiRecordMilliSecond[_MAX_CHANNEL];	// Record Second(ms)
unsigned char m_bAdcTimeOver = FALSE;
float m_fAdcPreVoltage[_MAX_CHANNEL];
float m_fAdcPreWatt[_MAX_CHANNEL];
float m_fAdcPreCurrent[_MAX_CHANNEL];
short m_shCurrentCellTemp[_MAX_CHANNEL];	//20230104 Cell Temp
short m_shMaxCellTemp[_MAX_CHANNEL];		//20240701 Max Cell Temp 
float m_fInternalADCReadVoltage[_MUX_MAX_CHANNEL];	//2023.04.24 LJK
float m_fInternalADCAirTempVoltage;					//2023.04.24 LJK
//int m_fInternalADCSum[_MUX_MAX_CHANNEL];			//2023.05.17 LJK
U8 m_bTrTf_Delay[_MUX_MAX_CHANNEL];	//LJK 2025.03.11
float m_fHeatSinkTemp[_MAX_CHANNEL];	//LJK 2025.04.25
float m_fAirTemp;						//LJK 2025.04.25

#ifdef SUPPORT_PROTECTION_CONDITION
float m_fReverseCellVolt;

_PROTECTION_CONDITION_TYPE m_pProtectionType[_MAX_CHANNEL];
#endif

#ifdef SUPPORT_BLACK_OUT
unsigned int m_uiPauseStepTimeNow[_MAX_CHANNEL];
U16 m_u16PausePulseTime1msNow[_MAX_CHANNEL];  
PAUSE_INFO_DATA m_pEEPROM_PAUSE_INFO_DATA;
float m_fPulseRefCurrent[_MAX_CHANNEL];
unsigned char m_ucPauseSequenceDelay[_MAX_CHANNEL];
unsigned char m_ucBlackOutFlag[_MAX_CHANNEL] = {0,0,0,0};
#endif

void SDRAM_AddressSet( void )
{
	m_pFORMATION_STEP_RECORD_DATA_STRUCTURE = (_FORMATIONM_STEP_RECORD_DATA_STRUCTURE* )(EXT_SDRAM_BASE_ADDRESS);
	m_pFORMATION_STEP_SEQUENCE_STRUCTURE = (_FORMATIONM_STEP_SEQUENCE_STRUCTURE*)((unsigned int)m_pFORMATION_STEP_RECORD_DATA_STRUCTURE + sizeof(_FORMATIONM_STEP_RECORD_DATA_STRUCTURE));
	m_pFORMATION_STEP_END_DATA_STRUCTURE = (_FORMATIONM_STEP_END_DATA_STRUCTURE*)((unsigned int)m_pFORMATION_STEP_SEQUENCE_STRUCTURE + sizeof(_FORMATIONM_STEP_SEQUENCE_STRUCTURE));
	//m_pEEPROM_DATA = (_EEPROM_DATA*)((unsigned int)m_pFORMATION_STEP_END_DATA_STRUCTURE + sizeof(_FORMATIONM_STEP_END_DATA_STRUCTURE));
	m_p1SecBuffer = (_1SEC_BUFFER_STRUCTURE*)((unsigned int)m_pFORMATION_STEP_END_DATA_STRUCTURE + sizeof(_FORMATIONM_STEP_END_DATA_STRUCTURE));
	
	m_pSecDcirBuffer = (_DCIR_SEC_BUFFER_STRUCTURE*)((unsigned int)m_p1SecBuffer + sizeof(_1SEC_BUFFER_STRUCTURE));
	m_pSDRAM_TAIL = (char*)((unsigned int)m_pSecDcirBuffer + sizeof(_DCIR_SEC_BUFFER_STRUCTURE));
	
	//m_pSDRAM_TAIL = (char*)((unsigned int)m_p1SecBuffer + sizeof(_1SEC_BUFFER_STRUCTURE));
	
	//m_pEEPROM_DATA = (_EEPROM_DATA*)((unsigned int)m_pFORMATION_STEP_END_DATA_STRUCTURE + sizeof(_FORMATIONM_STEP_END_DATA_STRUCTURE));
	//m_p1SecBuffer = (_1SEC_BUFFER_STRUCTURE*)((unsigned int)m_pEEPROM_DATA + sizeof(_EEPROM_DATA));
}