/*
 * GlobalData.h
 *
 * Created: 2021-11-26 AM 00:05:00
 *  Author: 4N Corporation, bgyu
 */ 


#ifndef GlobalData_H_
#define GlobalData_H_

#define  _LED_ALARM_TOGGLE		(pGpioLedPortAlarm->ovrt  = 1 << (_ALARM_LED & 0x1F))
#define  _LED_ALARM_ON			(pGpioLedPortAlarm->ovrc  = 1 << (_ALARM_LED & 0x1F))
#define  _LED_ALARM_OFF			(pGpioLedPortAlarm->ovrs  = 1 << (_ALARM_LED & 0x1F))
#define  _LED_RUN_TOGGLE		(pGpioLedPortRun->ovrt  = 1 << (_RUN_LED & 0x1F))
#define  _LED_RUN_ON			(pGpioLedPortRun->ovrc  = 1 << (_RUN_LED & 0x1F))
#define  _LED_RUN_OFF			(pGpioLedPortRun->ovrs  = 1 << (_RUN_LED & 0x1F))
/*
#define  _LED_CH1_TOGGLE		(pGpioLedPortCh1->ovrt  = 1 << (_CH1_LED & 0x1F))
#define  _LED_CH1_ON			(pGpioLedPortCh1->ovrc  = 1 << (_CH1_LED & 0x1F))
#define  _LED_CH1_OFF			(pGpioLedPortCh1->ovrs  = 1 << (_CH1_LED & 0x1F))
#define  _LED_CH2_TOGGLE		(pGpioLedPortCh2->ovrt  = 1 << (_CH2_LED & 0x1F))
#define  _LED_CH2_ON			(pGpioLedPortCh2->ovrc  = 1 << (_CH2_LED & 0x1F))
#define  _LED_CH2_OFF			(pGpioLedPortCh2->ovrs  = 1 << (_CH2_LED & 0x1F))
#define  _LED_CH3_TOGGLE		(pGpioLedPortCh3->ovrt  = 1 << (_CH3_LED & 0x1F))
#define  _LED_CH3_ON			(pGpioLedPortCh3->ovrc  = 1 << (_CH3_LED & 0x1F))
#define  _LED_CH3_OFF			(pGpioLedPortCh3->ovrs  = 1 << (_CH3_LED & 0x1F))
#define  _LED_CH4_TOGGLE		(pGpioLedPortCh4->ovrt  = 1 << (_CH4_LED & 0x1F))
#define  _LED_CH4_ON			(pGpioLedPortCh4->ovrc  = 1 << (_CH4_LED & 0x1F))
#define  _LED_CH4_OFF			(pGpioLedPortCh4->ovrs  = 1 << (_CH4_LED & 0x1F))
*/
enum
{
	_CHARGE_POWER_FIRST=0,
	_CHARGE_POWER_SET,
};
extern struct spi_device DAC_SPI_DEVICE;
extern struct spi_device ADC_SPI_DEVICE;

extern unsigned char m_ucMcuNumber;
extern unsigned char m_ucStepIndexNow[_MAX_CHANNEL];
extern unsigned char m_ucStartStepIndex[_MAX_CHANNEL];
extern unsigned char m_bChannelRunning[_MAX_CHANNEL];
extern unsigned char m_bStepRunning[_MAX_CHANNEL];
extern U8 m_ucCalculateMode[_MAX_CHANNEL];

extern unsigned char m_bRunSequence[_MAX_CHANNEL];
extern U8 m_bContinueNextSequence[_MAX_CHANNEL];
extern unsigned char m_ucStopSequence[_MAX_CHANNEL];
extern unsigned char m_ucPauseSequence[_MAX_CHANNEL];

extern unsigned char m_ucResumeSequence[_MAX_CHANNEL];
extern unsigned char m_ucPauseStatus[_MAX_CHANNEL];
extern unsigned int m_uiPauseTimeNow[_MAX_CHANNEL];
extern unsigned char m_ucResumeStatus[_MAX_CHANNEL];
extern unsigned char m_ucCompletStatus[_MAX_CHANNEL];

extern _FORMATIONM_STEP_RECORD_DATA_STRUCTURE* m_pFORMATION_STEP_RECORD_DATA_STRUCTURE;
extern _FORMATIONM_STEP_SEQUENCE_STRUCTURE* m_pFORMATION_STEP_SEQUENCE_STRUCTURE;
extern _FORMATIONM_STEP_END_DATA_STRUCTURE* m_pFORMATION_STEP_END_DATA_STRUCTURE;
extern _EEPROM_DATA m_pEEPROM_DATA;
extern _1SEC_BUFFER_STRUCTURE *m_p1SecBuffer;
extern _DCIR_SEC_BUFFER_STRUCTURE *m_pSecDcirBuffer;
extern char* m_pSDRAM_TAIL;

extern unsigned int m_ui1msCounter;
extern unsigned int m_uiFanTime;

extern unsigned int m_uiSystemInformation[_SystemParameterIndexLast];
extern U8  m_ucLedStatus;
extern int m_iPbcHz;
extern U8 m_ucPcCommPDCA_TxBuffer[PC_COMM_TX_PDCA_BUFFER_SIZE];
extern U8 m_ucPcCommPDCA_RxBuffer[PC_COMM_RX_PDCA_BUFFER_SIZE];

extern unsigned int m_uiAlarmMask[_MAX_CHANNEL];

extern U16 m_u16SafeErrorNumbers[_MAX_CHANNEL][32];
extern U16 m_u16SafeRetryNumbers[_MAX_CHANNEL][32];

extern unsigned int m_uiSdramCheckEnalbe;
extern unsigned int m_uiSdramCheckPassFail;

extern unsigned int m_ui1SecRingCountVoltage[_MAX_CHANNEL];
extern unsigned int m_ui1SecRingCountCurrent[_MAX_CHANNEL];
extern unsigned int m_ui1SecRingCountVoltage2[_MAX_CHANNEL];
extern float m_fVoltage1Sec[_MAX_CHANNEL];
extern float m_fVoltage1SecPre[_MAX_CHANNEL];
extern float m_fVoltage2_1Sec[_MAX_CHANNEL];
extern float m_fCurrent1Sec[_MAX_CHANNEL];
extern int m_iCurrent1SecTotal[_MAX_CHANNEL];
extern int m_iVoltage1SecTotal[_MAX_CHANNEL];
extern int m_iVoltage2_1SecTotal[_MAX_CHANNEL];
extern int m_i1msVoltage[_MAX_CHANNEL];
extern int m_i1msCurrent[_MAX_CHANNEL];
extern int m_i1msVoltage2[_MAX_CHANNEL];
extern float m_fRecordVoltage[_MAX_CHANNEL];
extern float m_fRecordCurrent[_MAX_CHANNEL];
extern float m_fRecordVoltage2[_MAX_CHANNEL];

extern unsigned int m_uiRecordRingCountVoltage[_MAX_CHANNEL];
extern unsigned int m_uiRecordRingCountCurrent[_MAX_CHANNEL];
extern unsigned int m_uiRecordRingCountVoltage2[_MAX_CHANNEL];
extern unsigned int m_uiRecordSecond[_MAX_CHANNEL];
extern double m_f3SecVoltage[_MAX_CHANNEL][3];
extern double m_f3SecCurrent[_MAX_CHANNEL][3];
extern double m_f3SecVoltage2[_MAX_CHANNEL][3];

extern unsigned int m_uiStepTimeNow[_MAX_CHANNEL];
extern unsigned int m_uiStepCV_TimeNow[_MAX_CHANNEL];

extern float m_fChargeVoltage[_MAX_CHANNEL];
extern float m_fDisChargeVoltage[_MAX_CHANNEL];
extern float m_fChargeCurrent[_MAX_CHANNEL];
extern float m_fDisChargeCurrent[_MAX_CHANNEL];
extern double m_dblChargeCapacity[_MAX_CHANNEL];
extern double m_dblDisChargeCapacity[_MAX_CHANNEL];
extern double m_dblChargeWattHour[_MAX_CHANNEL];
extern double m_dblDisChargeWattHour[_MAX_CHANNEL];

extern U32 u32RecordHead[_MAX_CHANNEL];
extern U32 u32RecordTail[_MAX_CHANNEL];

extern _FORMATIONM_STEP_END_DATA m_DataNow[_MAX_CHANNEL];

extern unsigned int m_uiStepDownloadSum[_MAX_CHANNEL];
extern char m_bPreMcOnOff[_MAX_CHANNEL];
extern char m_bMcOnOffNow[_MAX_CHANNEL];
extern char m_bMcOff[_MAX_CHANNEL];
extern unsigned int m_uiMcChangeDelay[_MAX_CHANNEL];
extern unsigned char m_bMcControlAllComplete[_MAX_CHANNEL];
extern unsigned short m_usReadyToStartSequenceCounter[_MAX_CHANNEL];
extern unsigned char m_bReadyToStartSequence[_MAX_CHANNEL];
extern unsigned char m_bNewStepSetting[_MAX_CHANNEL];
extern unsigned short m_usScalerAdcRead;
extern unsigned int m_uiAdcReadCycleInterruptCount;
extern int m_iVoltage[_MAX_CHANNEL];
extern int m_iCurrent[_MAX_CHANNEL];
extern int m_iV[_MAX_CHANNEL];
extern int m_iC[_MAX_CHANNEL];
extern int m_iAdcReadComplete;
extern int m_iNowVoltageAdc[_MAX_CHANNEL];
extern int m_iNowCurrentAdc[_MAX_CHANNEL];
extern float m_fNowVoltage[_MAX_CHANNEL];
extern float m_fNowCurrent[_MAX_CHANNEL];
extern float m_fNowVoltage2[_MAX_CHANNEL];
extern char m_bVoltage2Mode[_MAX_CHANNEL];
extern U32 m_uiVoltage2ModeAddress[_MAX_CHANNEL];
extern U32 m_uiChargePullupAddress[_MAX_CHANNEL];
extern float m_fVoltage[_MAX_CHANNEL];
extern float m_fCurrent[_MAX_CHANNEL];
extern float m_fVoltage2[_MAX_CHANNEL];
extern float m_fWatt[_MAX_CHANNEL];
extern float m_fResister[_MAX_CHANNEL];
extern int m_iCaptainClockCountT1Over;
extern _FORMATIONM_STEP_SEQUENCE* m_pSeqNow[_MAX_CHANNEL];
extern _FORMATIONM_STEP_SEQUENCE* m_pSeqNext[_MAX_CHANNEL];
extern unsigned char m_ucPreState[_MAX_CHANNEL];
extern unsigned char m_ucNowState[_MAX_CHANNEL];
extern unsigned char m_ucFirstStep[_MAX_CHANNEL];
extern unsigned char m_ucFirstStep2[_MAX_CHANNEL];
extern unsigned char m_ucNowMode[_MAX_CHANNEL];
extern unsigned char m_ucNowPulseMode[_MAX_CHANNEL];
extern unsigned char m_ucPulseNextState[_MAX_CHANNEL];
extern S16 m_s16DacVoltageNext[_MAX_CHANNEL];
extern S16 m_s16DacCurrentNext[_MAX_CHANNEL];
extern U8 m_ucNextCurrentDelay[_MAX_CHANNEL];	// bgyu 20250529
extern char m_bNewStepStartReady[_MAX_CHANNEL];
extern float m_fCvVoltageLow[_MAX_CHANNEL];
extern float m_fCvVoltageHigh[_MAX_CHANNEL];
extern float m_fStepStartVoltage[_MAX_CHANNEL];
extern char m_bChagedOrDisChargedNow[_MAX_CHANNEL];
extern float m_fAlarmVoltageLow[_MAX_CHANNEL];
extern float m_fAlarmVoltageHigh[_MAX_CHANNEL];
//2024.09.09 REST TOL_ABS
extern unsigned char m_ucREST_MODE[_MAX_CHANNEL];
extern float m_fAlarmTol_VoltageLow[_MAX_CHANNEL];
extern float m_fAlarmTol_VoltageHigh[_MAX_CHANNEL];

extern float m_fErrorCurrentLow[_MAX_CHANNEL];
extern float m_fErrorCurrentHigh[_MAX_CHANNEL];
extern float m_fAlarmWattLow[_MAX_CHANNEL];
extern float m_fAlarmWattHigh[_MAX_CHANNEL];
extern float m_fAlarmResisterLow[_MAX_CHANNEL];
extern float m_fAlarmResisterHigh[_MAX_CHANNEL];
extern unsigned char m_ucEEPRomControl;
extern U8 m_ucPulseIndex[_MAX_CHANNEL];
extern U8 m_ucPulseKind[_MAX_CHANNEL];
extern U16 m_u16PulseTime1ms[_MAX_CHANNEL];
extern U16 m_u16PulseTime1msNow[_MAX_CHANNEL];
extern float m_fPulseCurrentPre[_MAX_CHANNEL];
extern float m_fPulseCurrent[_MAX_CHANNEL];
extern float m_fPulseChargeCurrentPreStep[_MAX_CHANNEL];	//bgyu 2025.06.16
extern float m_fPulseDisChargeCurrentPreStep[_MAX_CHANNEL];	//bgyu 2025.06.16
extern U8 m_bNewPulseLoad[_MAX_CHANNEL];
extern U8 m_bStepStarted[_MAX_CHANNEL];
extern float m_fSoc[_MAX_CHANNEL];
extern float m_fDod[_MAX_CHANNEL];
extern S16 m_s16PulseCurrentDac1[_MAX_CHANNEL];
extern S16 m_s16PulseCurrentDac2[_MAX_CHANNEL];
extern float m_fStartPulseCurrent[_MAX_CHANNEL];
extern float m_fPulsePer1msDelta[_MAX_CHANNEL];
extern float m_fPulseTriangleCpCr1[_MAX_CHANNEL];
extern float m_fPulseTriangleCpCr2[_MAX_CHANNEL];
extern float m_fDCIR_Voltage1[_MAX_CHANNEL];
extern float m_fDCIR_Voltage2[_MAX_CHANNEL][MAX_DCIR_10MS];
extern float m_fDCIR_Current[_MAX_CHANNEL][MAX_DCIR_10MS];
extern U8 m_bDCIR_StepStop[_MAX_CHANNEL];
extern avr32_gpio_port_t *pGpioLedPortAlarm;
extern avr32_gpio_port_t *pGpioLedPortRun;
extern avr32_gpio_port_t *pGpioLedPortCh1;
extern avr32_gpio_port_t *pGpioLedPortCh2;
extern avr32_gpio_port_t *pGpioLedPortCh3;
extern avr32_gpio_port_t *pGpioLedPortCh4;
extern U8 m_ucChargePowerControl;
extern U8 m_bChargePowerControlOne;
extern U8 m_bFanOff;
extern U8 m_bFanError;
extern U8 m_bAirTemperaturError;
extern U8 m_bHeatSinkTemperaturError[_MAX_CHANNEL];
extern U8 m_bVoltageDeltaError[2][_MAX_CHANNEL];
extern U8 m_bLedRunStatus;
extern int m_iDacReadBack;
extern unsigned char m_ucNowRestChargeDisCharge[_MAX_CHANNEL];
extern short m_s16DacCurrent[_MAX_CHANNEL];
extern U8 m_bDacCurrentSoftUp[_MAX_CHANNEL];
extern U8 m_bSoftCurrentMode[_MAX_CHANNEL];
extern U8 m_bAlarmParameterSet[_MAX_CHANNEL];

extern U8 m_bHardWareInit;
extern unsigned int m_uiRecordMilliSecond[_MAX_CHANNEL];
extern unsigned char m_bAdcTimeOver;
extern float m_fAdcPreVoltage[_MAX_CHANNEL];
extern float m_fAdcPreWatt[_MAX_CHANNEL];
extern float m_fAdcPreCurrent[_MAX_CHANNEL];
extern short m_shCurrentCellTemp[_MAX_CHANNEL];
extern short m_shMaxCellTemp[_MAX_CHANNEL];		//20240701 Max Cell Temp 
extern float m_fInternalADCReadVoltage[_MUX_MAX_CHANNEL];	//2023.04.24 LJK
extern float m_fInternalADCAirTempVoltage;					//2023.04.24 LJK
//extern int m_fInternalADCSum[_MUX_MAX_CHANNEL];			//2023.05.17 LJK
extern U8 m_bTrTf_Delay[_MUX_MAX_CHANNEL];	//LJK 2025.03.11
extern float m_fHeatSinkTemp[_MAX_CHANNEL];	//LJK 2025.04.25
extern float m_fAirTemp;					//LJK 2025.04.25

#ifdef SUPPORT_PROTECTION_CONDITION
extern float m_fReverseCellVolt;

extern _PROTECTION_CONDITION_TYPE m_pProtectionType[_MAX_CHANNEL];
#endif

#ifdef SUPPORT_BLACK_OUT
extern unsigned int m_uiPauseStepTimeNow[_MAX_CHANNEL];
extern U16 m_u16PausePulseTime1msNow[_MAX_CHANNEL]; 
extern PAUSE_INFO_DATA m_pEEPROM_PAUSE_INFO_DATA;
extern float m_fPulseRefCurrent[_MAX_CHANNEL];
extern unsigned char m_ucPauseSequenceDelay[_MAX_CHANNEL];
extern unsigned char m_ucBlackOutFlag[_MAX_CHANNEL];dd
#endif


void SDRAM_AddressSet( void );

#endif /* GlobalData_H_ */
