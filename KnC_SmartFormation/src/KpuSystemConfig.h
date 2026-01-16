/*
 * KpuSystemConfig.h
 *
 * Created: 2021-11-26 AM 00:05:00
 *  Author: KnC Corporation, bgyu
 */ 


#ifndef KPUSYSTEMCONFIG_H_
#define KPUSYSTEMCONFIG_H_

void KpuPinDefineAndInitialize( void );
void KpuFrequenceClockSetUP( void );
void KpuSdram16M16BitInitialize( void );
void PcCommUsartSetup( void );
void KpuDacSPIUsartSetup( void );
void KpuAdcSPIUsartSetup( void );
void PDCA_PcCommSetup( void );
void PDCA_SetupForUsart( int iPdcaChannel, int iPdcaPid, unsigned char* pBuffer, int iBufferSize  );
void PDCA_ReLoadForUsart( int iPdcaChannel, unsigned char* pBuffer, unsigned long iBufferSize  );
void LedStatusSet( void );
void EtcAlarmSet( void );
float GetAirTemperature( void );
float GetHeatSinkTemperaturError( unsigned char ucCh );
void CommunicationTimerCounterSubRoutine(void);
void SetUpInterruptHandler( void );
void CaptainTimerCounterSetup( volatile avr32_tc_t *pTc );
void AdcReadCycleTimerCounterSetup( volatile avr32_tc_t *pTc );
void TimerInterruptCounterStart( void );
void SetUpInternelADC( void );

enum
{
	_STATUS_RUN,		// On/Off 500ms/500ms
	_STATUS_STOP,		// Off
	_STATUS_STEP_END,	// On
};

#endif /* KPUSYSTEMCONFIG_H_ */