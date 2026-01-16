/*
 * SystemControl.h
 *
 * Created: 2021-11-26 AM 00:05:00
 *  Author: KnC Corporation, bgyu
 */ 


#ifndef SYSTEMCONTROL_H_
#define SYSTEMCONTROL_H_

#define		MC_CONTROL_MAX_TIME	800
#define		MC_CONTROL_TIME	(MC_CONTROL_MAX_TIME/2)

void HardWareInitialize( void );
void InitializeVariable( void );
void PowerUp_First_Initialize( void );
void McControl( unsigned char ucCh );
void StepEnd( unsigned char ucCh, int ucEndKind );
void FormationEnd( unsigned char ucCh );
void MC_Off( unsigned char ucCh );
void DacReset( unsigned char ucCh );
void PortReset( unsigned char ucCh );
bool SetDacWriteData( int iDacChannel, unsigned int uiData );
bool SetDac2WriteData( int iDacChannel, unsigned int uiData );
void SetDacSpiWriteBits( unsigned long ucDacSdiWriteData );
int GetDacSpiReadBits( void );
bool SetDacWriteDataRetry( int iDacChannel, unsigned int uiData );
int ReadDacData( int iDacChannel, int iWritedData );
void PushStepRecordData( unsigned char ucCh, unsigned char ucEndKind );
int GetStepRecordRemainNumber( unsigned char ucCh );
_FORMATIONM_STEP_RECORD_DATA* GetStepRecordHeadPointer( unsigned char ucCh );
void DeletePopStepRecordDataAll( unsigned char ucCh );
U32 GetStepRecordNumber( unsigned char ucCh );
U32 DeletePopStepRecordData( unsigned char ucCh );
void MakeStepDataOne_Channel( unsigned char ucCh, unsigned char ucEndCondition );
void SaveStepEndData( unsigned char ucCh, unsigned char ucEndCondition );
void ClearStepEndData( unsigned char ucCh, unsigned char ucStep );
void ClearStepEndDataAll( unsigned char ucCh );
void SystemInfoCleanAndFwVersion(void);
unsigned short FanAlarmAirTempCheck( void );
void GetRingBufferVoltage( unsigned char ucCh );
void GetRingBufferCurrent( unsigned char ucCh );
void GetRingBufferVoltage2( unsigned char ucCh );
bool ReadAdcSpi8One( int* pVoltage, int* pCurrent );
bool ReadAdcData( char* cReadData );
float GetHexToVoltageCurrent( unsigned char iRange, int iAdc );
short GetVoltageCurrentToDacHex( unsigned char iRange, float fData );

void ChargePowerControl( void );
void SetChargePowerVoltage( U8 ucCh, float fVoltage );
void ChargePowerSet( U8 ucCh );
float GetFetVoltage( U8 ucCh );
void SetGate( U8 ucBits );

// 20230106 djl 다중Loop 관련 구현
void ClearLoopNowCount(unsigned char ucChannel, uint startIndex, uint endIndex);
void GetFetTempNVolt2( void );

#ifdef SUPPORT_BLACK_OUT
void BlackOutStepEnd( unsigned char ucCh, int ucEndKind );
void PauseResumeControl ( void);
#endif

#endif /* SYSTEMCONTROL_H_ */