/*
 * SequenceManager.h
 *
 * Created: 2021-11-26 AM 00:05:00
 *  Author: KnC Corporation, bgyu
 */ 


#ifndef SEQUENCEMANAGER_H_
#define SEQUENCEMANAGER_H_

bool ReadyToStartForSafety( unsigned char ucCh );
void FanControl( U8 bOnOff );
unsigned short StartAlarmCheck( unsigned char ucCh );
unsigned short AlarmCheck( unsigned char ucCh );
unsigned short GetAlarmMask( unsigned char ucCh, unsigned short usR );
void AlarmParameterSet( unsigned char ucCh );
unsigned short AlarmCheckProcess( unsigned char ucCh );
void NewStepSetting( unsigned char ucCh );
void SetPulseModeChange( unsigned char ucCh );
char FormationEndCheck( unsigned char ucCh );
void StepStart( unsigned char ucCh );
void CalculateCapacity( unsigned char ucCh );
#ifdef SUPPORT_BLACK_OUT
void CalculatePulseStepStart( unsigned char ucCh, unsigned char ucResumeStatus );
#else
void CalculatePulseStepStart( unsigned char ucCh );
#endif
void SetNextPulse( unsigned char ucCh );
void CpCrMode( unsigned char ucCh );
void StepEndCheck( unsigned char ucCh );
U8 StepEndConditionCheck( U8 ucCh );
void NowModeStateSet( unsigned char ucCh );
#ifdef SUPPORT_BLACK_OUT
void AlarmCheckAndMoveToPause( unsigned char ucCh );
void SetPulseResume( unsigned char ucCh );
#endif

#endif /* SEQUENCEMANAGER_H_ */