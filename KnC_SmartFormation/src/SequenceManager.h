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
void CalculatePulseStepStart( unsigned char ucCh );
void SetNextPulse( unsigned char ucCh );
void CpCrMode( unsigned char ucCh );
void StepEndCheck( unsigned char ucCh );
U8 StepEndConditionCheck( U8 ucCh );
void NowModeStateSet( unsigned char ucCh );

#endif /* SEQUENCEMANAGER_H_ */