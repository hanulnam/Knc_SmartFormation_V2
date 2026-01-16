/*
 * SequenceManager.c
 *
 * Created: 2021-11-26 AM 00:05:00
 *  Author: 4N Corporation, bgyu
 */ 


#include "KnC_IncludeFile.h"

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Time
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


bool ReadyToStartForSafety( unsigned char ucCh )
{
	unsigned short usRet = 0;
	switch( m_usReadyToStartSequenceCounter[ucCh] )
	{
		case 0:
			m_bFanError =
			m_bAirTemperaturError = FALSE;
			m_bHeatSinkTemperaturError[0]=m_bHeatSinkTemperaturError[1]= FALSE;	//LJK 2024.07.16
			m_bHeatSinkTemperaturError[2]=m_bHeatSinkTemperaturError[3]= FALSE;	//LJK 2024.07.16
			
			if ( m_ucFirstStep[ucCh] )
			{
				if ( m_bChannelRunning[ucCh] )
				{
					usRet = StartAlarmCheck( ucCh );
				}
			}			
			if ( usRet )
			{
				if ( m_bChannelRunning[ucCh] )
				{
					// 20230119 djl Error 발생 시 Time 값 초기화
					m_uiStepTimeNow[ucCh] = 0;
					
					StepEnd( ucCh, usRet ? usRet : _STEP_ALARM_OTHER_CHANNEL );
					FormationEnd( ucCh );
					
					m_bChannelRunning[ucCh] = FALSE;
				}
				return TRUE;	// Error Stop
			}
			
			gpio_set_pin_low(m_uiVoltage2ModeAddress[ucCh]);
			m_bVoltage2Mode[ucCh] = 0;
			break;
		case 1:
			m_ucChargePowerControl = _CHARGE_POWER_FIRST;
			break;
		case 2:
			if ( m_bChannelRunning[ucCh] )
			{
				m_uiMcChangeDelay[ucCh] = MC_CONTROL_MAX_TIME;
			}
			return TRUE;
	}

	return FALSE;
}

void FanControl( U8 bOnOff )
{
	unsigned char ucCh;

	if ( bOnOff )
	{
		m_bFanOff = FALSE;
		PDC_LOW = _PORT_C_FAN_OFF_H;
		m_u16SafeErrorNumbers[0][RTY_FAN] = 0;
	}
	else
	{
		for(ucCh=0; ucCh < _MAX_CHANNEL; ucCh++)
			if(m_bChannelRunning[ucCh] == TRUE)
				return;
				
		m_uiFanTime = 5*60;		// 5분 bgyu 20250528
		m_bFanOff = TRUE;
	}
}

unsigned short StartAlarmCheck( unsigned char ucCh )
{
	float fDiff;
	
    #ifdef MODIFY_ALARM_CONDITION 
    uint8_t volt1SensorError = FALSE;
    uint8_t volt2SensorError = FALSE;
    
    if(m_fVoltage1Sec[ucCh]   < m_pProtectionType[ucCh].fReverseCellVolt) volt1SensorError = TRUE;
    if(m_fVoltage2_1Sec[ucCh] < m_pProtectionType[ucCh].fReverseCellVolt) volt2SensorError = TRUE;

    if((volt1SensorError == TRUE) && (volt2SensorError == TRUE)) return _STEP_ALARM_REVERSE_CELL;
    else if(volt1SensorError == TRUE) return _STEP_ALARM_VSENSOR_MINUS;
    else if(volt2SensorError == TRUE) return _STEP_ALARM_ISENSOR_VOLTAGE_MINUS;
    #else
	if ( m_fVoltage1Sec[ucCh]<-0.2f  )	//bgyu 20250612
		return _STEP_ALARM_VSENSOR_MINUS;		
	if ( m_fVoltage2_1Sec[ucCh]<-0.2f  )
		return _STEP_ALARM_ISENSOR_VOLTAGE_MINUS;
	#endif
	
	if ( m_fVoltage1Sec[ucCh]>5.000f )
		return _STEP_ALARM_VSENSOR_OVER;
			
	if ( m_fVoltage2_1Sec[ucCh]>5.000f )
		return _STEP_ALARM_ISENSOR_VOLTAGE_OVER;
		
	fDiff = m_fVoltage1Sec[ucCh]-m_fVoltage2_1Sec[ucCh];
	
	if ( fDiff<0.0f )
		fDiff = -fDiff;
		
	if (fDiff>0.020f)	// LJK 2024.02.03 15mV -> 30mV 차이나면 Error (유병길 부사장님 요청사항)
		return _STEP_ALARM_VOLTAGE_BALANCE_ERROR;	
		
	return 0;
}

unsigned short AlarmCheck( unsigned char ucCh )
{
	unsigned short usR = AlarmCheckProcess( ucCh );
	if ( usR )
		usR = GetAlarmMask( ucCh, usR );
		
	return usR;
	/*
	unsigned short usR=0;
	
	usR = AlarmCheckProcess( ucCh );
	if ( usR )
		usR = GetAlarmMask( ucCh, usR );
	if ( usR )
	{
		return usR;
	}
	return usR;
	*/
}

unsigned short GetAlarmMask( unsigned char ucCh, unsigned short usR )
{
	
	//if ( usR==0 )
	//	return 0;
	
	unsigned int uiMask;
	switch( usR )
	{
		case _STEP_ALARM_UVP:						uiMask = 1<<_STEP_ALARM_MASK_UVP; break;
		case _STEP_ALARM_OVP:						uiMask = 1<<_STEP_ALARM_MASK_OVP; break;
		case _STEP_ALARM_UCP:						uiMask = 1<<_STEP_ALARM_MASK_UCP; break;
		case _STEP_ALARM_OCP:						uiMask = 1<<_STEP_ALARM_MASK_OCP; break;
		case _STEP_ALARM_UWP:						uiMask = 1<<_STEP_ALARM_MASK_UWP; break;
		case _STEP_ALARM_OWP:						uiMask = 1<<_STEP_ALARM_MASK_OWP; break;
		case _STEP_ALARM_URP:						uiMask = 1<<_STEP_ALARM_MASK_URP; break;
		case _STEP_ALARM_ORP:						uiMask = 1<<_STEP_ALARM_MASK_ORP; break;
		case _STEP_ALARM_CHARGE_VOLTAGE_DOWN:		uiMask = 1<<_STEP_ALARM_MASK_CHARGE_VOLTAGE_DOWN; break;
		case _STEP_ALARM_DISCHARGE_VOLTAGE_UP:		uiMask = 1<<_STEP_ALARM_MASK_DISCHARGE_VOLTAGE_UP; break;
		case _STEP_ALARM_MINUS_VOLTAGE:				uiMask = 1<<_STEP_ALARM_MASK_MINUS_VOLTAGE; break;
		case _STEP_ALARM_VOLTAGE_FIX_ERROR:			uiMask = 1<<_STEP_ALARM_MASK_VOLTAGE_FIX_ERROR; break;
		case _STEP_ALARM_VSENSOR_OPEN:				uiMask = 1<<_STEP_ALARM_MASK_VSENSOR_OPEN; break;
		case _STEP_ALARM_VOLTAGE_BALANCE_ERROR:		uiMask = 1<<_STEP_ALARM_MASK_VOLTAGE_BALANCE_ERROR; break;
		case _STEP_ALARM_VSENSOR_MINUS:				uiMask = 1<<_STEP_ALARM_MASK_VSENSOR_MINUS; break;
		case _STEP_ALARM_VSENSOR_OVER:				uiMask = 1<<_STEP_ALARM_MASK_VSENSOR_OVER; break;
		case _STEP_ALARM_ISENSOR_VOLTAGE_MINUS:		uiMask = 1<<_STEP_ALARM_MASK_ISENSOR_VOLTAGE_MINUS; break;
		case _STEP_ALARM_ISENSOR_VOLTAGE_OVER:		uiMask = 1<<_STEP_ALARM_MASK_ISENSOR_VOLTAGE_OVER; break;
		case _STEP_ALARM_FAN:						uiMask = 1<<_STEP_ALARM_MASK_FAN; break;
		case _STEP_ALARM_AIR_TEMP:					uiMask = 1<<_STEP_ALARM_MASK_AIR_TEMP; break;
		case _STEP_ALARM_HEAT_SINK_TEMP:			uiMask = 1<<_STEP_ALARM_MASK_HEAT_SINK_TEMP; break;
		case _STEP_ALARM_MY_CHANNEL:				uiMask = 1<<_STEP_ALARM_MASK_MY_CHANNEL; break;
		case _STEP_ALARM_OTHER_CHANNEL:				uiMask = 1<<_STEP_ALARM_MASK_OTHER_CHANNEL; break;
		case _STEP_ALARM_VOLTAGE_FAST_DOWN:			uiMask = 1<<_STEP_ALARM_MASK_VOLTAGE_FAST_DOWN; break;
		case _STEP_ALARM_VOLTAGE_FAST_UP:			uiMask = 1<<_STEP_ALARM_MASK_VOLTAGE_FAST_UP; break;
		case _STEP_ALARM_REST_OVP:					uiMask = 1<<_STEP_ALARM_MASK_REST_OVP; break;
		case _STEP_ALARM_REST_UVP:					uiMask = 1<<_STEP_ALARM_MASK_REST_UVP; break;
		case _STEP_ALARM_CELL_TEMP_ERROR:			uiMask = 1<<_STEP_ALARM_MASK_CELL_TEMP_ERROR; break;	//LJK 2024.07.01		
		#ifdef MODIFY_ALARM_CONDITION
		case _STEP_ALARM_REVERSE_CELL:			    uiMask = 1<<_STEP_ALARM_MASK_REVERSE_CELL; break;	    //20250908 jschoi
		#endif
		default: return usR;
	}
	uiMask &= ~m_uiAlarmMask[ucCh];
	if ( uiMask==0 )
		return 0;
	return usR;
}

#ifdef SUPPORT_PROTECTION_CONDITION
void AlarmParameterSet( unsigned char ucCh )
{
	#define	_ALARM_RESISTER_TOLERANCE	( MAX_SYSTEM_VOLTAGE / CHARGE_MINIMUM_CURREN * 0.02)	//	2% (윤책임)
	
	if ( m_ucNowMode[ucCh] == _MODE_2POINT_PULSE_DISCHARGE ||  m_ucNowMode[ucCh] == _MODE_10POINT_PULSE_DISCHARGE )
	{
		m_fCvVoltageLow[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent-0.003f;
		m_fCvVoltageHigh[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent+0.003f;
	}
	else
	{
		m_fCvVoltageLow[ucCh] = m_pSeqNow[ucCh]->fSettingVoltage-0.003f;
		m_fCvVoltageHigh[ucCh] = m_pSeqNow[ucCh]->fSettingVoltage+0.003f;
	}	
	
	m_bVoltageDeltaError[0][ucCh] = m_bVoltageDeltaError[1][ucCh] = FALSE;
	
	switch( m_pSeqNow[ucCh]->ucState )
	{
		case _STATE_REST:
			m_ucREST_MODE[ucCh] = m_pSeqNow[ucCh]->ucPulseMax;
			if( m_ucREST_MODE[ucCh] == _REST_TOLERANCE_MODE) 
			{		
				m_fAlarmTol_VoltageHigh[ucCh] = m_fVoltage1Sec[ucCh] + m_pSeqNow[ucCh]->StepEndCondition.fChargeCurrent;	//Upper Limit
				m_fAlarmTol_VoltageLow[ucCh] = m_fVoltage1Sec[ucCh] - m_pSeqNow[ucCh]->StepEndCondition.fDisChargeCurrent;	//Lower Limit
				
			}else if( m_ucREST_MODE[ucCh] == _REST_ABSOLUTE_MODE) 
			{				
				m_fAlarmVoltageHigh[ucCh] = m_pSeqNow[ucCh]->StepEndCondition.fChargeVoltage;	//Upper Limit
				m_fAlarmVoltageLow[ucCh] = m_pSeqNow[ucCh]->StepEndCondition.fDisChargeVoltage;	//Lower Limit
				
			}else if(m_ucREST_MODE[ucCh] == _REST_TOL_ABS_MODE ) 
			{
				m_fAlarmTol_VoltageHigh[ucCh] = m_fVoltage1Sec[ucCh] + m_pSeqNow[ucCh]->StepEndCondition.fChargeCurrent;	//Upper Limit
				m_fAlarmTol_VoltageLow[ucCh] = m_fVoltage1Sec[ucCh] - m_pSeqNow[ucCh]->StepEndCondition.fDisChargeCurrent;	//Lower Limit
				
				m_fAlarmVoltageHigh[ucCh] = m_pSeqNow[ucCh]->StepEndCondition.fChargeVoltage;	//Upper Limit
				m_fAlarmVoltageLow[ucCh] = m_pSeqNow[ucCh]->StepEndCondition.fDisChargeVoltage;	//Lower Limit
								
			}else if( m_ucREST_MODE[ucCh] == _REST_NONE_MODE ) 
			{
				m_fAlarmVoltageHigh[ucCh] = MAX_SYSTEM_VOLTAGE;
				m_fAlarmVoltageLow[ucCh] = MINIMUM_VOLTAGE;
			}						
			break;
			
		case _STATE_CHARGE:
			m_fAlarmVoltageLow[ucCh] = 0.001f;
			m_fAlarmVoltageHigh[ucCh] = m_pSeqNow[ucCh]->fSettingVoltage + m_pProtectionType[ucCh].fVoltUpperOffset;
			
			switch( m_pSeqNow[ucCh]->ucMode )
			{
				case _MODE_CC:
				case _MODE_CCCV:
					m_fErrorCurrentLow[ucCh]  = m_pSeqNow[ucCh]->fSettingCurrent - m_pProtectionType[ucCh].fCurrLowerOffset;
					m_fErrorCurrentHigh[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent + m_pProtectionType[ucCh].fCurrUpperOffset;
					break;
				case _MODE_CPCV:
					m_fErrorCurrentLow[ucCh] = 0.0f;
					m_fErrorCurrentHigh[ucCh] = MAX_SYSTEM_CURRENT + m_pProtectionType[ucCh].fCurrUpperOffset;
					m_fAlarmWattLow[ucCh]  = m_pSeqNow[ucCh]->fSettingCurrent * (1.0f - (m_pProtectionType[ucCh].fPowerLowerOffset / 100.0f));   //0.98f
					m_fAlarmWattHigh[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent * (1.0f + (m_pProtectionType[ucCh].fPowerUpperOffset / 100.0f));	// + 5.0f;	// 5Watt, 2% // -2% //LJK 2025.04.24
					break;
				case _MODE_CRCV:
					m_fErrorCurrentLow[ucCh] = 0.0f;
					m_fErrorCurrentHigh[ucCh] = MAX_SYSTEM_CURRENT + m_pProtectionType[ucCh].fCurrUpperOffset;
					m_fAlarmResisterLow[ucCh]  = m_pSeqNow[ucCh]->fSettingCurrent * (1.0f - (m_pProtectionType[ucCh].fPowerLowerOffset / 100.0f));	// - _ALARM_RESISTER_TOLERANCE; // -2% //LJK 2025.04.24
					m_fAlarmResisterHigh[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent * (1.0f + (m_pProtectionType[ucCh].fPowerUpperOffset / 100.0f));	// + _ALARM_RESISTER_TOLERANCE; // +2% //LJK 2025.04.24
					break;
				default:
					m_fErrorCurrentLow[ucCh] = 0.0f;
					m_fErrorCurrentHigh[ucCh] = MAX_SYSTEM_CURRENT + m_pProtectionType[ucCh].fCurrUpperOffset;
					break;
			}
			break;
		case _STATE_DISCHARGE:
			m_fAlarmVoltageLow[ucCh] = m_pSeqNow[ucCh]->fSettingVoltage - 0.01f;
			//m_fAlarmVoltageHigh[ucCh] = 5.003f;
            m_fAlarmVoltageHigh[ucCh] = 4.500f; //20250909 
			
			switch( m_pSeqNow[ucCh]->ucMode )
			{
				case _MODE_CC:
				case _MODE_CCCV:
					m_fErrorCurrentLow[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent - m_pProtectionType[ucCh].fCurrLowerOffset;
					m_fErrorCurrentHigh[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent + m_pProtectionType[ucCh].fCurrUpperOffset;
					break;
				case _MODE_CPCV:
					m_fErrorCurrentLow[ucCh] = 0.0f;
					m_fErrorCurrentHigh[ucCh] = MAX_SYSTEM_CURRENT + m_pProtectionType[ucCh].fCurrUpperOffset;
					m_fAlarmWattLow[ucCh]  = m_pSeqNow[ucCh]->fSettingCurrent * (1.0f - (m_pProtectionType[ucCh].fPowerLowerOffset / 100.0f));	// - 5.0f;	// 5Watt, 2% // -2% //LJK 2025.04.24
					m_fAlarmWattHigh[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent * (1.0f + (m_pProtectionType[ucCh].fPowerUpperOffset / 100.0f));	// + 5.0f;	// 5Watt, 2% // -2% //LJK 2025.04.24
					break;
				case _MODE_CRCV:
					m_fErrorCurrentLow[ucCh] = 0.0f;
					m_fErrorCurrentHigh[ucCh] = MAX_SYSTEM_CURRENT + m_pProtectionType[ucCh].fCurrUpperOffset;
					m_fAlarmResisterLow[ucCh]  = m_pSeqNow[ucCh]->fSettingCurrent * (1.0f - (m_pProtectionType[ucCh].fPowerLowerOffset / 100.0f));	// - _ALARM_RESISTER_TOLERANCE; // -2% //LJK 2025.04.24
					m_fAlarmResisterHigh[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent * (1.0f + (m_pProtectionType[ucCh].fPowerUpperOffset / 100.0f));	// + _ALARM_RESISTER_TOLERANCE; // +2% //LJK 2025.04.24
					break;
				default:
					m_fErrorCurrentLow[ucCh] = 0.0f;
					m_fErrorCurrentHigh[ucCh] = MAX_SYSTEM_CURRENT + m_pProtectionType[ucCh].fCurrUpperOffset;
					break;
			}
			break;
		case _STATE_PULSE:
			if ( m_ucNowMode[ucCh] == _MODE_2POINT_PULSE_CHARGE || m_ucNowMode[ucCh] == _MODE_10POINT_PULSE_CHARGE )
			{
				m_fAlarmVoltageLow[ucCh] = -0.003f;
				m_fAlarmVoltageHigh[ucCh] = m_pSeqNow[ucCh]->fSettingVoltage + m_pProtectionType[ucCh].fVoltUpperOffset;
			}
			else
			{
				m_fAlarmVoltageLow[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent-0.01f;
				m_fAlarmVoltageHigh[ucCh] = 5.003f;
			}
			break;
			//LJK 2023.05.27 _STATE_BALANCE 모드 추가
		case _STATE_BALANCE:
			if( m_ucNowRestChargeDisCharge[ucCh] == _NOW_STATE_CHARGE )
			{
				m_fAlarmVoltageLow[ucCh] = 0.001f;
				m_fAlarmVoltageHigh[ucCh] = m_pSeqNow[ucCh]->fSettingVoltage + m_pProtectionType[ucCh].fVoltUpperOffset;
				switch( m_pSeqNow[ucCh]->ucMode )
				{
					case _MODE_CC:
					case _MODE_CCCV:
						m_fErrorCurrentLow[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent - m_pProtectionType[ucCh].fCurrLowerOffset;
						m_fErrorCurrentHigh[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent + m_pProtectionType[ucCh].fCurrUpperOffset;
						break;
					case _MODE_CPCV:
						m_fErrorCurrentLow[ucCh] = 0.0f;
						m_fErrorCurrentHigh[ucCh] = MAX_SYSTEM_CURRENT + m_pProtectionType[ucCh].fCurrUpperOffset;
						m_fAlarmWattLow[ucCh]  = m_pSeqNow[ucCh]->fSettingCurrent * (1.0f - (m_pProtectionType[ucCh].fPowerLowerOffset / 100.0f));	// - 5.0f;	// 5Watt, 2% // -2% //LJK 2025.04.24
						m_fAlarmWattHigh[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent * (1.0f + (m_pProtectionType[ucCh].fPowerUpperOffset / 100.0f));	// + 5.0f;	// 5Watt, 2% // -2% //LJK 2025.04.24
						break;
					case _MODE_CRCV:
						m_fErrorCurrentLow[ucCh] = 0.0f;
						m_fErrorCurrentHigh[ucCh] = MAX_SYSTEM_CURRENT + m_pProtectionType[ucCh].fCurrUpperOffset;
						m_fAlarmResisterLow[ucCh]  = m_pSeqNow[ucCh]->fSettingCurrent * (1.0f - (m_pProtectionType[ucCh].fPowerLowerOffset / 100.0f));	// - _ALARM_RESISTER_TOLERANCE; // -2% //LJK 2025.04.24
						m_fAlarmResisterHigh[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent * (1.0f + (m_pProtectionType[ucCh].fPowerUpperOffset / 100.0f));	// + _ALARM_RESISTER_TOLERANCE; // +2% //LJK 2025.04.24
						break;
					default:
						m_fErrorCurrentLow[ucCh] = 0.0f;
						m_fErrorCurrentHigh[ucCh] = MAX_SYSTEM_CURRENT + m_pProtectionType[ucCh].fCurrUpperOffset;
						break;
				}
			}			
			else
			{
				m_fAlarmVoltageLow[ucCh] = m_pSeqNow[ucCh]->fSettingVoltage-0.01f;
				m_fAlarmVoltageHigh[ucCh] = 5.003f;
				switch( m_pSeqNow[ucCh]->ucMode )
				{
					case _MODE_CC:
					case _MODE_CCCV:
						m_fErrorCurrentLow[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent - m_pProtectionType[ucCh].fCurrLowerOffset;
						m_fErrorCurrentHigh[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent + m_pProtectionType[ucCh].fCurrUpperOffset;
						break;
					case _MODE_CPCV:
						m_fErrorCurrentLow[ucCh] = 0.0f;
						m_fErrorCurrentHigh[ucCh] = MAX_SYSTEM_CURRENT + m_pProtectionType[ucCh].fCurrUpperOffset;
						m_fAlarmWattLow[ucCh]  = m_pSeqNow[ucCh]->fSettingCurrent * (1.0f - (m_pProtectionType[ucCh].fPowerLowerOffset / 100.0f));	// 5Watt, 2%
						m_fAlarmWattHigh[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent + (1.0f + (m_pProtectionType[ucCh].fPowerUpperOffset / 100.0f));	// 5Watt, 2%
						break;
					case _MODE_CRCV:
						m_fErrorCurrentLow[ucCh] = 0.0f;
						m_fErrorCurrentHigh[ucCh] = MAX_SYSTEM_CURRENT + m_pProtectionType[ucCh].fCurrUpperOffset;
						m_fAlarmResisterLow[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent - _ALARM_RESISTER_TOLERANCE;
						m_fAlarmResisterHigh[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent + _ALARM_RESISTER_TOLERANCE;
						break;
					default:
						m_fErrorCurrentLow[ucCh] = 0.0f;
						m_fErrorCurrentHigh[ucCh] = MAX_SYSTEM_CURRENT + m_pProtectionType[ucCh].fCurrUpperOffset;
						break;
				}				
			}			
			break;
		default:
			m_fAlarmVoltageLow[ucCh]  = 0.001f;
			m_fAlarmVoltageHigh[ucCh] = 5.003f;
			m_fErrorCurrentLow[ucCh]  = 0.000f;
			m_fErrorCurrentHigh[ucCh] = MAX_SYSTEM_CURRENT + m_pProtectionType[ucCh].fCurrUpperOffset;
			break;
	}
	
	m_bAlarmParameterSet[ucCh] = true;
}

#else
void AlarmParameterSet( unsigned char ucCh )
{
	//#define		_ALARM_RESISTER_TOLERANCE	(MAX_SYSTEM_VOLTAGE / MAX_SYSTEM_CURRENT * 0.02)	//	2%
	#define		_ALARM_RESISTER_TOLERANCE	( MAX_SYSTEM_VOLTAGE / CHARGE_MINIMUM_CURREN * 0.02)	//	2% (윤책임)
	if ( m_ucNowMode[ucCh] == _MODE_2POINT_PULSE_DISCHARGE ||  m_ucNowMode[ucCh] == _MODE_10POINT_PULSE_DISCHARGE )
	{
		m_fCvVoltageLow[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent-0.003f;
		m_fCvVoltageHigh[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent+0.003f;
	}
	else
	{
		m_fCvVoltageLow[ucCh] = m_pSeqNow[ucCh]->fSettingVoltage-0.003f;
		m_fCvVoltageHigh[ucCh] = m_pSeqNow[ucCh]->fSettingVoltage+0.003f;
	}	
	m_bVoltageDeltaError[0][ucCh] = m_bVoltageDeltaError[1][ucCh] = FALSE;
	switch( m_pSeqNow[ucCh]->ucState )
	{
		case _STATE_REST:// Rest시는 첫 Cell이 기준
			//2024.09.02 Rest Upper/Lower Limit 조건 상대값/절대값
			/*
			if(m_pSeqNow[ucCh]->ucPulseMax == _REST_TOLERANCE_MODE)
			{
				m_fAlarmVoltageHigh[ucCh] = m_fVoltage1Sec[ucCh] + m_pSeqNow[ucCh]->fSettingVoltage;	//Upper Limit
				m_fAlarmVoltageLow[ucCh] = m_fVoltage1Sec[ucCh] - m_pSeqNow[ucCh]->fSettingCurrent;		//Lower Limit
			}else if(m_pSeqNow[ucCh]->ucPulseMax == _REST_ABSOLUTE_MODE)
			{
				m_fAlarmVoltageHigh[ucCh] = m_pSeqNow[ucCh]->fSettingVoltage;	//Upper Limit
				m_fAlarmVoltageLow[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent;	//Lower Limit
			}else{
				m_fAlarmVoltageLow[ucCh] = m_fVoltage1Sec[ucCh]-0.5f;
				m_fAlarmVoltageHigh[ucCh] = m_fVoltage1Sec[ucCh]+0.5f;
			}
			*/
			m_ucREST_MODE[ucCh] = m_pSeqNow[ucCh]->ucPulseMax;
			if( m_ucREST_MODE[ucCh] == _REST_TOLERANCE_MODE) 
			{		
				m_fAlarmTol_VoltageHigh[ucCh] = m_fVoltage1Sec[ucCh] + m_pSeqNow[ucCh]->StepEndCondition.fChargeCurrent;	//Upper Limit
				m_fAlarmTol_VoltageLow[ucCh] = m_fVoltage1Sec[ucCh] - m_pSeqNow[ucCh]->StepEndCondition.fDisChargeCurrent;	//Lower Limit
				
			}else if( m_ucREST_MODE[ucCh] == _REST_ABSOLUTE_MODE) 
			{				
				m_fAlarmVoltageHigh[ucCh] = m_pSeqNow[ucCh]->StepEndCondition.fChargeVoltage;	//Upper Limit
				m_fAlarmVoltageLow[ucCh] = m_pSeqNow[ucCh]->StepEndCondition.fDisChargeVoltage;	//Lower Limit
				
			}else if(m_ucREST_MODE[ucCh] == _REST_TOL_ABS_MODE ) 
			{
				m_fAlarmTol_VoltageHigh[ucCh] = m_fVoltage1Sec[ucCh] + m_pSeqNow[ucCh]->StepEndCondition.fChargeCurrent;	//Upper Limit
				m_fAlarmTol_VoltageLow[ucCh] = m_fVoltage1Sec[ucCh] - m_pSeqNow[ucCh]->StepEndCondition.fDisChargeCurrent;	//Lower Limit
				
				m_fAlarmVoltageHigh[ucCh] = m_pSeqNow[ucCh]->StepEndCondition.fChargeVoltage;	//Upper Limit
				m_fAlarmVoltageLow[ucCh] = m_pSeqNow[ucCh]->StepEndCondition.fDisChargeVoltage;	//Lower Limit
								
			}else if( m_ucREST_MODE[ucCh] == _REST_NONE_MODE ) 
			{
				m_fAlarmVoltageHigh[ucCh] = MAX_SYSTEM_VOLTAGE;
				m_fAlarmVoltageLow[ucCh] = MINIMUM_VOLTAGE;
			}
						
			break;
		case _STATE_CHARGE:
			m_fAlarmVoltageLow[ucCh] = 0.001f;
			m_fAlarmVoltageHigh[ucCh] = m_pSeqNow[ucCh]->fSettingVoltage+0.01f;
			switch( m_pSeqNow[ucCh]->ucMode )
			{
				case _MODE_CC:
				case _MODE_CCCV:
					m_fErrorCurrentLow[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent-0.125f;
					m_fErrorCurrentHigh[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent+0.125f;
					break;
				case _MODE_CPCV:
					m_fErrorCurrentLow[ucCh] = 0.0f;
					m_fErrorCurrentHigh[ucCh] = MAX_SYSTEM_CURRENT + 0.125f;
					m_fAlarmWattLow[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent * 0.98f;	// - 5.0f;	// 5Watt, 2% // -2% //LJK 2025.04.24
					m_fAlarmWattHigh[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent * 1.02f;	// + 5.0f;	// 5Watt, 2% // -2% //LJK 2025.04.24
					break;
				case _MODE_CRCV:
					m_fErrorCurrentLow[ucCh] = 0.0f;
					m_fErrorCurrentHigh[ucCh] = MAX_SYSTEM_CURRENT + 0.125f;
					m_fAlarmResisterLow[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent * 0.98f;	// - _ALARM_RESISTER_TOLERANCE; // -2% //LJK 2025.04.24
					m_fAlarmResisterHigh[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent * 1.02f;	// + _ALARM_RESISTER_TOLERANCE; // +2% //LJK 2025.04.24
					break;
				default:
					m_fErrorCurrentLow[ucCh] = 0.0f;
					m_fErrorCurrentHigh[ucCh] = MAX_SYSTEM_CURRENT + 0.125f;
					break;
			}
			break;
		case _STATE_DISCHARGE:
			m_fAlarmVoltageLow[ucCh] = m_pSeqNow[ucCh]->fSettingVoltage-0.01f;
			m_fAlarmVoltageHigh[ucCh] = 5.003f;
			switch( m_pSeqNow[ucCh]->ucMode )
			{
				case _MODE_CC:
				case _MODE_CCCV:
					m_fErrorCurrentLow[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent-0.125f;
					m_fErrorCurrentHigh[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent+0.125f;
					break;
				case _MODE_CPCV:
					m_fErrorCurrentLow[ucCh] = 0.0f;
					m_fErrorCurrentHigh[ucCh] = MAX_SYSTEM_CURRENT + 0.125f;
					m_fAlarmWattLow[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent * 0.98f;	// - 5.0f;	// 5Watt, 2% // -2% //LJK 2025.04.24
					m_fAlarmWattHigh[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent * 1.02f;	// + 5.0f;	// 5Watt, 2% // -2% //LJK 2025.04.24
					break;
				case _MODE_CRCV:
					m_fErrorCurrentLow[ucCh] = 0.0f;
					m_fErrorCurrentHigh[ucCh] = MAX_SYSTEM_CURRENT + 0.125f;
					m_fAlarmResisterLow[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent * 0.98f;	// - _ALARM_RESISTER_TOLERANCE; // -2% //LJK 2025.04.24
					m_fAlarmResisterHigh[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent * 1.02f;	// + _ALARM_RESISTER_TOLERANCE; // +2% //LJK 2025.04.24
					break;
				default:
					m_fErrorCurrentLow[ucCh] = 0.0f;
					m_fErrorCurrentHigh[ucCh] = MAX_SYSTEM_CURRENT + 0.125f;
					break;
			}
			break;
		case _STATE_PULSE:
			if ( m_ucNowMode[ucCh] == _MODE_2POINT_PULSE_CHARGE || m_ucNowMode[ucCh] == _MODE_10POINT_PULSE_CHARGE )
			{
				m_fAlarmVoltageLow[ucCh] = -0.003f;
				m_fAlarmVoltageHigh[ucCh] = m_pSeqNow[ucCh]->fSettingVoltage+0.01f;
			}
			else
			{
				m_fAlarmVoltageLow[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent-0.01f;
				m_fAlarmVoltageHigh[ucCh] = 5.003f;
			}
			break;
			//LJK 2023.05.27 _STATE_BALANCE 모드 추가
		case _STATE_BALANCE:
			if( m_ucNowRestChargeDisCharge[ucCh] == _NOW_STATE_CHARGE )
			{
				m_fAlarmVoltageLow[ucCh] = 0.001f;
				m_fAlarmVoltageHigh[ucCh] = m_pSeqNow[ucCh]->fSettingVoltage+0.01f;
				switch( m_pSeqNow[ucCh]->ucMode )
				{
					case _MODE_CC:
					case _MODE_CCCV:
						m_fErrorCurrentLow[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent-0.125f;
						m_fErrorCurrentHigh[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent+0.125f;
						break;
					case _MODE_CPCV:
						m_fErrorCurrentLow[ucCh] = 0.0f;
						m_fErrorCurrentHigh[ucCh] = MAX_SYSTEM_CURRENT + 0.125f;
						m_fAlarmWattLow[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent * 0.98f;	// - 5.0f;	// 5Watt, 2% // -2% //LJK 2025.04.24
						m_fAlarmWattHigh[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent * 1.02f;	// + 5.0f;	// 5Watt, 2% // -2% //LJK 2025.04.24
						break;
					case _MODE_CRCV:
						m_fErrorCurrentLow[ucCh] = 0.0f;
						m_fErrorCurrentHigh[ucCh] = MAX_SYSTEM_CURRENT + 0.125f;
						m_fAlarmResisterLow[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent * 0.98f;	// - _ALARM_RESISTER_TOLERANCE; // -2% //LJK 2025.04.24
						m_fAlarmResisterHigh[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent * 1.02f;	// + _ALARM_RESISTER_TOLERANCE; // +2% //LJK 2025.04.24
						break;
					default:
						m_fErrorCurrentLow[ucCh] = 0.0f;
						m_fErrorCurrentHigh[ucCh] = MAX_SYSTEM_CURRENT + 0.125f;
						break;
				}
			}			
			else
			{
				m_fAlarmVoltageLow[ucCh] = m_pSeqNow[ucCh]->fSettingVoltage-0.01f;
				m_fAlarmVoltageHigh[ucCh] = 5.003f;
				switch( m_pSeqNow[ucCh]->ucMode )
				{
					case _MODE_CC:
					case _MODE_CCCV:
						m_fErrorCurrentLow[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent-0.125f;
						m_fErrorCurrentHigh[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent+0.125f;
						break;
					case _MODE_CPCV:
						m_fErrorCurrentLow[ucCh] = 0.0f;
						m_fErrorCurrentHigh[ucCh] = MAX_SYSTEM_CURRENT + 0.125f;
						m_fAlarmWattLow[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent - 5.0f;	// 5Watt, 2%
						m_fAlarmWattHigh[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent + 5.0f;	// 5Watt, 2%
						break;
					case _MODE_CRCV:
						m_fErrorCurrentLow[ucCh] = 0.0f;
						m_fErrorCurrentHigh[ucCh] = MAX_SYSTEM_CURRENT + 0.125f;
						m_fAlarmResisterLow[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent - _ALARM_RESISTER_TOLERANCE;
						m_fAlarmResisterHigh[ucCh] = m_pSeqNow[ucCh]->fSettingCurrent + _ALARM_RESISTER_TOLERANCE;
						break;
					default:
						m_fErrorCurrentLow[ucCh] = 0.0f;
						m_fErrorCurrentHigh[ucCh] = MAX_SYSTEM_CURRENT + 0.125f;
						break;
				}				
			}			
			break;
		default:
			m_fAlarmVoltageLow[ucCh] = 0.001f;
			m_fAlarmVoltageHigh[ucCh] = 5.003f;
			m_fErrorCurrentLow[ucCh] = 0.000f;
			m_fErrorCurrentHigh[ucCh] = MAX_SYSTEM_CURRENT + 0.125f;
			break;
	}
	
	m_bAlarmParameterSet[ucCh] = true;
}
#endif

unsigned short AlarmCheckProcess( unsigned char ucCh )
{
	float fCheckVoltage = m_fVoltage[ucCh];
	float fCheckCurrent = m_fCurrent[ucCh];
	float fCheckWatt = m_fWatt[ucCh];
	float fCheckResister = m_fResister[ucCh];
	float fVoltagePer10Hour;

	//m_bChagedOrDisChargedNow[ucCh] = ( m_ucNowState[ucCh] == _NOW_STATE_CHARGE || m_ucNowState[ucCh] == _NOW_STATE_DISCHARGE ) ? TRUE : FALSE;
	if( fCheckCurrent <0.0)
		fCheckCurrent = -fCheckCurrent;
	
	if( fCheckWatt <0.0)
		fCheckWatt = -fCheckWatt;
	
	if (fCheckResister < 0.0)
		fCheckResister = -fCheckResister;
	
	if ( fCheckVoltage>(float)5.003 )
	{
		if ( ++m_u16SafeErrorNumbers[ucCh][RTY_VSENSOR_OPEN_ERROR]>=10 )	// 10ms
			return _STEP_ALARM_VSENSOR_OPEN;
	}
	else
		m_u16SafeErrorNumbers[ucCh][RTY_VSENSOR_OPEN_ERROR] = 0;
		
	//if ( fCheckVoltage<(float)-0.005)
	#ifdef MODIFY_ALARM_CONDITION
	if(fCheckVoltage < (float)-5.003)
	#else
	if ( fCheckVoltage<(float)-0.3 ) //LJK 2024.04.09 req-YBG
	#endif
	{
		if ( ++m_u16SafeErrorNumbers[ucCh][RTY_VOLTAGE_MINUS_ERROR]>=10 )	// 10ms
		{
	        #ifdef MODIFY_ALARM_CONDITION
			return _STEP_ALARM_VSENSOR_OPEN;
	        #else
			return _STEP_ALARM_MINUS_VOLTAGE;
			#endif
		}
	}
	else
	{
		m_u16SafeErrorNumbers[ucCh][RTY_VOLTAGE_MINUS_ERROR] = 0;
	}
	
	//Rest 2024.09.02 조건 변경
	if ( m_pSeqNow[ucCh]->ucState == _STATE_REST )
	{
		//LJK 2023.09.09 Rest 조건 추가
		if( m_ucREST_MODE[ucCh] == _REST_ABSOLUTE_MODE )
		{
			if ( fCheckVoltage<m_fAlarmVoltageLow[ucCh] )
			{
				if ( ++m_u16SafeErrorNumbers[ucCh][RTY_REST_UVP]>=2500 )	// 2500ms
					return _STEP_ALARM_REST_UVP;
			}
			else
				m_u16SafeErrorNumbers[ucCh][RTY_REST_UVP] = 0;
			if ( fCheckVoltage>m_fAlarmVoltageHigh[ucCh] )
			{
				if ( ++m_u16SafeErrorNumbers[ucCh][RTY_REST_OVP]>=2500 )	// 2500ms
					return _STEP_ALARM_REST_OVP;
			}
			else
				m_u16SafeErrorNumbers[ucCh][RTY_REST_OVP] = 0;			
		}			
		else if( m_ucREST_MODE[ucCh] == _REST_TOLERANCE_MODE )
		{
			if ( fCheckVoltage<m_fAlarmTol_VoltageLow[ucCh] )
			{
				if ( ++m_u16SafeErrorNumbers[ucCh][RTY_REST_UVP]>=2500 )	// 2500ms
					return _STEP_ALARM_REST_UVP;
			}
			else
				m_u16SafeErrorNumbers[ucCh][RTY_REST_UVP] = 0;
			if ( fCheckVoltage>m_fAlarmTol_VoltageHigh[ucCh] )
			{
				if ( ++m_u16SafeErrorNumbers[ucCh][RTY_REST_OVP]>=2500 )	// 2500ms
					return _STEP_ALARM_REST_OVP;
			}
			else
				m_u16SafeErrorNumbers[ucCh][RTY_REST_OVP] = 0;
		}
		else if( m_ucREST_MODE[ucCh] == _REST_TOL_ABS_MODE )
		{
			if ( fCheckVoltage<m_fAlarmVoltageLow[ucCh] || fCheckVoltage<m_fAlarmTol_VoltageLow[ucCh] )
			{
				if ( ++m_u16SafeErrorNumbers[ucCh][RTY_REST_UVP]>=2500 )	// 2500ms
					return _STEP_ALARM_REST_UVP;
			}
			else
				m_u16SafeErrorNumbers[ucCh][RTY_REST_UVP] = 0;
			if ( fCheckVoltage>m_fAlarmVoltageHigh[ucCh] || fCheckVoltage>m_fAlarmTol_VoltageHigh[ucCh] )
			{
				if ( ++m_u16SafeErrorNumbers[ucCh][RTY_REST_OVP]>=2500 )	// 2500ms
					return _STEP_ALARM_REST_OVP;
			}
			else
				m_u16SafeErrorNumbers[ucCh][RTY_REST_OVP] = 0;				
		}
	}
	else
	{
		if ( fCheckVoltage<m_fAlarmVoltageLow[ucCh] )
		{
			if ( ++m_u16SafeErrorNumbers[ucCh][RTY_UVP]>=2500 )	// 2500ms
				return _STEP_ALARM_UVP;
		}
		else
			m_u16SafeErrorNumbers[ucCh][RTY_UVP] = 0;
		
		if ( fCheckVoltage>m_fAlarmVoltageHigh[ucCh] )
		{
			if ( ++m_u16SafeErrorNumbers[ucCh][RTY_OVP]>=2500 )	// 2500ms
				return _STEP_ALARM_OVP;
		}
		else
			m_u16SafeErrorNumbers[ucCh][RTY_OVP] = 0;
	}
		
	if ( !m_bStepRunning[ucCh] )
		return 0;
	
	//LJK 2024.07.01
	#ifdef SUPPORT_PROTECTION_CONDITION
	float fCellTempLimit = m_pProtectionType[ucCh].fCellTemp * 10.0f;
	
	if (fCellTempLimit <= m_shCurrentCellTemp[ucCh])
	{
		return _STEP_ALARM_CELL_TEMP_ERROR;
	}
	#else
	if (m_shMaxCellTemp[ucCh] <= m_shCurrentCellTemp[ucCh])
		return _STEP_ALARM_CELL_TEMP_ERROR;
    #endif
				
	/*	LJK 2023.05.18 위치 아래로 이동 		
	if ( m_bAirTemperaturError )
		return _STEP_ALARM_AIR_TEMP;
	if ( m_bHeatSinkTemperaturError )
		return _STEP_ALARM_HEAT_SINK_TEMP;
	if ( m_bVoltageDeltaError[0][ucCh] )
		return _STEP_ALARM_VOLTAGE_FAST_DOWN;
	if ( m_bVoltageDeltaError[1][ucCh] )
		return _STEP_ALARM_VOLTAGE_FAST_UP;
	*/
	if ( !m_bStepRunning[ucCh] )
		return 0;
		
	#define _15HourToMs 54000000.0f	//(float)(3600*1000*15)
	
	if ( m_ucNowRestChargeDisCharge[ucCh] == _NOW_STATE_CHARGE )
	{
		fVoltagePer10Hour = (m_fVoltage1Sec[ucCh]-m_fStepStartVoltage[ucCh])/((float)m_uiStepTimeNow[ucCh]/_15HourToMs);	// 15시간시
		if ( m_ucNowMode[ucCh] != _MODE_2POINT_PULSE_DISCHARGE && m_ucNowMode[ucCh] != _MODE_10POINT_PULSE_DISCHARGE )
		{
			if ( fVoltagePer10Hour<(float)(4.2f-2.5f) )
			{
				if ( ++m_u16SafeErrorNumbers[ucCh][RTY_VOLTAGE_FIX_ERROR]>=5000 )	// 5초 연속
					return _STEP_ALARM_VOLTAGE_FIX_ERROR;
			}
			else
				m_u16SafeErrorNumbers[ucCh][RTY_VOLTAGE_FIX_ERROR] = 0;
		}
	}
	else
	{
		fVoltagePer10Hour = (m_fStepStartVoltage[ucCh]-m_fVoltage1Sec[ucCh])/((float)m_uiStepTimeNow[ucCh]/_15HourToMs);	// 15시간시
		if ( m_ucNowMode[ucCh] != _MODE_2POINT_PULSE_CHARGE && m_ucNowMode[ucCh] != _MODE_10POINT_PULSE_CHARGE )
		{
			if ( fVoltagePer10Hour<(float)(4.2-2.5) )
			{
				if ( ++m_u16SafeErrorNumbers[ucCh][RTY_VOLTAGE_FIX_ERROR]>=5000 )	// 5초 연속
					return _STEP_ALARM_VOLTAGE_FIX_ERROR;
			}
			else
				m_u16SafeErrorNumbers[ucCh][RTY_VOLTAGE_FIX_ERROR] = 0;
		}
	}
	//STATE_BALANCE 일때 UCP, OCP, UWP, OWP, URP ORP 안되는 버그
	//LJK 2023.05.22 m_pSeqNow[ucCh]->ucState == _STATE_BALANCE 조건추가
	if ( m_pSeqNow[ucCh]->ucState == _STATE_CHARGE || m_pSeqNow[ucCh]->ucState == _STATE_DISCHARGE || m_pSeqNow[ucCh]->ucState == _STATE_BALANCE)
	{
		if ( fCheckCurrent<m_fErrorCurrentLow[ucCh] && m_uiStepCV_TimeNow[ucCh]<100 )
		{
			if ( ++m_u16SafeErrorNumbers[ucCh][RTY_UCP]>=5000 )	// 1000ms // 5000ms LJK 2024.02.03
				return _STEP_ALARM_UCP;
		}
		else
			m_u16SafeErrorNumbers[ucCh][RTY_UCP] = 0;
			
		if ( fCheckCurrent>m_fErrorCurrentHigh[ucCh] )
		{
			if ( ++m_u16SafeErrorNumbers[ucCh][RTY_OCP]>=5000 )	// 1000ms // 5000ms LJK 2024.02.03
				return _STEP_ALARM_OCP;
		}
		else
			m_u16SafeErrorNumbers[ucCh][RTY_OCP] = 0;
			
		if ( m_pSeqNow[ucCh]->ucMode == _MODE_CPCV )
		{
			if ( fCheckWatt<m_fAlarmWattLow[ucCh] && m_uiStepCV_TimeNow[ucCh]<100 )
			{
				if ( ++m_u16SafeErrorNumbers[ucCh][RTY_UWP]>=1000 )			// 1000ms
					return _STEP_ALARM_UWP;
			}
			else
				m_u16SafeErrorNumbers[ucCh][RTY_UWP] = 0;
		
			if ( fCheckWatt>m_fAlarmWattHigh[ucCh] )
			{
				if ( ++m_u16SafeErrorNumbers[ucCh][RTY_OWP]>=1000 )			// 1000ms
					return _STEP_ALARM_OWP;
			}
			else
				m_u16SafeErrorNumbers[ucCh][RTY_OWP] = 0;			
		}
	
		if ( m_pSeqNow[ucCh]->ucMode == _MODE_CRCV )
		{
			if ( fCheckResister < m_fAlarmResisterLow[ucCh] )
			{   
			
				#ifdef MODIFY_ALARM_CONDITION
				if( ++m_u16SafeErrorNumbers[ucCh][RTY_URP] > 1000 )
					return _STEP_ALARM_URP;	
				#else
				//if ( ++m_u16SafeErrorNumbers[ucCh][RTY_URP]>=1000 && m_uiStepCV_TimeNow[ucCh]<4 )			// 1000ms
				//	return _STEP_ALARM_URP;				
				//if( m_uiStepTimeNow[ucCh] > 1000 && ++m_u16SafeErrorNumbers[ucCh][RTY_URP]>=1000 )	//LJK 2025.04.23
				if(++m_u16SafeErrorNumbers[ucCh][RTY_URP]>=1000 ))
					return _STEP_ALARM_URP;
				#endif
			}
			else
				m_u16SafeErrorNumbers[ucCh][RTY_URP] = 0;
				
			if ( fCheckResister > m_fAlarmResisterHigh[ucCh] )
			{
				#ifdef MODIFY_ALARM_CONDITION
				if( ++m_u16SafeErrorNumbers[ucCh][RTY_ORP] > 1000 )
					return _STEP_ALARM_ORP;	
				#else
				//if ( ++m_u16SafeErrorNumbers[ucCh][RTY_ORP]>=1000 && m_uiStepCV_TimeNow[ucCh]<4 )			// 1000ms
				//	return _STEP_ALARM_ORP;
				if( m_uiStepTimeNow[ucCh] > 1000 && ++m_u16SafeErrorNumbers[ucCh][RTY_ORP]>=1000 )	//LJK 2025.04.23
					return _STEP_ALARM_ORP;	
				#endif			
			}
			else
				m_u16SafeErrorNumbers[ucCh][RTY_ORP] = 0;
		}
		
		{
			static float fPreVoltage[_MAX_CHANNEL];

			#ifdef MODIFY_ALARM_CONDITION
			static unsigned int uiVoltageDeltaCheckTick[_MAX_CHANNEL] = {0,};

			if ( m_pSeqNow[ucCh]->ucState == _STATE_CHARGE || m_pSeqNow[ucCh]->ucState == _STATE_DISCHARGE)
			{
    			if((m_ui1msCounter - uiVoltageDeltaCheckTick[ucCh]) > 1000)    			
    			{
                    if ( m_pSeqNow[ucCh]->ucState == _STATE_CHARGE )
        			{
        				if ( fCheckVoltage < (fPreVoltage[ucCh] - 0.01f) ) //10mV offset
        				{   
        					if ( ++m_u16SafeErrorNumbers[ucCh][RTY_CHARGE_VOLTATGE_DOWN_ERROR] >= 3 )
        					{
        						return _STEP_ALARM_CHARGE_VOLTAGE_DOWN;
        					}
        				}
        				else
        				{
        					m_u16SafeErrorNumbers[ucCh][RTY_CHARGE_VOLTATGE_DOWN_ERROR] = 0;
        				}
        			}
                    else if ( m_pSeqNow[ucCh]->ucState == _STATE_DISCHARGE )
                    {
                        if ( fCheckVoltage > (fPreVoltage[ucCh] + 0.01f) ) //10mV offset
                        {
                            if ( ++m_u16SafeErrorNumbers[ucCh][RTY_DISCHARGE_VOLTATGE_UP_ERROR] >= 3 ) 
                            {
                                return _STEP_ALARM_DISCHARGE_VOLTAGE_UP;
                            }
                        }
                        else
                        {
                            m_u16SafeErrorNumbers[ucCh][RTY_DISCHARGE_VOLTATGE_UP_ERROR] = 0;
                        }
                    }
    			
                    uiVoltageDeltaCheckTick[ucCh] = m_ui1msCounter;
    			    //fPreVoltage[ucCh] = fCheckVoltage;
    			    fPreVoltage[ucCh] = m_fVoltage1Sec[ucCh];
    			}
    	    }
			#else
			// Charge시 10초 동안 지속적으로 V Down시 에러
			if ( m_pSeqNow[ucCh]->ucState == _STATE_CHARGE )
			{
				if ( fCheckVoltage <= fPreVoltage[ucCh] )
				{   
					if ( ++m_u16SafeErrorNumbers[ucCh][RTY_CHARGE_VOLTATGE_DOWN_ERROR]>=10000 )			// 10000ms
						return _STEP_ALARM_CHARGE_VOLTAGE_DOWN;
				}
				else
					m_u16SafeErrorNumbers[ucCh][RTY_CHARGE_VOLTATGE_DOWN_ERROR] = 0;
			}
			// DisCharge시 10초 동안 지속적으로 V Up시 에러
			if ( m_pSeqNow[ucCh]->ucState == _STATE_DISCHARGE )
			{
				if ( fCheckVoltage>=fPreVoltage[ucCh] )
				{
					if ( ++m_u16SafeErrorNumbers[ucCh][RTY_DISCHARGE_VOLTATGE_UP_ERROR]>=10000 )			// 10000ms
						return _STEP_ALARM_DISCHARGE_VOLTAGE_UP;
				}
				else
					m_u16SafeErrorNumbers[ucCh][RTY_DISCHARGE_VOLTATGE_UP_ERROR] = 0;
			}
			
			fPreVoltage[ucCh] = m_fVoltage[ucCh];
			#endif
		}
	}
	//2025.01.13 80도 -> 100도변경 (Req 유부사장님, 박희규 부장님)
	//2025.05.21 HeatSink 상시에서 Alarm Mask 영역으로 변경(bgyu 요청사항)
	#ifdef SUPPORT_PROTECTION_CONDITION	
	if (GetHeatSinkTemperaturError(ucCh) > m_pProtectionType[ucCh].fHeatSinkTemp) 
	{
		if ( ++m_u16SafeErrorNumbers[ucCh][RTY_HEAT_SINK_TEMP]>=5000 )
			return _STEP_ALARM_HEAT_SINK_TEMP;
	}
	else
	{
		m_u16SafeErrorNumbers[ucCh][RTY_HEAT_SINK_TEMP] = 0;
    }
    
	if (GetHeatSinkTemperaturError(ucCh+1) > m_pProtectionType[ucCh].fHeatSinkTemp)
	{
		if ( ++m_u16SafeErrorNumbers[ucCh+1][RTY_HEAT_SINK_TEMP]>=5000 )
			return _STEP_ALARM_HEAT_SINK_TEMP;
	}
	else
	{
		m_u16SafeErrorNumbers[ucCh+1][RTY_HEAT_SINK_TEMP] = 0;
    }	
	#else
	if ( GetHeatSinkTemperaturError(ucCh)>90.0f ) 
	{
		if ( ++m_u16SafeErrorNumbers[ucCh][RTY_HEAT_SINK_TEMP]>=5000 )
			return _STEP_ALARM_HEAT_SINK_TEMP;
	}
	else
		m_u16SafeErrorNumbers[ucCh][RTY_HEAT_SINK_TEMP] = 0;
	#endif
			
	if ( m_bAirTemperaturError )
		return _STEP_ALARM_AIR_TEMP;
	//if ( m_bHeatSinkTemperaturError[ucCh] )
	//	return _STEP_ALARM_HEAT_SINK_TEMP;	
	if ( m_bFanError )
		return _STEP_ALARM_FAN;
	if ( m_bVoltageDeltaError[0][ucCh] )
		return _STEP_ALARM_VOLTAGE_FAST_DOWN;
	if ( m_bVoltageDeltaError[1][ucCh] )
		return _STEP_ALARM_VOLTAGE_FAST_UP;	
			
	return 0;
}

void NewStepSetting( unsigned char ucCh )
{		
	S16 s16DacVoltage;
	S16 s16DacCurrent;

	m_bDacCurrentSoftUp[ucCh] = FALSE;
	m_ucChargePowerControl = _CHARGE_POWER_FIRST;
	m_ucNextCurrentDelay[ucCh] = 0;

	if ( m_ucStepIndexNow[ucCh] >= _MAX_SEQUENCE_STEP || m_pSeqNow[ucCh]->ucState == _STATE_NONE )
	{
		if ( m_ucStepIndexNow[ucCh] >= _MAX_SEQUENCE_STEP )
			m_ucStepIndexNow[ucCh] = _MAX_SEQUENCE_STEP-1;

		if ( m_bChannelRunning[ucCh] )
		{
			FormationEnd( ucCh );
			m_ucCompletStatus[ucCh] = TRUE;	//LJK 2023.04.04 스케줄 완료
		}

		return;
	}
		
	// 20230109 djl LGES 현장 이슈
	// ReadyToStartForSafety에 의해 Stop이 될 경우 ChargeV, discV 0으로 출력하는 현상 해결
	if(m_bChannelRunning[ucCh] == TRUE)
		NowModeStateSet( ucCh );
		
	m_ucNowPulseMode[ucCh] = m_pSeqNow[ucCh]->ucPulseMode;
	FanControl( m_pSeqNow[ucCh]->ucState == _STATE_REST ? FALSE : TRUE );
	if ( m_pSeqNow[ucCh]->ucState == _STATE_REST )
	{
		m_ucNowRestChargeDisCharge[ucCh] = _NOW_STATE_REST;
		switch(ucCh)
		{
			case 0 : 
				if ( m_pSeqNext[0]->ucState == _STATE_CHARGE )
				{
					_CH1_CHARGE_ENABLE_OFF;
					_CH1_DISCHARGE_ENABLE_OFF;
					_CH1_MODE_CHARGE;
					SetDacWriteData( _DAC_CH1_CC, 0x7fff );	// bgyu 20250528
					SetDacWriteData( _DAC_CH1_CV, 0x7fff );
					_LDAC_ENABLE;
				}
				else
				if ( m_pSeqNext[0]->ucState == _STATE_DISCHARGE ||
					m_pSeqNext[0]->ucMode==_MODE_2POINT_PULSE_DISCHARGE || m_pSeqNext[0]->ucMode==_MODE_10POINT_PULSE_DISCHARGE ||  m_pSeqNext[0]->ucMode==_MODE_DISCHARGE_CONTACT )	// bgyu 20250616
				{
					_CH1_CHARGE_ENABLE_OFF;
					_CH1_DISCHARGE_ENABLE_OFF;
					_CH1_MODE_DISCHARGE;
					SetDacWriteData( _DAC_CH1_CC, 0x8000 );	// bgyu 20250528
					SetDacWriteData( _DAC_CH1_CV, 0x0000 );
					_LDAC_ENABLE;
				}
				break;
			case 1 :
				if ( m_pSeqNext[1]->ucState == _STATE_CHARGE )
				{
					_CH2_CHARGE_ENABLE_OFF;
					_CH2_DISCHARGE_ENABLE_OFF;
					_CH2_MODE_CHARGE;
					SetDacWriteData( _DAC_CH2_CC, 0x7fff );	// bgyu 20250528
					SetDacWriteData( _DAC_CH2_CV, 0x7fff );
					_LDAC_ENABLE;
				}
				else
				if ( m_pSeqNext[1]->ucState == _STATE_DISCHARGE ||
					m_pSeqNext[1]->ucMode==_MODE_2POINT_PULSE_DISCHARGE || m_pSeqNext[1]->ucMode==_MODE_10POINT_PULSE_DISCHARGE ||  m_pSeqNext[1]->ucMode==_MODE_DISCHARGE_CONTACT )	// bgyu 20250616
				{
					_CH2_CHARGE_ENABLE_OFF;
					_CH2_DISCHARGE_ENABLE_OFF;
					_CH2_MODE_DISCHARGE;
					SetDacWriteData( _DAC_CH2_CC, 0x8000 );	// bgyu 20250528
					SetDacWriteData( _DAC_CH2_CV, 0x0000 );
					_LDAC_ENABLE;
				}
				break;
			case 2 :
				if ( m_pSeqNext[2]->ucState == _STATE_CHARGE )
				{
					_CH3_CHARGE_ENABLE_OFF;
					_CH3_DISCHARGE_ENABLE_OFF;
					_CH3_MODE_CHARGE;
					SetDacWriteData( _DAC_CH3_CC, 0x7fff );	// bgyu 20250528
					SetDacWriteData( _DAC_CH3_CV, 0x7fff );
					_LDAC_ENABLE;
				}
				else
				if ( m_pSeqNext[2]->ucState == _STATE_DISCHARGE ||
					m_pSeqNext[2]->ucMode==_MODE_2POINT_PULSE_DISCHARGE || m_pSeqNext[2]->ucMode==_MODE_10POINT_PULSE_DISCHARGE ||  m_pSeqNext[2]->ucMode==_MODE_DISCHARGE_CONTACT )	// bgyu 20250616
				{
					_CH3_CHARGE_ENABLE_OFF;
					_CH3_DISCHARGE_ENABLE_OFF;
					_CH3_MODE_DISCHARGE;
					SetDacWriteData( _DAC_CH3_CC, 0x8000 );	// bgyu 20250528
					SetDacWriteData( _DAC_CH3_CV, 0x0000 );
					_LDAC_ENABLE;
				}
				break;
			case 3 :
				if ( m_pSeqNext[3]->ucState == _STATE_CHARGE )
				{
					_CH4_CHARGE_ENABLE_OFF;
					_CH4_DISCHARGE_ENABLE_OFF;
					_CH4_MODE_CHARGE;
					SetDacWriteData( _DAC_CH4_CC, 0x7fff );	// bgyu 20250528
					SetDacWriteData( _DAC_CH4_CV, 0x7fff );
					_LDAC_ENABLE;
				}
				else
				if ( m_pSeqNext[3]->ucState == _STATE_DISCHARGE ||
					m_pSeqNext[3]->ucMode==_MODE_2POINT_PULSE_DISCHARGE || m_pSeqNext[3]->ucMode==_MODE_10POINT_PULSE_DISCHARGE ||  m_pSeqNext[3]->ucMode==_MODE_DISCHARGE_CONTACT )	// bgyu 20250616
				{
					_CH4_CHARGE_ENABLE_OFF;
					_CH4_DISCHARGE_ENABLE_OFF;
					_CH4_MODE_DISCHARGE;
					SetDacWriteData( _DAC_CH4_CC, 0x8000 );	// bgyu 20250528
					SetDacWriteData( _DAC_CH4_CV, 0x0000 );
					_LDAC_ENABLE;
				}
				break;
			//case 1 : _CH2_CHARGE_ENABLE_OFF; _CH2_DISCHARGE_ENABLE_OFF; break;
			//case 2 : _CH3_CHARGE_ENABLE_OFF; _CH3_DISCHARGE_ENABLE_OFF; break;
			//case 3 : _CH4_CHARGE_ENABLE_OFF; _CH4_DISCHARGE_ENABLE_OFF; break;
		}		
	}
	else
	if ( m_pSeqNow[ucCh]->ucState == _STATE_CHARGE || m_pSeqNow[ucCh]->ucMode == _MODE_CHARGE_CONTACT )
	{
		m_ucNowRestChargeDisCharge[ucCh] = _NOW_STATE_CHARGE;

		if ( m_bChannelRunning[ucCh] )
		{
			s16DacVoltage = GetVoltageCurrentToDacHex( VOLTAGE_RANGE_INDEX+ucCh,  m_pSeqNow[ucCh]->fSettingVoltage );		// Charge Voltage

			s16DacCurrent = GetVoltageCurrentToDacHex( CURRENT_RANGE_INDEX+ucCh*2+1, m_pSeqNow[ucCh]->fSettingCurrent );	// Charge Current
			m_ucNextCurrentDelay[ucCh] = 2;	// bgyu 20250529
			if ( m_pSeqNow[ucCh]->ucMode != _MODE_CPCV && m_pSeqNow[ucCh]->ucMode != _MODE_CRCV )
				 m_s16DacCurrentNext[ucCh] = s16DacCurrent;
			else
				 m_s16DacCurrentNext[ucCh] = 0x0100 ;
			SetDacWriteData( _DAC_CH1_CV+ucCh*2, s16DacVoltage );
			SetDacWriteData( _DAC_CH1_CC+ucCh*2, 0x7fff );	// bgyu 20250528	적분기 쳐박기
		}
	}
	else
	if ( m_pSeqNow[ucCh]->ucState == _STATE_DISCHARGE || m_pSeqNow[ucCh]->ucMode == _MODE_DISCHARGE_CONTACT)
	{
		m_ucNowRestChargeDisCharge[ucCh] = _NOW_STATE_DISCHARGE;
		if ( m_bChannelRunning[ucCh] )
		{
			s16DacVoltage = GetVoltageCurrentToDacHex( VOLTAGE_RANGE_INDEX+ucCh,  m_pSeqNow[ucCh]->fSettingVoltage );		// DisCharge Voltage
			s16DacCurrent = GetVoltageCurrentToDacHex( CURRENT_RANGE_INDEX+ucCh*2, m_pSeqNow[ucCh]->fSettingCurrent );	// DisCharge Current
			
			if ( ucCh==1 || ucCh==2  )	
				m_ucNextCurrentDelay[ucCh] = 3;	// bgyu 20250604
			else
				m_ucNextCurrentDelay[ucCh] = 2;	// bgyu 20250529
			if ( m_pSeqNow[ucCh]->ucMode != _MODE_CPCV && m_pSeqNow[ucCh]->ucMode != _MODE_CRCV )
				 m_s16DacCurrentNext[ucCh] = s16DacCurrent;
			else
				 m_s16DacCurrentNext[ucCh] = 0xfe00;
			SetDacWriteData( _DAC_CH1_CV+ucCh*2, s16DacVoltage );
			SetDacWriteData( _DAC_CH1_CC+ucCh*2, 0x8000 );	// bgyu 20250528	적분기 쳐박기
		}
	}
	else
	if ( m_pSeqNow[ucCh]->ucState == _STATE_BALANCE )	// Balance의 경우 마지막 Enable 채널의 배터리 전압으로 충방전 결정
	{
		if ( m_bChannelRunning[ucCh] )
		{
			if ( m_fVoltage1Sec[ucCh] > m_pSeqNow[ucCh]->fSettingVoltage)
			{
				m_ucNowRestChargeDisCharge[ucCh] = _NOW_STATE_DISCHARGE;
				s16DacCurrent = GetVoltageCurrentToDacHex( CURRENT_RANGE_INDEX+ucCh*2, m_pSeqNow[ucCh]->fSettingCurrent );	// DisCharge Current
			}
			else
			{
				m_ucNowRestChargeDisCharge[ucCh] = _NOW_STATE_CHARGE;
				s16DacCurrent = GetVoltageCurrentToDacHex( CURRENT_RANGE_INDEX+ucCh*2+1, m_pSeqNow[ucCh]->fSettingCurrent );	// Charge Current
			}
			s16DacVoltage = GetVoltageCurrentToDacHex( VOLTAGE_RANGE_INDEX+ucCh,  m_pSeqNow[ucCh]->fSettingVoltage );		// DisCharge Voltage
			
			// 20221204 Balance에서 CPCR모드 시작 수정
			if ( m_pSeqNow[ucCh]->ucMode == _MODE_CPCV || m_pSeqNow[ucCh]->ucMode == _MODE_CRCV )
				SetDacWriteData( _DAC_CH1_CC+ucCh*2, m_ucNowRestChargeDisCharge[ucCh] == _NOW_STATE_CHARGE ? 0x1000 : 0xf000 );
			else
				SetDacWriteData( _DAC_CH1_CC+ucCh*2, s16DacCurrent );
			SetDacWriteData( _DAC_CH1_CV+ucCh*2, s16DacVoltage );
		}
	}
	else
	if ( m_pSeqNow[ucCh]->ucState == _STATE_PULSE || m_pSeqNow[ucCh]->ucState == _STATE_DCIR )
	{
		CalculatePulseStepStart(ucCh);
	}
	
	if ( m_pSeqNow[ucCh]->ucMode==_MODE_2POINT_PULSE_DISCHARGE || m_pSeqNow[ucCh]->ucMode==_MODE_2POINT_PULSE_CHARGE )	// bgyu 20250616
	{
		if ( (m_pSeqNow[ucCh]->ucPulseMode==_PULSE_MODE_CRCV && m_pSeqNow[ucCh]->fPulseCurrent[0]==0.0f) || m_pSeqNow[ucCh]->ucPulseMode==_PULSE_MODE_PRCV_CP2 )
		{
			m_pSeqNow[ucCh]->fPulseCurrent[0] = m_fVoltage1Sec[ucCh]/m_fPulseDisChargeCurrentPreStep[ucCh]*1000.0f;
		}
		else
		if ( ((m_pSeqNow[ucCh]->ucPulseMode==_PULSE_MODE_CCCV||m_pSeqNow[ucCh]->ucPulseMode==_PULSE_MODE_CCCV_DP) && m_pSeqNow[ucCh]->fPulseCurrent[0]==0.0f) || m_pSeqNow[ucCh]->ucPulseMode==_PULSE_MODE_PCCV_DP2 )
		{
			m_pSeqNow[ucCh]->fPulseCurrent[0] = m_fPulseChargeCurrentPreStep[ucCh];
		}
		else
		if ( ((m_pSeqNow[ucCh]->ucPulseMode==_PULSE_MODE_CPCV||m_pSeqNow[ucCh]->ucPulseMode==_PULSE_MODE_CPCV_DP) && m_pSeqNow[ucCh]->fPulseCurrent[0]==0.0f) || m_pSeqNow[ucCh]->ucPulseMode==_PULSE_MODE_PPCV_DP2 )
		{
			m_pSeqNow[ucCh]->fPulseCurrent[0] = m_fPulseChargeCurrentPreStep[ucCh]*m_fVoltage1Sec[ucCh];
		}
	}			

	
	#ifdef MODIDY_PULSE_OVERSHOOT
	if ( m_pSeqNow[ucCh]->ucMode == _MODE_2POINT_PULSE_DISCHARGE || m_pSeqNow[ucCh]->ucMode==_MODE_2POINT_PULSE_CHARGE ||
        m_pSeqNow[ucCh]->ucMode == _MODE_10POINT_PULSE_DISCHARGE || m_pSeqNow[ucCh]->ucMode == _MODE_10POINT_PULSE_CHARGE ||
        m_pSeqNow[ucCh]->ucMode == _MODE_CCCV
	    )	// bgyu 20250616
	{
    	s16DacVoltage = GetVoltageCurrentToDacHex( VOLTAGE_RANGE_INDEX+ucCh,  m_pSeqNow[ucCh]->fSettingVoltage );
        SetDacWriteData( _DAC_CH1_CV+ucCh*2, s16DacVoltage );
    	
    	if ( m_pSeqNow[ucCh]->ucMode==_MODE_2POINT_PULSE_DISCHARGE )
    	{
    		SetDacWriteData( _DAC_CH1_CC+ucCh*2, 0x8000 );	// bgyu 20250905	적분기 쳐박기
    	}
    	else
    	{
    		SetDacWriteData( _DAC_CH1_CC+ucCh*2, 0x7fff );	// bgyu 20250905	적분기 쳐박기		    
    	}
    }
	#endif

	if ( !m_ucFirstStep[ucCh] )
		m_bNewPulseLoad[ucCh] = TRUE;
		
	m_bNewStepStartReady[ucCh] = TRUE;

	memset( m_u16SafeErrorNumbers[ucCh], 0, sizeof( m_u16SafeErrorNumbers[ucCh] ) );

	if ( m_bChannelRunning[ucCh] )
	{
		AlarmParameterSet( ucCh );
	}
	
	m_uiRecordSecond[ucCh] = m_pSeqNow[ucCh]->uiDataRecordSecond32;
	m_uiRecordMilliSecond[ucCh] = m_uiRecordSecond[ucCh] * 1000;
}

void SetPulseModeChange( unsigned char ucCh )
{
	if ( m_ucNowRestChargeDisCharge[ucCh] == m_ucPreState[ucCh] && !m_ucFirstStep2[ucCh] )
		return;

	m_ucPreState[ucCh] = m_ucNowRestChargeDisCharge[ucCh];

	if ( m_ucNowRestChargeDisCharge[ucCh] == _NOW_STATE_DISCHARGE )
	{
		if ( m_bStepRunning[ucCh] || (m_ucFirstStep2[ucCh] && m_bChannelRunning[ucCh]) )
		{
			
			gpio_set_pin_low(m_uiChargePullupAddress[ucCh]);
			switch(ucCh)
			{
				case 0 : _CH1_CHARGE_ENABLE_OFF; break;
				case 1 : _CH2_CHARGE_ENABLE_OFF; break;
				case 2 : _CH3_CHARGE_ENABLE_OFF; break;
				case 3 : _CH4_CHARGE_ENABLE_OFF; break;
			}
			
			switch(ucCh)
			{	
				case 0 : 
					_CH1_MODE_DISCHARGE;
					if ( m_pSeqNow[0]->ucState!= _STATE_DISCHARGE )	//bgyu 20250529
						_CH1_DISCHARGE_ENABLE_ON;
					 break;
				case 1 : 
					_CH2_MODE_DISCHARGE; 
					if ( m_pSeqNow[1]->ucState!= _STATE_DISCHARGE )	//bgyu 20250529
						_CH2_DISCHARGE_ENABLE_ON; 
					break;
				case 2 : 
					_CH3_MODE_DISCHARGE; 
					if ( m_pSeqNow[2]->ucState!= _STATE_DISCHARGE )	//bgyu 20250529
						_CH3_DISCHARGE_ENABLE_ON; 
					break;
				case 3 : 
					_CH4_MODE_DISCHARGE; 
					if ( m_pSeqNow[3]->ucState!= _STATE_DISCHARGE )	//bgyu 20250529
						_CH4_DISCHARGE_ENABLE_ON; 
					break;
			}
		}
	}
	else
	if ( m_ucNowRestChargeDisCharge[ucCh] == _NOW_STATE_CHARGE )
	{
		if ( m_bStepRunning[ucCh] || (m_ucFirstStep2[ucCh] && m_bChannelRunning[ucCh]) )
		{
			gpio_set_pin_high(m_uiChargePullupAddress[ucCh]);	//LJK 2023.05.24 low->high
			switch(ucCh)
			{
				case 0 : _CH1_DISCHARGE_ENABLE_OFF; break;
				case 1 : _CH2_DISCHARGE_ENABLE_OFF; break;
				case 2 : _CH3_DISCHARGE_ENABLE_OFF; break;
				case 3 : _CH4_DISCHARGE_ENABLE_OFF; break;
			}
				
			switch(ucCh)	
			{
				case 0 : 
					_CH1_MODE_CHARGE;
					if ( m_pSeqNow[0]->ucState!=_STATE_CHARGE )	//bgyu 20250529
						 _CH1_CHARGE_ENABLE_ON; 
					break;
				case 1 : 
					_CH2_MODE_CHARGE;
					if ( m_pSeqNow[1]->ucState!=_STATE_CHARGE )	//bgyu 20250529
						_CH2_CHARGE_ENABLE_ON; 
					break;
				case 2 : 
					_CH3_MODE_CHARGE; 
					if ( m_pSeqNow[2]->ucState!=_STATE_CHARGE )	//bgyu 20250529
						_CH3_CHARGE_ENABLE_ON; 
					break;
				case 3 : 
					_CH4_MODE_CHARGE;
					if ( m_pSeqNow[3]->ucState!=_STATE_CHARGE )	//bgyu 20250529
						_CH4_CHARGE_ENABLE_ON; 
					break;
			}
		}
	}
	else
	{
		gpio_set_pin_high(m_uiChargePullupAddress[ucCh]);
		switch (ucCh)
		{
			case 0 : _CH1_DISCHARGE_ENABLE_OFF; _CH1_CHARGE_ENABLE_OFF; break;
			case 1 : _CH2_DISCHARGE_ENABLE_OFF; _CH2_CHARGE_ENABLE_OFF; break;
			case 2 : _CH3_DISCHARGE_ENABLE_OFF; _CH3_CHARGE_ENABLE_OFF; break;
			case 3 : _CH4_DISCHARGE_ENABLE_OFF; _CH4_CHARGE_ENABLE_OFF; break;
		}
	}
}

char FormationEndCheck( unsigned char ucCh )
{
	if ( !m_bChannelRunning[ucCh] )
		return FALSE;
	if ( m_pSeqNow[ucCh]->usLoopCountNow < m_pSeqNow[ucCh]->usLoopCountMax )
		return FALSE;
	if ( m_ucStepIndexNow[ucCh] + 1 >= _MAX_SEQUENCE_STEP || (m_pSeqNext[ucCh]->ucState==_STATE_NONE && m_pSeqNow[ucCh]->usLoopCountNow > m_pSeqNow[ucCh]->usLoopCountMax) )
	{
		FormationEnd( ucCh );
		return TRUE;
	}
	if ( m_pSeqNext[ucCh]->ucState==_STATE_NONE && m_pSeqNow[ucCh]->usLoopCountMax==0 )	// bgyu 20250611
	{
		FormationEnd( ucCh );
		return TRUE;
	}
	if ( m_pSeqNow[ucCh]->usLoopCountMax && m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[ucCh][m_pSeqNow[ucCh]->ucLoopIndex].ucState==_STATE_NONE )
	{
		FormationEnd( ucCh );
		return TRUE;
	}
	return FALSE;
}

void StepStart( unsigned char ucCh )
{
	U8 j;
	_RESULT_DATA * pStepEndCondition = &m_pFORMATION_STEP_END_DATA_STRUCTURE->stStepEndData[ucCh][m_pSeqNow[ucCh]->ucSocDodIndex].StepEndCondition;
		
	
	m_ui1SecRingCountVoltage[ucCh] = m_ui1SecRingCountCurrent[ucCh] = m_ui1SecRingCountVoltage2[ucCh] =
	m_uiRecordRingCountVoltage[ucCh] = m_uiRecordRingCountCurrent[ucCh] = m_uiRecordRingCountVoltage2[ucCh] = 0;
	m_bDCIR_StepStop[ucCh] = FALSE;
	m_bContinueNextSequence[ucCh] = FALSE;
	


	m_bStepRunning[ucCh] = FALSE;
	m_uiStepCV_TimeNow[ucCh] = 0;
	m_fChargeVoltage[ucCh] = m_fDisChargeVoltage[ucCh] = 
	m_fWatt[ucCh] = m_fChargeCurrent[ucCh] = m_fDisChargeCurrent[ucCh] = m_dblChargeCapacity[ucCh] = m_dblDisChargeCapacity[ucCh]= m_dblChargeWattHour[ucCh] = m_dblDisChargeWattHour[ucCh] = 0;
	
	// 20230118 djl
	m_fSoc[ucCh] = (pStepEndCondition->fChargeCapacity - pStepEndCondition->fDisChargeCapacity) * (m_pSeqNow[ucCh]->StepEndCondition.fChargeCapacity/100.0f);
	m_fDod[ucCh] = (pStepEndCondition->fDisChargeCapacity - pStepEndCondition->fChargeCapacity) * (m_pSeqNow[ucCh]->StepEndCondition.fDisChargeCapacity/100.0f);
	
	//Test 2024.10.22 SoC, DoD Adaptive
	if ( m_pSeqNow[ucCh]->u32StepEndConditionsEnable & (1<<_STEP_END_CONDITION_SOC)  || m_pSeqNow[ucCh]->u32StepEndConditionsEnable & (1<<_STEP_END_CONDITION_DOD))
	{
		j = m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[ucCh][m_pSeqNow[ucCh]->ucSocDodIndex].ucSocDodIndex;		
		if( j != 0)
		{
			pStepEndCondition = &m_pFORMATION_STEP_END_DATA_STRUCTURE->stStepEndData[ucCh][j].StepEndCondition;
			m_fSoc[ucCh] += (pStepEndCondition->fChargeCapacity - pStepEndCondition->fDisChargeCapacity) * (m_pSeqNow[ucCh]->StepEndCondition.fChargeCapacity/100.0f);
			m_fDod[ucCh] += (pStepEndCondition->fDisChargeCapacity - pStepEndCondition->fChargeCapacity) * (m_pSeqNow[ucCh]->StepEndCondition.fDisChargeCapacity/100.0f);			
		}		
	}
	
	//LJK 2024.10.23	
	if(  m_pSeqNow[ucCh]->ucState == _STATE_DCIR )
	{
		m_fDCIR_Voltage1[ucCh] = 0;
		if( m_pSeqNow[ucCh]->ucMode == _MODE_10ms_DCIR)
		{
			memset(m_fDCIR_Voltage2[ucCh], 0x00, sizeof(float)*MAX_DCIR_10MS);
			memset(m_fDCIR_Current[ucCh], 0x00, sizeof(float)*MAX_DCIR_10MS);
		}
		else if( m_pSeqNow[ucCh]->ucMode == _MODE_Sec_DCIR )
		{
			memset(m_pSecDcirBuffer->fDCIR_Current[ucCh], 0x00, sizeof(float)*MAX_DCIR_SEC);	
			m_pSecDcirBuffer->usDCIR_Count[ucCh] = 0;			
		}
	}
		
	/*	
	m_fDCIR_Voltage1[ucCh] = 0;
	
	for( j=0; j<15; j++)
		m_fDCIR_Voltage2[ucCh][j] = m_fDCIR_Current[ucCh][j] = 0;

	//DCIR Sec 2024.10.14
	memset(m_pSecDcirBuffer->fDCIR_Current[ucCh], 0x00, sizeof(float)*MAX_DCIR_SEC);	
	m_pSecDcirBuffer->usDCIR_Count[ucCh] = 0;
	*/
	
	
	//이어서하기 테스트 중
	if( m_ucResumeStatus[ucCh] )
	{
		m_ucResumeStatus[ucCh] = FALSE;
		m_bDCIR_StepStop[ucCh] = FALSE;
	}
	else
	{
		m_uiStepTimeNow[ucCh] = 0;
		// 20230111 djl Pulse Run Time 동기화
		if ( m_ucNowState[ucCh] == _STATE_PULSE)// || m_ucNowState[ucCh] ==_STATE_DCIR )
			m_uiStepTimeNow[ucCh] = 1;
	}
	//LJK 2023.06.26  첫번째 레코드 m_uiStepTimeNow 이전 시간문제, 위치변경
	if ( m_bChannelRunning[ucCh] )
	{
		m_bStepRunning[ucCh] = TRUE;
		m_fStepStartVoltage[ucCh] = m_fVoltage1Sec[ucCh];
		if( !m_ucResumeStatus[ucCh] ) PushStepRecordData( ucCh, _STEP_TESTING );
	}	
		
}

void CalculateCapacity( unsigned char ucCh )
{
	#define dblHourFactor (0.001/3600.0);
	
	if ( m_uiStepTimeNow[ucCh] ==0 || m_uiStepTimeNow[ucCh] >0xfffffff0)
	{
		m_fWatt[ucCh] = m_fChargeCurrent[ucCh] = m_fDisChargeCurrent[ucCh] = m_dblChargeCapacity[ucCh] = m_dblDisChargeCapacity[ucCh]= m_dblChargeWattHour[ucCh] = m_dblDisChargeWattHour[ucCh] = 0;
		return;
	}
		
	if ( !m_bChannelRunning[ucCh] || !m_bStepRunning[ucCh] )
		return;
	if ( m_ucCalculateMode[ucCh] == _NOW_STATE_DISCHARGE )
	{
		m_dblDisChargeCapacity[ucCh] += m_fCurrent[ucCh] * dblHourFactor;
		m_dblDisChargeWattHour[ucCh] += m_fWatt[ucCh] * dblHourFactor;
	}
	else
	if ( m_ucCalculateMode[ucCh] == _NOW_STATE_CHARGE )
	{
		m_dblChargeCapacity[ucCh] += m_fCurrent[ucCh] * dblHourFactor;
		m_dblChargeWattHour[ucCh] += m_fWatt[ucCh] * dblHourFactor;
	}
}

void CalculatePulseStepStart( unsigned char ucCh )
{
	m_ucPulseIndex[ucCh] = 0xff;
	m_u16PulseTime1ms[ucCh] = m_pSeqNow[ucCh]->u16PulseTime1ms[0];
	m_u16PulseTime1msNow[ucCh] = m_u16PulseTime1ms[ucCh];
	//m_u16PulseTime1msNow[ucCh]--;	//LJK 2023.05.26
	//m_fPulseCurrent[ucCh] = m_pSeqNow[ucCh]->fPulseCurrent[0] >= 0.0f ? 0.4f : -0.4f;
	m_fPulseCurrent[ucCh] = m_pSeqNow[ucCh]->fPulseCurrent[0] >= 0.0f ? CHARGE_MINIMUM_CURREN : DISCHARGE_MINIMUM_CURRENT;
	//SetNextPulse(ucCh);
}

void NowModeStateSet( unsigned char ucCh )
{	
	m_ucNowState[ucCh] = m_pSeqNow[ucCh]->ucState;
	m_ucNowMode[ucCh] = m_pSeqNow[ucCh]->ucMode;
	m_bSoftCurrentMode[ucCh] = !!((1<<_STEP_END_CONDITION_SOFT_CURRENT_MODE)&m_pSeqNow[ucCh]->u32StepEndConditionsEnable);
	
	if ( m_ucNowState[ucCh] == _STATE_REST )
	{
		m_ucNowRestChargeDisCharge[ucCh] = _NOW_STATE_REST;
	}
	else
	if ( m_ucNowState[ucCh] == _STATE_CHARGE || m_ucNowState[ucCh] == _STATE_DCIR || (m_ucNowState[ucCh] == _STATE_PULSE && m_pSeqNow[ucCh]->fPulseCurrent[0]>0) || m_ucNowMode[ucCh] == _MODE_CHARGE_CONTACT )
		m_ucNowRestChargeDisCharge[ucCh] = _NOW_STATE_CHARGE;
	else
		m_ucNowRestChargeDisCharge[ucCh] = _NOW_STATE_DISCHARGE;

	if ( m_pSeqNow[ucCh]->ucState == _STATE_BALANCE )	// Balance의 경우 마지막 Enable 채널의 배터리 전압으로 충방전 결정
	{
		if ( !m_bChannelRunning[ucCh] )
			return;
		if ( m_fVoltage1Sec[ucCh] > m_pSeqNow[ucCh]->fSettingVoltage)
		{
			m_ucNowRestChargeDisCharge[ucCh] = _NOW_STATE_DISCHARGE;
			//LJK 2023.05.17 Discharge 일때 Charge EndConditions Mask
			m_pSeqNow[ucCh]->u32StepEndConditionsEnable &=~(1<<_STEP_END_CONDITION_CHARGE_VOLTAGE);
			m_pSeqNow[ucCh]->u32StepEndConditionsEnable &=~(1<<_STEP_END_CONDITION_CHARGE_CURRENT);
			m_pSeqNow[ucCh]->u32StepEndConditionsEnable &=~(1<<_STEP_END_CONDITION_CHARGE_CAPACITY);
			m_pSeqNow[ucCh]->u32StepEndConditionsEnable &=~(1<<_STEP_END_CONDITION_CHARGE_WATTHOUR);			
		}
		else
		{
			m_ucNowRestChargeDisCharge[ucCh] = _NOW_STATE_CHARGE;
			//LJK 2023.05.17 Charge 일때  Discharge EndConditions Mask
			m_pSeqNow[ucCh]->u32StepEndConditionsEnable &=~(1<<_STEP_END_CONDITION_DISCHARGE_VOLTAGE);
			m_pSeqNow[ucCh]->u32StepEndConditionsEnable &=~(1<<_STEP_END_CONDITION_DISCHARGE_CURRENT);
			m_pSeqNow[ucCh]->u32StepEndConditionsEnable &=~(1<<_STEP_END_CONDITION_DISCHARGE_CAPACITY);
			m_pSeqNow[ucCh]->u32StepEndConditionsEnable &=~(1<<_STEP_END_CONDITION_DISCHARGE_WATTHOUR);
		}
	}
	
#ifdef _SOFT_AUTO_
	if( m_ucNowState[ucCh] == _STATE_PULSE || m_ucNowState[ucCh] == _STATE_DCIR )
	{
		switch(ucCh)
		{
			case 0: PDC_HIGH = _PORT_C_I_TR_Soft0_Fast1; break;	//Ch1 Boost
			case 1: PDD_HIGH = _PORT_D_GATE2; break;	//Ch2 Boost
			case 2: PDD_HIGH = _PORT_D_GATE3; break;	//Ch3 Boost
			case 3: PDD_HIGH = _PORT_D_GATE4; break;	//Ch4 Boost
		}
	}
	else
	{
		switch(ucCh)
		{
			case 0: PDC_LOW =_PORT_C_I_TR_Soft0_Fast1; break;	//Ch1 Soft
			case 1: PDD_LOW =_PORT_D_GATE2; break;	//Ch2 Soft
			case 2: PDD_LOW =_PORT_D_GATE3; break;	//Ch3 Soft
			case 3: PDD_LOW =_PORT_D_GATE4; break;	//Ch4 Soft
		}
	}
#endif	
}

void SetNextPulse( unsigned char ucCh )
{
	if ( m_ucNowState[ucCh] != _STATE_PULSE && m_ucNowState[ucCh] != _STATE_DCIR )
		return;

	S16 s16DacCurrent;
	S16 s16DacVoltage;
	float fCpCrCurrent;
	float fTriangleCurrentDac;
	float m_fPulseCpCr;
	float fDisChargePercentCurrent;
	float fChargePercentCurrent;
	
	#ifdef MODIFY_PULSE_CPCR
	float fMaxCurrent = 0.0f;
	float fPulseUp1Sec = 0.0f;
	#endif
	
	m_u16PulseTime1msNow[ucCh]++;
	
	if ( m_u16PulseTime1ms[ucCh]< m_u16PulseTime1msNow[ucCh] )
	{
		m_bNewPulseLoad[ucCh] = TRUE;
		
		m_ucPulseIndex[ucCh] = (U8)(m_ucPulseIndex[ucCh] + 1) % m_pSeqNow[ucCh]->ucPulseMax;	//Next Pulse Index
		m_ucPulseKind[ucCh] = m_pSeqNow[ucCh]->ucPulseKind[m_ucPulseIndex[ucCh]];
		m_fPulseCurrentPre[ucCh] = m_fPulseCurrent[ucCh];
		m_fPulseCurrent[ucCh] = m_pSeqNow[ucCh]->fPulseCurrent[m_ucPulseIndex[ucCh]];
		m_u16PulseTime1ms[ucCh] = m_pSeqNow[ucCh]->u16PulseTime1ms[m_ucPulseIndex[ucCh]];
		m_u16PulseTime1msNow[ucCh] = 1;
		
		if ( m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV_DP ||	// bgyu 20250612
			m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV || m_ucNowPulseMode[ucCh] == _PULSE_MODE_PPCV_DP2 || m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV_CP  )	// bgyu 20250722
			m_fPulseCurrent[ucCh] = m_fPulseCurrent[ucCh]/m_fVoltage1Sec[ucCh];	// Watt
		else
		if ( m_ucNowPulseMode[ucCh] == _PULSE_MODE_CRCV || m_ucNowPulseMode[ucCh] == _PULSE_MODE_PRCV_CP2 )
			m_fPulseCurrent[ucCh] = m_fVoltage1Sec[ucCh]/m_fPulseCurrent[ucCh]*1000.0f;	// Resister mΩ 단위 bgyu 20250612

		if ( m_ucPulseKind[ucCh] == _PULSE_RECTANGLE )
		{
			//DEBUG_TP1_H;
			if ( m_fPulseCurrent[ucCh] < 0.0f )
			{
				m_ucNowRestChargeDisCharge[ucCh] = m_ucPulseNextState[ucCh] = _NOW_STATE_DISCHARGE;				
					
				#ifdef MODIFY_PULSE_CPCR
                if ( m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV    ||
			         m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV_CP || 
			         m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV_DP || 
			         m_ucNowPulseMode[ucCh] == _PULSE_MODE_PPCV_DP2 )
			    {
			        fPulseUp1Sec = m_pSeqNow[ucCh]->fPulseDisChargeUp1Sec  / m_fVoltage1Sec[ucCh];
			        fMaxCurrent  = m_pSeqNow[ucCh]->fPulseDisChargeCurrentMax / m_fVoltage1Sec[ucCh];
			    }
			    else if(m_ucNowPulseMode[ucCh] == _PULSE_MODE_CRCV || 
			            m_ucNowPulseMode[ucCh] == _PULSE_MODE_PRCV_CP2 )
			    {
			        fPulseUp1Sec = m_fVoltage1Sec[ucCh] / (m_pSeqNow[ucCh]->fPulseDisChargeUp1Sec * 1000.0f);
                    fMaxCurrent  = m_fVoltage1Sec[ucCh] / (m_pSeqNow[ucCh]->fPulseDisChargeCurrentMax * 1000.0f);
			    }
			    else
			    {
			        fPulseUp1Sec = m_pSeqNow[ucCh]->fPulseDisChargeUp1Sec;
                    fMaxCurrent  = m_pSeqNow[ucCh]->fPulseDisChargeCurrentMax;
			    }

				if ( m_pSeqNow[ucCh]->fPulseCurrent[m_ucPulseIndex[ucCh]] != CHARGE_MINIMUM_CURREN && m_pSeqNow[ucCh]->fPulseCurrent[m_ucPulseIndex[ucCh]] != DISCHARGE_MINIMUM_CURRENT )
					m_fPulseCurrent[ucCh] -= (float)m_uiStepTimeNow[ucCh] / 1000.0f * fPulseUp1Sec;

				if ( fPulseUp1Sec < (float)-0.0000001 )
				{
					if ( -m_fPulseCurrent[ucCh] < fMaxCurrent )	m_fPulseCurrent[ucCh] = -fMaxCurrent;
				}
				else if ( -m_fPulseCurrent[ucCh] > fMaxCurrent )
				{
					m_fPulseCurrent[ucCh] = -fMaxCurrent;
			    }
				#else
				if ( m_pSeqNow[ucCh]->fPulseCurrent[m_ucPulseIndex[ucCh]] != CHARGE_MINIMUM_CURREN && m_pSeqNow[ucCh]->fPulseCurrent[m_ucPulseIndex[ucCh]] != DISCHARGE_MINIMUM_CURRENT )
					m_fPulseCurrent[ucCh] -= (float)m_uiStepTimeNow[ucCh] / 1000.0f * m_pSeqNow[ucCh]->fPulseDisChargeUp1Sec;
					
				if ( m_ucNowPulseMode[ucCh] == _PULSE_MODE_CCCV || m_ucNowPulseMode[ucCh] == _PULSE_MODE_CCCV_DP || m_ucNowPulseMode[ucCh] == _PULSE_MODE_CCCV_CP ||	// bgyu 20250612
					m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV_DP || m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV_CP || m_ucNowPulseMode[ucCh] == _PULSE_MODE_CRCV ||
					m_ucNowPulseMode[ucCh] == _PULSE_MODE_PCCV_DP2 || m_ucNowPulseMode[ucCh] == _PULSE_MODE_PPCV_DP2 || m_ucNowPulseMode[ucCh] == _PULSE_MODE_PRCV_CP2 ||		// bgyu 20250616
					m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV )	// bgyu 20250722
				{
					if ( m_pSeqNow[ucCh]->fPulseDisChargeUp1Sec < (float)-0.0000001 )	// bgyu 20250611	증가시는 Max Limit, 감소시는 Min Limit
					{
						if ( -m_fPulseCurrent[ucCh] < m_pSeqNow[ucCh]->fPulseDisChargeCurrentMax )
							m_fPulseCurrent[ucCh] = -m_pSeqNow[ucCh]->fPulseDisChargeCurrentMax;
					}
					else
					if ( -m_fPulseCurrent[ucCh] > m_pSeqNow[ucCh]->fPulseDisChargeCurrentMax )
						m_fPulseCurrent[ucCh] = -m_pSeqNow[ucCh]->fPulseDisChargeCurrentMax;
				}
				#endif
				
				m_fPulseDisChargeCurrentPreStep[ucCh] = m_fPulseCurrent[ucCh];	// bgyu 20250616
			}
			else
			if ( m_fPulseCurrent[ucCh] > 0.0f )
			{
				m_ucNowRestChargeDisCharge[ucCh] = m_ucPulseNextState[ucCh] = _NOW_STATE_CHARGE;
				
				#ifdef MODIFY_PULSE_CPCR
                if ( m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV    ||
			         m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV_CP || 
			         m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV_DP || 
			         m_ucNowPulseMode[ucCh] == _PULSE_MODE_PPCV_DP2 )
			    {
			        fPulseUp1Sec = m_pSeqNow[ucCh]->fPulseChargeUp1Sec / m_fVoltage1Sec[ucCh];
			        fMaxCurrent  = m_pSeqNow[ucCh]->fPulseChargeCurrentMax / m_fVoltage1Sec[ucCh];
			    }
			    else if(m_ucNowPulseMode[ucCh] == _PULSE_MODE_CRCV || 
			            m_ucNowPulseMode[ucCh] == _PULSE_MODE_PRCV_CP2 )
			    {
			        fPulseUp1Sec = m_fVoltage1Sec[ucCh] / (m_pSeqNow[ucCh]->fPulseChargeUp1Sec * 1000.0f);
                    fMaxCurrent  = m_fVoltage1Sec[ucCh] / (m_pSeqNow[ucCh]->fPulseChargeCurrentMax * 1000.0f);
			    }
			    else
			    {
			        fPulseUp1Sec = m_pSeqNow[ucCh]->fPulseChargeUp1Sec;
                    fMaxCurrent  = m_pSeqNow[ucCh]->fPulseChargeCurrentMax;
			    }
			    
				if ( m_pSeqNow[ucCh]->fPulseCurrent[m_ucPulseIndex[ucCh]] != CHARGE_MINIMUM_CURREN && m_pSeqNow[ucCh]->fPulseCurrent[m_ucPulseIndex[ucCh]] != DISCHARGE_MINIMUM_CURRENT )
					m_fPulseCurrent[ucCh] += (float)m_uiStepTimeNow[ucCh] / 1000.0f * fPulseUp1Sec;

				if ( fPulseUp1Sec < (float)-0.0000001 )
				{
					if ( m_fPulseCurrent[ucCh] < fMaxCurrent ) m_fPulseCurrent[ucCh] = fMaxCurrent;
				}
				else if ( m_fPulseCurrent[ucCh] > fMaxCurrent )	
				{
				    m_fPulseCurrent[ucCh] = fMaxCurrent;
				}
				#else
				if ( m_pSeqNow[ucCh]->fPulseCurrent[m_ucPulseIndex[ucCh]] != CHARGE_MINIMUM_CURREN && m_pSeqNow[ucCh]->fPulseCurrent[m_ucPulseIndex[ucCh]] != DISCHARGE_MINIMUM_CURRENT )
					m_fPulseCurrent[ucCh] += (float)m_uiStepTimeNow[ucCh] / 1000.0f * m_pSeqNow[ucCh]->fPulseChargeUp1Sec;
					
				if ( m_ucNowPulseMode[ucCh] == _PULSE_MODE_CCCV || m_ucNowPulseMode[ucCh] == _PULSE_MODE_CCCV_DP || m_ucNowPulseMode[ucCh] == _PULSE_MODE_CCCV_CP ||	// bgyu 20250612
					m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV_DP || m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV_CP || m_ucNowPulseMode[ucCh] == _PULSE_MODE_CRCV ||
					m_ucNowPulseMode[ucCh] == _PULSE_MODE_PCCV_DP2 || m_ucNowPulseMode[ucCh] == _PULSE_MODE_PPCV_DP2 || m_ucNowPulseMode[ucCh] == _PULSE_MODE_PRCV_CP2 ||		// bgyu 20250616
					m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV )	// bgyu 20250722
				{
					if ( m_pSeqNow[ucCh]->fPulseChargeUp1Sec < (float)-0.0000001 )	// bgyu 20250611	증가시는 Max Limit, 감소시는 Min Limit
					{
						if ( m_fPulseCurrent[ucCh] < m_pSeqNow[ucCh]->fPulseChargeCurrentMax )
							m_fPulseCurrent[ucCh] = m_pSeqNow[ucCh]->fPulseChargeCurrentMax;
					}
					else
					if ( m_fPulseCurrent[ucCh] > m_pSeqNow[ucCh]->fPulseChargeCurrentMax )
						m_fPulseCurrent[ucCh] = m_pSeqNow[ucCh]->fPulseChargeCurrentMax;
				}
				#endif
				
				m_fPulseChargeCurrentPreStep[ucCh] = m_fPulseCurrent[ucCh];	// bgyu 20250616
			}
			else
				m_ucPulseNextState[ucCh] = m_ucNowState[ucCh];
			
			if ( m_bChannelRunning[ucCh] )
			{
				if ( m_ucNowPulseMode[ucCh] == _PULSE_MODE_CCCV || m_ucNowPulseMode[ucCh] == _PULSE_MODE_CCCV_DP  || m_ucNowPulseMode[ucCh] == _PULSE_MODE_CCCV_CP ||	// bgyu 20250612
					m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV_DP || m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV_CP || m_ucNowPulseMode[ucCh] == _PULSE_MODE_CRCV ||
					m_ucNowPulseMode[ucCh] == _PULSE_MODE_PCCV_DP2 || m_ucNowPulseMode[ucCh] == _PULSE_MODE_PPCV_DP2 || m_ucNowPulseMode[ucCh] == _PULSE_MODE_PRCV_CP2 ||		// bgyu 20250616
					m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV )	// bgyu 20250722
				{
					if ( m_ucPulseNextState[ucCh] == _NOW_STATE_DISCHARGE )
					{
						if ( m_ucNowPulseMode[ucCh] == _PULSE_MODE_CCCV_DP || m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV_DP ||	// bgyu 20250612
							m_ucNowPulseMode[ucCh] == _PULSE_MODE_PCCV_DP2 || m_ucNowPulseMode[ucCh] == _PULSE_MODE_PPCV_DP2 )		// bgyu 20250616
						{
							fDisChargePercentCurrent = m_fChargeCurrent[ucCh]*(m_pSeqNow[ucCh]->fPulseCurrent[1]/100.0f);
							m_fPulseDisChargeCurrentPreStep[ucCh] = fDisChargePercentCurrent;	// bgyu 20250616	
							s16DacCurrent = GetVoltageCurrentToDacHex( CURRENT_RANGE_INDEX+ucCh*2, -fDisChargePercentCurrent );	// DisCharge Current
						}
						else
							s16DacCurrent = GetVoltageCurrentToDacHex( CURRENT_RANGE_INDEX+ucCh*2, -m_fPulseCurrent[ucCh] );	// DisCharge Current

						s16DacVoltage = GetVoltageCurrentToDacHex( VOLTAGE_RANGE_INDEX+ucCh, m_pSeqNow[ucCh]->fSettingCurrent  );	// DisCharge Voltage
					}
					else
					{
						if ( m_ucNowPulseMode[ucCh] == _PULSE_MODE_CCCV_CP || m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV_CP || m_ucNowPulseMode[ucCh] == _PULSE_MODE_CRCV ||
							m_ucNowPulseMode[ucCh] == _PULSE_MODE_PRCV_CP2 )		// bgyu 20250616
						{
							fChargePercentCurrent = m_fDisChargeCurrent[ucCh]*(m_pSeqNow[ucCh]->fPulseCurrent[1]/100.0f);
							m_fPulseChargeCurrentPreStep[ucCh] = fChargePercentCurrent;	// bgyu 20250616	
							s16DacCurrent = GetVoltageCurrentToDacHex( CURRENT_RANGE_INDEX+ucCh*2+1, fChargePercentCurrent );	// Charge Current
						}
						else
							s16DacCurrent = GetVoltageCurrentToDacHex( CURRENT_RANGE_INDEX+ucCh*2+1, m_fPulseCurrent[ucCh] );	// Charge Current

						s16DacVoltage = GetVoltageCurrentToDacHex( VOLTAGE_RANGE_INDEX+ucCh, m_pSeqNow[ucCh]->fSettingVoltage  );	// Charge Voltage
					}
					
					SetDacWriteData( _DAC_CH1_CC+ucCh*2, s16DacCurrent );
				}
				else
				if ( /*m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV_CP || m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV_DP || */m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV || m_ucNowPulseMode[ucCh] == _PULSE_MODE_CRCV )	// bgyu 20250612
				{
					if ( m_ucPulseNextState[ucCh] == _NOW_STATE_DISCHARGE )
					{
						s16DacVoltage = GetVoltageCurrentToDacHex( VOLTAGE_RANGE_INDEX+ucCh, m_pSeqNow[ucCh]->fSettingCurrent  );	// DisCharge Voltage
					}
					else
					{
						s16DacVoltage = GetVoltageCurrentToDacHex( VOLTAGE_RANGE_INDEX+ucCh, m_pSeqNow[ucCh]->fSettingVoltage  );	// Charge Voltage
					}
				}
				
				SetDacWriteData( _DAC_CH1_CV+ucCh*2, s16DacVoltage );
			}
			//DEBUG_TP1_L;
			return;
		}
		
		if ( m_ucPulseKind[ucCh] == _PULSE_TRIANGLE )
		{
			if ( m_bChannelRunning[ucCh] )
			{
				if ( m_ucNowPulseMode[ucCh] == _PULSE_MODE_CCCV )
				{
					if ( m_fPulseCurrentPre[ucCh] < 0.0f )
					{
						m_s16PulseCurrentDac1[ucCh] = GetVoltageCurrentToDacHex( CURRENT_RANGE_INDEX+ucCh*2, -m_fPulseCurrentPre[ucCh] );	// DisCharge Current
					}
					else
					{
						m_s16PulseCurrentDac1[ucCh] = GetVoltageCurrentToDacHex( CURRENT_RANGE_INDEX+ucCh*2+1, m_fPulseCurrentPre[ucCh] );	// Charge Current
					}
					
					if ( m_fPulseCurrent[ucCh] < 0.0f )
					{
						if ( m_fPulseCurrent[ucCh] != DISCHARGE_MINIMUM_CURRENT )
							m_fPulseCurrent[ucCh] -= (float)m_uiStepTimeNow[ucCh]/1000.0f*m_pSeqNow[ucCh]->fPulseDisChargeUp1Sec;
							
						if ( -m_fPulseCurrent[ucCh] > m_pSeqNow[ucCh]->fPulseDisChargeCurrentMax )
							m_fPulseCurrent[ucCh] = -m_pSeqNow[ucCh]->fPulseDisChargeCurrentMax;
							
						m_s16PulseCurrentDac2[ucCh] = GetVoltageCurrentToDacHex( CURRENT_RANGE_INDEX+ucCh*2, -m_fPulseCurrent[ucCh] );	// DisCharge Current
					}
					else
					{
						if ( m_fPulseCurrent[ucCh] != CHARGE_MINIMUM_CURREN )
							m_fPulseCurrent[ucCh] += (float)m_uiStepTimeNow[ucCh]/1000.0f*m_pSeqNow[ucCh]->fPulseChargeUp1Sec;
							
						if ( m_fPulseCurrent[ucCh] > m_pSeqNow[ucCh]->fPulseChargeCurrentMax )
							m_fPulseCurrent[ucCh] = m_pSeqNow[ucCh]->fPulseChargeCurrentMax;

							
						m_s16PulseCurrentDac2[ucCh] = GetVoltageCurrentToDacHex( CURRENT_RANGE_INDEX+ucCh*2+1, m_fPulseCurrent[ucCh] );	// Charge Current
					}

					m_fStartPulseCurrent[ucCh] = m_s16PulseCurrentDac1[ucCh];
					m_fPulsePer1msDelta[ucCh] = (float)(m_s16PulseCurrentDac2[ucCh]-m_s16PulseCurrentDac1[ucCh])/(float)m_u16PulseTime1ms[ucCh];
					fTriangleCurrentDac = (m_fStartPulseCurrent[ucCh]+(float)m_u16PulseTime1msNow[ucCh]*m_fPulsePer1msDelta[ucCh]);

					if ( fTriangleCurrentDac>=0.0f )
					{
						m_ucNowRestChargeDisCharge[ucCh] = m_ucPulseNextState[ucCh] = _NOW_STATE_DISCHARGE;
						s16DacCurrent = (S16)(fTriangleCurrentDac+0.5f);
						s16DacVoltage = GetVoltageCurrentToDacHex( VOLTAGE_RANGE_INDEX+ucCh, m_pSeqNow[ucCh]->fSettingCurrent  );	// DisCharge Voltage
					}
					else
					{
						m_ucNowRestChargeDisCharge[ucCh] = m_ucPulseNextState[ucCh] = _NOW_STATE_CHARGE;
						s16DacCurrent = (S16)(fTriangleCurrentDac-0.5f);
						s16DacVoltage = GetVoltageCurrentToDacHex( VOLTAGE_RANGE_INDEX+ucCh, m_pSeqNow[ucCh]->fSettingVoltage  );	// Charge Voltage
					}
				}
				else
				if ( m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV_CP || m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV_DP || m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV || m_pSeqNow[ucCh]->ucPulseMode==_PULSE_MODE_CRCV )
				{
					if ( m_fVoltage1Sec[ucCh] < MINIMUM_VOLTAGE )
						m_fVoltage1Sec[ucCh] = MINIMUM_VOLTAGE;
						
					m_fPulseTriangleCpCr1[ucCh] = m_fPulseCurrentPre[ucCh];
					m_fPulseTriangleCpCr2[ucCh] = m_fPulseCurrent[ucCh];
					m_fStartPulseCurrent[ucCh] = m_fPulseCurrentPre[ucCh];
					m_fPulsePer1msDelta[ucCh] = (m_fPulseTriangleCpCr2[ucCh]-m_fPulseTriangleCpCr1[ucCh])/(float)m_u16PulseTime1ms[ucCh];
					m_fPulseCpCr = (m_fStartPulseCurrent[ucCh]+(float)m_u16PulseTime1msNow[ucCh]*m_fPulsePer1msDelta[ucCh]);
					
					if ( m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV || m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV_DP || m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV_CP)
						fCpCrCurrent = m_fPulseCpCr/m_fVoltage1Sec[ucCh];	// Watt
					else
						fCpCrCurrent = m_fVoltage1Sec[ucCh]/m_fPulseCpCr*1000.0f;	// Resister mΩ 단위 bgyu 20250612
						
					if ( fCpCrCurrent> MAX_SYSTEM_CURRENT)
						fCpCrCurrent = MAX_SYSTEM_CURRENT;
					else
						if ( fCpCrCurrent< -MAX_SYSTEM_CURRENT )
						fCpCrCurrent = -MAX_SYSTEM_CURRENT;
						
					if ( fCpCrCurrent<0.0f )
					{
						m_ucPulseNextState[ucCh] = _NOW_STATE_DISCHARGE;
						if ( m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV_DP )
						{
							fDisChargePercentCurrent = m_fChargeCurrent[ucCh]*(m_pSeqNow[ucCh]->fPulseCurrent[1]/100.0f);
							s16DacCurrent = GetVoltageCurrentToDacHex( CURRENT_RANGE_INDEX+ucCh*2, -fDisChargePercentCurrent );	// DisCharge Current
						}
						else
							s16DacCurrent = GetVoltageCurrentToDacHex( CURRENT_RANGE_INDEX+ucCh*2, -fCpCrCurrent );	// DisCharge Current
							
						s16DacVoltage = GetVoltageCurrentToDacHex( VOLTAGE_RANGE_INDEX+ucCh, m_pSeqNow[ucCh]->fSettingCurrent  );	// DisCharge Voltage
					}
					else
					{
						m_ucPulseNextState[ucCh] = _NOW_STATE_CHARGE;
						if ( m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV_CP )
						{
							fChargePercentCurrent = m_fDisChargeCurrent[ucCh]*(m_pSeqNow[ucCh]->fPulseCurrent[1]/100.0f);
							s16DacCurrent = GetVoltageCurrentToDacHex( CURRENT_RANGE_INDEX+ucCh*2+1, fChargePercentCurrent );	// Charge Current
						}
						else 
							s16DacCurrent = GetVoltageCurrentToDacHex( CURRENT_RANGE_INDEX+ucCh*2+1, fCpCrCurrent );	// Charge Current
							
						s16DacVoltage = GetVoltageCurrentToDacHex( VOLTAGE_RANGE_INDEX+ucCh, m_pSeqNow[ucCh]->fSettingVoltage  );	// Charge Voltage
					}
				}
				
				SetDacWriteData( _DAC_CH1_CC+ucCh*2, s16DacCurrent );
				SetDacWriteData( _DAC_CH1_CV+ucCh*2, s16DacVoltage );
			}
			return;
		}
		return;
	}
	
	if ( m_ucPulseKind[ucCh] == _PULSE_TRIANGLE )
	{
		m_bNewPulseLoad[ucCh] = TRUE;
		
		if ( m_bChannelRunning[ucCh] )
		{
			if ( m_ucNowPulseMode[ucCh] == _PULSE_MODE_CCCV )
			{
				m_fPulsePer1msDelta[ucCh] = (float)(m_s16PulseCurrentDac2[ucCh]-m_s16PulseCurrentDac1[ucCh])/(float)m_u16PulseTime1ms[ucCh];
				fTriangleCurrentDac = (m_fStartPulseCurrent[ucCh]+(float)m_u16PulseTime1msNow[ucCh]*m_fPulsePer1msDelta[ucCh]);
				if ( fTriangleCurrentDac>=0.0f )
				{
					m_ucNowRestChargeDisCharge[ucCh] = m_ucPulseNextState[ucCh] = _NOW_STATE_DISCHARGE;
					s16DacCurrent = (S16)(fTriangleCurrentDac+0.5f);
					s16DacVoltage = GetVoltageCurrentToDacHex( VOLTAGE_RANGE_INDEX+ucCh, m_pSeqNow[ucCh]->fSettingCurrent  );	// DisCharge Voltage
				}
				else
				{
					m_ucNowRestChargeDisCharge[ucCh] = m_ucPulseNextState[ucCh] = _NOW_STATE_CHARGE;
					s16DacCurrent = (S16)(fTriangleCurrentDac-0.5f);
					s16DacVoltage = GetVoltageCurrentToDacHex( VOLTAGE_RANGE_INDEX+ucCh, m_pSeqNow[ucCh]->fSettingVoltage  );	// Charge Voltage
				}
			}
			else
			if ( m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV || m_ucNowPulseMode[ucCh] == _PULSE_MODE_CRCV )
			{
				m_fPulseCpCr = (m_fStartPulseCurrent[ucCh]+(float)m_u16PulseTime1msNow[ucCh]*m_fPulsePer1msDelta[ucCh]);
				if ( m_fVoltage[ucCh]<MINIMUM_VOLTAGE )
					m_fVoltage[ucCh] = MINIMUM_VOLTAGE;
				if ( m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV )
					fCpCrCurrent = m_fPulseCpCr/m_fVoltage[ucCh];	// Watt
				else
					fCpCrCurrent = m_fVoltage[ucCh]/m_fPulseCpCr;	// Resister
				if ( fCpCrCurrent > MAX_SYSTEM_CURRENT)
					fCpCrCurrent = MAX_SYSTEM_CURRENT;
				else
					if ( fCpCrCurrent < -MAX_SYSTEM_CURRENT )
						fCpCrCurrent = -MAX_SYSTEM_CURRENT;
				if ( fCpCrCurrent<0.0f )
				{
					m_ucNowRestChargeDisCharge[ucCh] = m_ucPulseNextState[ucCh] = _NOW_STATE_DISCHARGE;
					s16DacCurrent = GetVoltageCurrentToDacHex( CURRENT_RANGE_INDEX+ucCh*2, -fCpCrCurrent );	// DisCharge Current
					s16DacVoltage = GetVoltageCurrentToDacHex( VOLTAGE_RANGE_INDEX+ucCh, m_pSeqNow[ucCh]->fSettingCurrent  );	// DisCharge Voltage
				}
				else
				{
					m_ucNowRestChargeDisCharge[ucCh] = m_ucPulseNextState[ucCh] = _NOW_STATE_CHARGE;
					s16DacCurrent = GetVoltageCurrentToDacHex( CURRENT_RANGE_INDEX+ucCh*2+1, fCpCrCurrent );	// Charge Current
					s16DacVoltage = GetVoltageCurrentToDacHex( VOLTAGE_RANGE_INDEX+ucCh, m_pSeqNow[ucCh]->fSettingVoltage  );	// Charge Voltage
				}
			}
			SetDacWriteData( _DAC_CH1_CC+ucCh*2, s16DacCurrent );
			SetDacWriteData( _DAC_CH1_CV+ucCh*2, s16DacVoltage );
		}
		return;
	}
}

void CpCrMode( unsigned char ucCh )
{
	if ( m_ucNowMode[ucCh] != _MODE_CPCV && m_ucNowMode[ucCh] != _MODE_CRCV /*&& m_ucNowPulseMode[ucCh] != _PULSE_MODE_CPCV && m_ucNowPulseMode[ucCh] != _PULSE_MODE_CPCV_DP && m_ucNowPulseMode[ucCh] != _PULSE_MODE_CRCV &&*/
	/* m_ucNowPulseMode[ucCh] != _PULSE_MODE_CPCV_CP */ )
		return;
		
	if ( m_uiStepTimeNow[ucCh] < 5 || m_uiStepTimeNow[ucCh] > 0xfffffff0 || ( m_ucNowState[ucCh] == _STATE_PULSE && m_ucPulseKind[ucCh] == _PULSE_TRIANGLE ) )
		return;
//	DEBUG_TP2_H;

	float fCpCr;
	float fCpCrCurrent;
	S16 s16DacCurrent;
	char bLDac = FALSE;
	
	if ( m_bStepRunning[ucCh] )
	{
		if ( m_fVoltage1Sec[ucCh] < MINIMUM_VOLTAGE )
			m_fVoltage1Sec[ucCh] = MINIMUM_VOLTAGE;
			
		if ( m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV_CP || m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV || m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV_DP || m_ucNowPulseMode[ucCh] == _PULSE_MODE_CRCV )
			fCpCr = m_fPulseCurrent[ucCh];
		else
			fCpCr = m_pSeqNow[ucCh]->fSettingCurrent;

        #if 0 //disable jschoi 20250826 
		// bgyu 2025.05.20 CPCV Max watt limit
		if ( fCpCr<-m_pSeqNow[ucCh]->fPulseDisChargeCurrentMax )
			fCpCr = -m_pSeqNow[ucCh]->fPulseDisChargeCurrentMax;
		else
		if ( fCpCr>m_pSeqNow[ucCh]->fPulseChargeCurrentMax )
			fCpCr = m_pSeqNow[ucCh]->fPulseChargeCurrentMax;
		#endif
		
		if (  m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV_CP || m_ucNowMode[ucCh] == _MODE_CPCV || m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV || m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV_DP )
		{
			fCpCrCurrent = fCpCr/m_fVoltage1Sec[ucCh];	// Watt
		}
		else
			fCpCrCurrent = m_fVoltage1Sec[ucCh]/fCpCr;	// Resister
			
		if ( fCpCrCurrent > MAX_SYSTEM_CURRENT )
			fCpCrCurrent = MAX_SYSTEM_CURRENT;
		else
		if ( fCpCrCurrent < -MAX_SYSTEM_CURRENT )
			fCpCrCurrent = -MAX_SYSTEM_CURRENT;
			
		if ( m_uiStepTimeNow[ucCh] < 100 )
		{	
			if ( m_uiStepTimeNow[ucCh] < 10 )
				fCpCrCurrent *= 0.1;	// 10%
			else
			if ( m_uiStepTimeNow[ucCh] < 20 )
				fCpCrCurrent *= 0.91;	// 91%
			else
			if ( m_uiStepTimeNow[ucCh] < 30 )
				fCpCrCurrent *= 0.92;	// 92%
			else
			if ( m_uiStepTimeNow[ucCh] < 40 )
				fCpCrCurrent *= 0.93;	// 93%
			else
			if ( m_uiStepTimeNow[ucCh] < 50 )
				fCpCrCurrent *= 0.94;	// 94%
			else
			if ( m_uiStepTimeNow[ucCh] < 60 )
				fCpCrCurrent *= 0.95;	// 95%
			else
			if ( m_uiStepTimeNow[ucCh] < 70 )
				fCpCrCurrent *= 0.96;	// 96%
			else
			if ( m_uiStepTimeNow[ucCh] < 80 )
				fCpCrCurrent *= 0.97;	// 97%
			else
			if ( m_uiStepTimeNow[ucCh] < 90 )
				fCpCrCurrent *= 0.98;	// 98%
			else
			if ( m_uiStepTimeNow[ucCh] < 100 )
				fCpCrCurrent *= 0.99;	// 99%
		}
		
		// 20221215 DJL Balance, CPCR에서 m_ucNowState 값 오류로 인한 버그 수정
		if ( fCpCrCurrent<0.0f || m_ucNowRestChargeDisCharge[ucCh] == _NOW_STATE_DISCHARGE )
		//if ( fCpCrCurrent<0.0f || m_ucNowState[ucCh] == _STATE_DISCHARGE )
		{
			m_ucPulseNextState[ucCh] = _NOW_STATE_DISCHARGE;
			if ( m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV_DP )
				fCpCrCurrent = m_fChargeCurrent[ucCh]*(m_pSeqNow[ucCh]->fPulseCurrent[1]/100.0f);

			s16DacCurrent = GetVoltageCurrentToDacHex( CURRENT_RANGE_INDEX+ucCh*2, fCpCrCurrent<0.0f ? -fCpCrCurrent : fCpCrCurrent );	// DisCharge Current
		}
		else
		{
			m_ucPulseNextState[ucCh] = _NOW_STATE_CHARGE;
			if ( m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV_CP )
				fCpCrCurrent = m_fDisChargeCurrent[ucCh]*(m_pSeqNow[ucCh]->fPulseCurrent[1]/100.0f);

			s16DacCurrent = GetVoltageCurrentToDacHex( CURRENT_RANGE_INDEX+ucCh*2+1, fCpCrCurrent );	// Charge Current
		}
		SetDacWriteData( _DAC_CH1_CC+ucCh*2, s16DacCurrent );
		bLDac = TRUE;
	}

	if ( bLDac )
	{
		bLDac = FALSE;
/*	bgyu 20250612
		if ( m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV_CP || m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV || m_ucNowPulseMode[ucCh] == _PULSE_MODE_CPCV_DP )
			m_bNewPulseLoad[ucCh] = TRUE;
		else
*/		
			_LDAC_ENABLE;
	}
//	DEBUG_TP2_L;
}

void StepEndCheck( unsigned char ucCh )
{
	//최적화 LJK 2023.04.18
	U8 ucR;
	if ( m_bStepRunning[ucCh] )
	{
		ucR = StepEndConditionCheck( ucCh );
		if ( ucR )
		{
			StepEnd( ucCh, ucR );					
			m_bStepRunning[ucCh] = FALSE;
			m_bContinueNextSequence[ucCh] = 5;	// bgyu 20250528 
			
			if( m_ucNowRestChargeDisCharge[ucCh] == _NOW_STATE_CHARGE )
			{
				SetDacWriteData( _DAC_CH1_CC+ucCh*2, 0x0 );	// bgyu 20250528
				_LDAC_ENABLE;
			}
			else if( m_ucNowRestChargeDisCharge[ucCh] == _NOW_STATE_DISCHARGE )
			{
				SetDacWriteData( _DAC_CH1_CC+ucCh*2, 0x0 );	// bgyu 20250528
				_LDAC_ENABLE;
			}
		}
	}	
}

U8 StepEndConditionCheck( U8 ucCh )
{
	U32 u32EndCondition = m_pSeqNow[ucCh]->u32StepEndConditionsEnable;
	
	if ( !u32EndCondition )
		return _STEP_END_CONDITION_NONE;
	if ( m_uiStepTimeNow[ucCh] < 1000 || m_uiStepTimeNow[ucCh] > 0xfffffff0 )	// 1초 이하 종료 조건 없음
		return 0;
		
	//LJK 2023.03.31 이어서하기 시간이후 1초동안 조건없음
	if( (m_uiStepTimeNow[ucCh] - m_uiPauseTimeNow[ucCh]) < 1000) 
		return 0;  
	
	if ( m_bDCIR_StepStop[ucCh] )
	{
		return _STEP_END_TIME;
	}
	if ( u32EndCondition&(1<<_STEP_END_CONDITION_TIME) )
	{
		if ( (m_uiStepTimeNow[ucCh]-1)/100 >= m_pSeqNow[ucCh]->uiTestSecond32 )
			return _STEP_END_TIME;
	}
	if ( u32EndCondition&(1<<_STEP_END_CONDITION_CV_TIME) )
	{
		if ( m_uiStepCV_TimeNow[ucCh]>=m_pSeqNow[ucCh]->uiCV_msSecond32 )
			return _STEP_END_CV_TIME;
	}
	if ( u32EndCondition&(1<<_STEP_END_CONDITION_CHARGE_VOLTAGE) )
	{
		if ( m_fVoltage1Sec[ucCh]>=m_pSeqNow[ucCh]->StepEndCondition.fChargeVoltage )
			return _STEP_END_CHARGE_VOLTAGE;
	}
	if ( u32EndCondition&(1<<_STEP_END_CONDITION_DISCHARGE_VOLTAGE) )
	{
		if ( m_fVoltage1Sec[ucCh]<=m_pSeqNow[ucCh]->StepEndCondition.fDisChargeVoltage )
			return _STEP_END_DISCHARGE_VOLTAGE;
	}
	if ( u32EndCondition&(1<<_STEP_END_CONDITION_CHARGE_CURRENT) )
	{
		if ( m_fCurrent1Sec[ucCh]<=m_pSeqNow[ucCh]->StepEndCondition.fChargeCurrent )
			return _STEP_END_CHARGE_CURRENT;
	}
	if ( u32EndCondition&(1<<_STEP_END_CONDITION_DISCHARGE_CURRENT) )
	{
		if ( m_fCurrent1Sec[ucCh]<=m_pSeqNow[ucCh]->StepEndCondition.fDisChargeCurrent )
			return _STEP_END_DISCHARGE_CURRENT;
	}
	if ( u32EndCondition&(1<<_STEP_END_CONDITION_CHARGE_CAPACITY) )
	{
		if ( ( m_dblChargeCapacity[ucCh] - m_dblDisChargeCapacity[ucCh] ) >= m_pSeqNow[ucCh]->StepEndCondition.fChargeCapacity )
			return _STEP_END_CHARGE_CAPACITY;
	}
	if ( u32EndCondition&(1<<_STEP_END_CONDITION_DISCHARGE_CAPACITY) )
	{
		if ( ( m_dblDisChargeCapacity[ucCh] - m_dblChargeCapacity[ucCh] ) >= m_pSeqNow[ucCh]->StepEndCondition.fDisChargeCapacity )
			return _STEP_END_DISCHARGE_CAPACITY;
	}
	if ( u32EndCondition&(1<<_STEP_END_CONDITION_CHARGE_WATTHOUR) )
	{
		if ( ( m_dblChargeWattHour[ucCh] - m_dblDisChargeWattHour[ucCh] ) >= m_pSeqNow[ucCh]->StepEndCondition.fChargeWattHour )
			return _STEP_END_CHARGE_WATTHOUR;
	}
	if ( u32EndCondition&(1<<_STEP_END_CONDITION_DISCHARGE_WATTHOUR) )
	{
		if ( ( m_dblDisChargeWattHour[ucCh] - m_dblChargeWattHour[ucCh] ) >= m_pSeqNow[ucCh]->StepEndCondition.fDisChargeWattHour )
			return _STEP_END_DISCHARGE_WATTHOUR;
	}
	if ( u32EndCondition&(1<<_STEP_END_CONDITION_SOC) )
	{
		if ( ( m_dblDisChargeCapacity[ucCh] - m_dblChargeCapacity[ucCh] ) >= m_fSoc[ucCh]  )
			return _STEP_END_SOC;
	}
	if ( u32EndCondition&(1<<_STEP_END_CONDITION_DOD) )
	{
		if ( ( m_dblChargeCapacity[ucCh] - m_dblDisChargeCapacity[ucCh] ) >= m_fDod[ucCh] )
			return _STEP_END_DOD;
	}
	
	// 20230104 djl Cell Temp End Condition Check
	if ( u32EndCondition&(1<<_STEP_END_CONDITION_HIGH_TEMPERATURE) )
	{
		if(m_shCurrentCellTemp[ucCh] > m_pSeqNow[ucCh]->shStepEndTemperatureHigh)
			return _STEP_END_HIGH_TEMPERATURE;
	}
	
	if ( u32EndCondition&(1<<_STEP_END_CONDITION_LOW_TEMERATURE) )
	{
		if(m_shCurrentCellTemp[ucCh] < m_pSeqNow[ucCh]->shStepEndTemperatureLow)
			return _STEP_END_LOW_TEMPERATURE;
	}
	
	return 0;
}
