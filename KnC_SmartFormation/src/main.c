/*
 * main.c
 *
 * Created: 2021-11-26 AM 00:05:00
 *  Author: 4N Corporation, bgyu
 */ 


#include "KnC_IncludeFile.h"
#include "main.h"

__attribute__((__interrupt__))
void    Kpu_B_FatalError(	uint32_t r12,				uint32_t r11,	uint32_t r10,	uint32_t r9,
uint32_t excpiton_number,	uint32_t lr,	uint32_t r17,	uint32_t r6,
uint32_t r5,				uint32_t r4,	uint32_t r3,	uint32_t r2,		uint32_t r1,
uint32_t r0,				uint32_t sp,	uint32_t pc,	uint32_t stack0,	uint32_t stack1,
uint32_t stack2 )
{
}



int main( void )
{
	Disable_global_interrupt();
	INTC_init_interrupts();															// Initializes the hardware interrupt controller driver.

	KpuPinDefineAndInitialize();													// KPU Pin Define & Pin Initialize
	KpuFrequenceClockSetUP();														// KPU Frequence : 7,200,000 Hz
	SDRAM_AddressSet();
	
	m_iPbcHz = sysclk_get_pbc_hz();
	
	KpuSdram16M16BitInitialize();													// External SDRAM 16M * 16Bit Initialize
	
	SystemInfoCleanAndFwVersion(); 													// FW Version
	LoadMcuNumber();
	
	PcCommUsartSetup();																// KPU_A <-> PC		Communication Set Up
	KpuDacSPIUsartSetup();													// Dac SPI Set Up
	KpuAdcSPIUsartSetup();													// Adc SPI Set Up
	SetUpInternelADC();					// 20230113 djl ADC Setting
	
	CaptainTimerCounterSetup( CAPTAIN_TIMER_COUNT_ADDRESS );
	AdcReadCycleTimerCounterSetup( ADC_READ_CYCLE_TIMER_COUNT_ADDRESS );

	SetUpInterruptHandler();														
	EEP_Cal_Data_Read( (uint8_t *)&m_pEEPROM_DATA, sizeof(_EEPROM_DATA) );
  #ifdef SUPPORT_BLACK_OUT
  ReadPauseInfoData();
  #endif
	Enable_global_interrupt();
	TimerInterruptCounterStart();

	DacRest();
	DummyDebugFunction();
	HardWareInitialize();

	//FanControl( 1 );
	FanControl( 0 );
	
	//Boost Mode
	PDC_HIGH = _PORT_C_I_TR_Soft0_Fast1;
	_LED_RUN_ON;	
	
	//ChargeCvTest2();
	
	Dac2Initialize();
	SetDac2WriteData( 0, 0xAF );
	SetDac2WriteData( 1, 0xAF );
	
	
	//CCCV_CP_Test();
	DummyFunctionForMain();
	
}

void Dac2Initialize ( void )
{
	int i;
	volatile int iData;
	
	PDB_HIGH = _PORT_B_POWER_DAC_LAT;
	PDB_LOW = _PORT_B_POWER_DAC_CS_;
	PDC_LOW = _PORT_C_POWER_DAC_SCK;
	
	iData = 0x8 << 19;    // Write address : Vref Register
	iData |= 0x0 << 17;   // Write command
	iData |= 0xf << 0;    // Write data
	
	PDB_LOW = _PORT_B_POWER_DAC_LAT;
	__asm__ __volatile__("nop");
	PDB_LOW = _PORT_B_POWER_DAC_CS_;
	__asm__ __volatile__ ("nop");
	PDC_LOW = _PORT_C_POWER_DAC_SCK;
	
	for(i = 0; i < 24; i++)
	{
		PDC_LOW	= _PORT_C_POWER_DAC_SCK;
		if( iData & (1<<23) )
		{
			PDC_HIGH = _PORT_C_POWER_DAC_SDI;
		}
		else
		{
			PDC_LOW = _PORT_C_POWER_DAC_SDI;
		}
		PDC_HIGH = _PORT_C_POWER_DAC_SCK;
		iData <<= 1;
	}
	PDC_LOW = _PORT_C_POWER_DAC_SCK;
	PDC_LOW = _PORT_C_POWER_DAC_SDI;
	
	PDB_LOW = _PORT_B_POWER_DAC_CS_;
	__asm__ __volatile__("nop");
	PDB_LOW = _PORT_B_POWER_DAC_LAT;
	__asm__ __volatile__("nop");
	
	PDB_HIGH = _PORT_B_POWER_DAC_CS_;
	__asm__ __volatile__("nop");
	PDB_HIGH = _PORT_B_POWER_DAC_LAT;
	__asm__ __volatile__("nop");	
}

void CCCV_CP_Test( void )
{
	
	U8 i = 0;
	int channel = 0;
	_FORMATIONM_STEP_SEQUENCE* pStepSeq;

	m_ucStartStepIndex[channel] = 0;
	m_uiAlarmMask[channel] = 1 << _STEP_ALARM_MASK_VOLTAGE_FIX_ERROR |
				1 << _STEP_ALARM_MASK_MY_CHANNEL | 1 <<	_STEP_ALARM_MASK_OTHER_CHANNEL |
				1 << _STEP_ALARM_MASK_VOLTAGE_FAST_DOWN | 1 << _STEP_ALARM_MASK_VOLTAGE_FAST_UP |
				1 << _STEP_ALARM_MASK_REST_OVP | 1 << _STEP_ALARM_MASK_REST_UVP;
				
//DCIR Sec Test Seq
	pStepSeq = &m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[channel][i++];
	pStepSeq->ucLoopIndex = 0;
	pStepSeq->usLoopCountNow = 0;
	pStepSeq->usLoopCountMax = 0; //1 번 Balance
	pStepSeq->ucState = _STATE_DCIR;
	pStepSeq->ucMode = _MODE_Sec_DCIR;// _MODE_10ms_DCIR;
	pStepSeq->fSettingVoltage = 4.2f;
	pStepSeq->fSettingCurrent = 2.5f;	// Pulse시 Discharge Voltage
	pStepSeq->u32StepEndConditionsEnable = (1<<_STEP_END_CONDITION_TIME);
	pStepSeq->ucSocDodIndex = 0;
	pStepSeq->uiTestSecond32 = 55; //5.5sec
	pStepSeq->uiDataRecordSecond32 = 1;		
	
	pStepSeq->ucPulseMax = 2;	// Pulse Point #
	pStepSeq->ucPulseMode = _PULSE_MODE_CCCV;	// _PULSE_MODE_CCCV...
	pStepSeq->ucPulseKind[0] = _PULSE_RECTANGLE;	// Rectangle/Triangle
	pStepSeq->ucPulseKind[1] = _PULSE_RECTANGLE;	// Rectangle/Triangle
	pStepSeq->u16PulseTime1ms[0] = 3*1000;	// 1ms Pulse 유지 시간들
	pStepSeq->u16PulseTime1ms[1] = 15;	// 1ms Pulse 유지 시간들
	pStepSeq->fPulseCurrent[0] = 0.5;		// DCIR의 경우 0=Start 1초동안 전류, Pulse, +값은 Chargae, -값은 DisCharge
	pStepSeq->fPulseCurrent[1] = -20;		// DCIR의 경우 0=Start 1초동안 전류, Pulse, +값은 Chargae, -값은 DisCharge
	pStepSeq->fPulseChargeUp1Sec = 0;	// 2Point Pulse시 1초당 Charge Current Up Delta Up
	pStepSeq->fPulseChargeCurrentMax = 50;;	// Charge Delta Up 절대치 최대
	pStepSeq->fPulseDisChargeUp1Sec = 0;	// 2Point Pulse시 1초당 DisCharge Current Up Delta Down
	pStepSeq->fPulseDisChargeCurrentMax = 50;// DisCharge Delta Up 절대치 최대				

	pStepSeq = &m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[channel][i];
	pStepSeq->ucState = _STATE_NONE;

	m_bRunSequence[0] = true;
		
}

void DummyOn( void )
{
	_CH1_DISCHARGE_ENABLE_OFF;
	_CH1_DISCHARGE_ENABLE_OFF;
	_CH1_MODE_CHARGE;
	SetDacWriteData( 0, 0x7fff );
	_LDAC_ENABLE;
	delay_ms( 500 );
	_CH1_MC_ON; //SMY
	delay_ms( 500 );
	_CH1_CHARGE_ENABLE_ON;
	SetDacWriteData( 0, 0xff00 );
	SetDacWriteData( 1, 0x9b00 );
	_LDAC_ENABLE;
}

void DummyDebugFunction( void )
{
}

void DacRest( void )
{
	unsigned char i;
	for( i=0; i<8; i++ )
		SetDacWriteData( i, 0 );
	_LDAC_ENABLE;
}


void ChargeDischargeDirectTest( void )
{
	int j;
	FanControl( 1 );
	for(;;)
	{
		_CH1_DISCHARGE_ENABLE_OFF;
		_CH1_CHARGE_ENABLE_OFF;
		_CH1_MODE_DISCHARGE;
		SetDacWriteData( 0, 0xf000 );
		SetDacWriteData( 1, 0x0000 );
		_LDAC_ENABLE;
		delay_ms( 500 );
		_CH4_MC_ON;
		delay_ms( 500 );
		_CH1_DISCHARGE_ENABLE_ON;
		
		for( j=0; j<10; )
		{
			SetDacWriteData( 0, 0x23b8 );	// 24A
			SetDacWriteData( 1, 0xac38 );	// 2.5V
//			SetDacWriteData( 1, 0xeaab );	// 0.9V
//			SetDacWriteData( 0, 0x3000 );
//			SetDacWriteData( 1, 0xac38 );
			_LDAC_ENABLE;
			_CH1_CHARGE_ENABLE_OFF;
			_CH1_DISCHARGE_ENABLE_ON;
			_CH1_MODE_DISCHARGE;
			delay_ms( 14 );
//			delay_ms( 36 );

//			SetDacWriteData( 0, 0x0e00 );
//			_LDAC_ENABLE;
//			delay_ms( 14 );
//			continue;
/*			
			SetDacWriteData( 0, 0xc000 );
			SetDacWriteData( 1, 0x9e59 );
			_LDAC_ENABLE;
			_CH1_DISCHARGE_ENABLE_OFF;
			_CH1_CHARGE_ENABLE_ON;
			_CH1_MODE_CHARGE;
			delay_ms( 36 );
			*/
		}
		_CH1_DISCHARGE_ENABLE_OFF;
		_CH1_CHARGE_ENABLE_OFF;
		break;
	}
}

void DischargeDirectTest( void )
{
	int i;
	FanControl( 1 );
	for(;;)
	{
		_CH1_DISCHARGE_ENABLE_OFF;
		_CH1_CHARGE_ENABLE_OFF;
		_CH1_MODE_DISCHARGE;
		SetDacWriteData( 0, 0xf000 );
		SetDacWriteData( 1, 0x0000 );
		_LDAC_ENABLE;
		delay_ms( 500 );
		_CH4_MC_ON;
		delay_ms( 500 );
		_CH1_DISCHARGE_ENABLE_ON;
		
		for(;;)
		{
			for( i=1000; i<0x7fff; i+=1000 )
			{
				SetDacWriteData( 0, i );
				_LDAC_ENABLE;
				delay_ms( 36 );
				SetDacWriteData( 0, 0x0000 );
				_LDAC_ENABLE;
				delay_ms( 1000 );
			}
		}
	}
	for(;;)
	{
		_CH4_DISCHARGE_ENABLE_OFF;
		_CH4_CHARGE_ENABLE_OFF;
		_CH4_MODE_DISCHARGE;
		SetDacWriteData( 6, 0x7fff );
		SetDacWriteData( 7, 0x8000 );
		_LDAC_ENABLE;
		_CH4_DISCHARGE_ENABLE_ON;
		
		for(;;)
		{
			SetDacWriteData( 6, 0x1000 );
			SetDacWriteData( 7, 0xc000 );
			_LDAC_ENABLE;
			delay_us( 1000 );
			SetDacWriteData( 6, 0x3000 );
			SetDacWriteData( 7, 0xf000 );
			_LDAC_ENABLE;
			delay_us( 1000 );
		}
	}
}

void ChargeCvTest( void )
{
	FanControl( 1 );
	_CH1_DISCHARGE_ENABLE_OFF;
	_CH1_CHARGE_ENABLE_OFF;
	_CH1_MODE_CHARGE;
	SetDacWriteData( 0, 0xd000 );
	SetDacWriteData( 1, 0x7f00 );
	_LDAC_ENABLE;
	delay_ms( 500 );
	_CH4_MC_ON;
	delay_ms( 500 );
	_CH1_CHARGE_ENABLE_ON;
	delay_us( 200 );

	int i;
	int j;
	int k;
	float fVoltage[] = 
	{
		0.5,
		1.0,
		1.5,
		2.0,
		2.5,
		3.0,
		3.5,
		4.0,
		4.5,
		5.0,
		5.2,
		4.2,
		3.6,
		/*
		0.5069392,
		0.5069373,
		1.5058153,
		1.5058193,
		2.0053763,
		2.0053791,
		2.4049546,
		2.4049584,
		2.8045918,
		2.8046038,
		3.2041902,
		3.2041964,
		3.6038064,
		3.6038105,
		4.0035390,
		4.0035376,
		4.4031443,
		4.4031419,
		4.8027534,
		4.8027472,
		5.2023730,
		5.2023768,
		*/
	};
	float fRecordVoltage[sizeof(fVoltage)/sizeof(float)*2];
	float fRecordVoltageDiff[sizeof(fVoltage)/sizeof(float)*2];
	int iCounter[sizeof(fVoltage)/sizeof(float)*2];
	int iVoltage1SecTotal[sizeof(fVoltage)/sizeof(float)*2];
	short shVoltage[sizeof(fVoltage)/sizeof(float)*2];
	float fPer[sizeof(fVoltage)/sizeof(float)*2];
	
	//float fT1 = GetHexToVoltageCurrent( 0, 0x038E0740 );
	for( i=0; i<sizeof(fVoltage)/sizeof(float); i++ )
	{
		for( j=0; j<2; j++ )
		{
			k = i*2+j;
			if ( k==34 )
				k++;
			shVoltage[i*2+j] = GetVoltageCurrentToDacHex( 0, fVoltage[i] );
			SetDacWriteData( 1, shVoltage[i*2+j] );
			_LDAC_ENABLE;
			delay_ms( 200 );
			//m_ui1SecRingCountVoltage = m_ui1SecRingCountCurrent = m_ui1SecRingCountVoltage2 = 0;
			//delay_ms( 1010 );
			delay_ms( 1500 );
			fRecordVoltage[i*2+j] = m_fVoltage1Sec[0];
			iCounter[i*2+j] = m_ui1SecRingCountVoltage[0];
			fRecordVoltageDiff[i*2+j] = fVoltage[i]-m_fRecordVoltage[0];
			iVoltage1SecTotal[i*2+j] = m_iVoltage1SecTotal[0];
			fPer[i*2+j] = fRecordVoltageDiff[i*2+j]/5.0f*100.0f;
		}
	}
	_LDAC_ENABLE;
}


void ChargeCvTest2( void )
{
	FanControl( 1 );
	_CH1_DISCHARGE_ENABLE_OFF;
	_CH1_CHARGE_ENABLE_OFF;
	_CH1_MODE_CHARGE;
	SetDacWriteData( 0, 0xd000 );
	SetDacWriteData( 1, 0x7f00 );
	_LDAC_ENABLE;
	delay_ms( 500 );
	_CH4_MC_ON;
	delay_ms( 500 );
	_CH1_CHARGE_ENABLE_ON;
	delay_us( 200 );

	int i;
	int j;
	short shVoltage[] = 
	{
		0xF45C,
		0xF45C,
		0xDD17,
		0xDD17,
		0xD174,
		0xD174,
		0xC825,
		0xC825,
		0xBED6,
		0xBED6,
		0xB587,
		0xB587,
		0xAC38,
		0xAC38,
		0xA2E8,
		0xA2E8,
		0x9999,
		0x9999,
		0x904A,
		0x904A,
		0x86FB,
		0x86FB,
	};
	int iVoltage1SecTotal[sizeof(shVoltage)/sizeof(short)*2];
	for( i=0; i<sizeof(shVoltage)/sizeof(short); i++ )
	{
		SetDacWriteData( 1, shVoltage[i] );
		_LDAC_ENABLE;
		for( j=0; j<2; j++ )
		{
			delay_ms( 1010 );
			iVoltage1SecTotal[i*2+j] = m_iVoltage1SecTotal[0];
		}
	}
	_LDAC_ENABLE;
}

void ChargeDirectTest( void )
{
	FanControl( 1 );
	unsigned short i;
	for(;0;)	// Cv
	{
		_CH1_DISCHARGE_ENABLE_OFF;
		_CH1_CHARGE_ENABLE_OFF;
		_CH1_MODE_CHARGE;
		SetDacWriteData( 0, 0xe000 );
		SetDacWriteData( 1, 0x7f00 );
		_LDAC_ENABLE;
		delay_ms( 500 );
		_CH4_MC_ON;
		delay_ms( 500 );
		_CH1_CHARGE_ENABLE_ON;
		delay_us( 200 );
		SetDacWriteData( 1, 0x9e80 );
		_LDAC_ENABLE;
		
		for(;;)
		{
			for( i=0xffff; i>0x8000; i -=1000 )
			{
				SetDacWriteData( 1, i );
				_LDAC_ENABLE;
				delay_ms( 50 );
				SetDacWriteData( 1, 0x0000 );
				_LDAC_ENABLE;
				delay_ms( 500 );
			}
		}
	}
	for(;;)	// CC
	{
		_CH1_DISCHARGE_ENABLE_OFF;
		_CH1_CHARGE_ENABLE_OFF;
		_CH1_MODE_CHARGE;
		SetDacWriteData( 0, 0x7fff );
		SetDacWriteData( 1, 0x9e80 );
		_LDAC_ENABLE;
		delay_ms( 500 );
		_LDAC_ENABLE;
		_CH4_MC_ON;
		delay_ms( 500 );
		_CH1_CHARGE_ENABLE_ON;
		
		for( i=0; i<2; i++ )
		{
			SetDacWriteData( 0, 0x8000 );
			_LDAC_ENABLE;
			delay_ms( 36 );
			SetDacWriteData( 0, 0x0000 );
			_LDAC_ENABLE;
			delay_ms( 1000 );
		}
		return;
	}
	for(;;)
	{
		_CH1_DISCHARGE_ENABLE_OFF;
		_CH1_CHARGE_ENABLE_OFF;
		_CH1_MODE_CHARGE;
		SetDacWriteData( 0, 0x1000 );
		SetDacWriteData( 1, 0xf000 );
		_LDAC_ENABLE;
		delay_ms( 500 );
		_CH4_MC_ON;
		delay_ms( 500 );
		_CH1_CHARGE_ENABLE_ON;
		
		for(;;)
		{
			SetDacWriteData( 0, 0xf000 );
			SetDacWriteData( 1, 0xa000 );
			_LDAC_ENABLE;
			delay_us( 5000 );
			SetDacWriteData( 0, 0xfe00 );
			SetDacWriteData( 1, 0xe000 );
			_LDAC_ENABLE;
			delay_ms( 1000 );
		}
	}
}

void DummyFunctionForMain(void)
{
	for(;;)
	{
		if(m_bHardWareInit)
		{
			m_bHardWareInit = FALSE;
			HardWareInitialize();
		}			
		
		//m_uiSdramCheckEnalbe = 1;
		if( m_uiSdramCheckEnalbe == TRUE )
		{
			for( int i=0; i<7; i++ )
			{
				m_uiSdramCheckPassFail = 0x1001+i;
				if ( !CheckSdramPatternTest( i ) )
				{
					m_uiSdramCheckEnalbe = FALSE;
					m_uiSdramCheckPassFail = 0x4444;			// Fail
					break;
				}
				if ( 0 )
				{
					short i, j;
					for( i=-32700; i<+32700; i++ )
						for( j=0; j<8; j++ )
							SetDacWriteData( j, i );
				}
			}
			if( m_uiSdramCheckPassFail != 0x4444)			
				m_uiSdramCheckPassFail = 0x9999;				// Pass
				
			m_uiSdramCheckEnalbe = FALSE;
			m_uiSdramCheckEnalbe = FALSE;
		}
    #ifdef SUPPORT_BLACK_OUT
    PauseResumeControl();
    #endif
		EEPRom_Write_Control();
		EEPRom_Read_Control();
		LedStatusSet();
		m_uiSystemInformation[_CPUResetCause] = reset_cause_get_causes();
		//ChargePowerControl();
	}
}

void EEPRom_Write_Control( void )
{
  #ifdef SUPPORT_BLACK_OUT
  unsigned short i;
  #endif
  
	if ( m_ucEEPRomControl == _EEPRom_WRITE_ALL )
	{
		AVR32_ENTER_CRITICAL_REGION();
		EEP_Cal_Data_Write( (uint8_t *)&m_pEEPROM_DATA, sizeof(_EEPROM_DATA) );
		m_ucEEPRomControl = _EEPRom_READ_ALL;
		AVR32_LEAVE_CRITICAL_REGION();
	}
  #ifdef SUPPORT_BLACK_OUT
  else if ( m_ucEEPRomControl == _EEPRom_WRITE_PAUSE_SYS_RESUME_INFO )
  {
    AVR32_ENTER_CRITICAL_REGION();
		EEP_PauseInfo_Data_Write( (uint8_t *)&m_pEEPROM_PAUSE_INFO_DATA, sizeof(PAUSE_INFO_DATA) );
		m_ucEEPRomControl = _EEPRom_NONE;
		AVR32_LEAVE_CRITICAL_REGION();
  }
  else if ( m_ucEEPRomControl == _EEPRom_WRITE_CLEAR_PAUSE_INFO )
  {
    AVR32_ENTER_CRITICAL_REGION();
		EEP_PauseInfo_Data_Write( (uint8_t *)&m_pEEPROM_PAUSE_INFO_DATA, sizeof(PAUSE_INFO_DATA) );
		m_ucEEPRomControl = _EEPRom_NONE;
		AVR32_LEAVE_CRITICAL_REGION();
  }
  #endif
}

void EEPRom_Read_Control( void )
{
	if ( m_ucEEPRomControl == _EEPRom_READ_ALL )
	{
		AVR32_ENTER_CRITICAL_REGION();
		EEP_Cal_Data_Read( (uint8_t *)&m_pEEPROM_DATA, sizeof(_EEPROM_DATA) );
		m_ucEEPRomControl = _EEPRom_NONE;
		AVR32_LEAVE_CRITICAL_REGION();
	}
}
#ifdef SUPPORT_BLACK_OUT
void ReadPauseInfoData ( void )
{
  unsigned int size = sizeof(PAUSE_INFO_DATA);
  unsigned char checkSum = 0;
  unsigned int i = 0;
  unsigned char *pArr = NULL;
  
  memset(&m_pEEPROM_PAUSE_INFO_DATA, 0, sizeof(PAUSE_INFO_DATA));
  EEP_PauseInfo_Data_Read( (uint8_t *)&m_pEEPROM_PAUSE_INFO_DATA, sizeof(PAUSE_INFO_DATA) );

  pArr = (unsigned char *)&m_pEEPROM_PAUSE_INFO_DATA;

  for (i = 0; i < (size-1); i++)
  {
    checkSum ^= pArr[i];
  }

  if (checkSum != m_pEEPROM_PAUSE_INFO_DATA.checkSum)
  {
    memset(&m_pEEPROM_PAUSE_INFO_DATA, 0, sizeof(PAUSE_INFO_DATA));
  }
  else
  {
    for (i = 0; i < _MAX_CHANNEL; i++)
    {
      if (m_pEEPROM_PAUSE_INFO_DATA.ucIsPauseDataValid[i] == 1)
      {  
        //Load Step parameters
        m_uiStepTimeNow[i] = m_pEEPROM_PAUSE_INFO_DATA.uiStepTimeNow[i];
        m_uiStepCV_TimeNow[i] = m_pEEPROM_PAUSE_INFO_DATA.uiStepCV_TimeNow[i];
        
        //m_ucNowRestChargeDisCharge[i] = m_pEEPROM_PAUSE_INFO_DATA.ucNowRestChargeDisCharge[i];
        m_ucPauseStatus[i] = m_pEEPROM_PAUSE_INFO_DATA.ucPauseStatus[i];
        m_ucStartStepIndex[i] = m_pEEPROM_PAUSE_INFO_DATA.ucStartStepIndex[i];
        m_ucStepIndexNow[i] = m_ucStartStepIndex[i];
        
        m_dblChargeCapacity[i] = m_pEEPROM_PAUSE_INFO_DATA.dblChargeCapacity[i];
        m_dblDisChargeCapacity[i] = m_pEEPROM_PAUSE_INFO_DATA.dblDisChargeCapacity[i];
        m_dblChargeWattHour[i] = m_pEEPROM_PAUSE_INFO_DATA.dblChargeWattHour[i];
        m_dblDisChargeWattHour[i] = m_pEEPROM_PAUSE_INFO_DATA.dblDisChargeWattHour[i];

        m_uiPauseStepTimeNow[i] = m_pEEPROM_PAUSE_INFO_DATA.uiPauseStepTimeNow[i];
        m_u16PausePulseTime1msNow[i] = m_pEEPROM_PAUSE_INFO_DATA.u16PausePulseTime1msNow[i];
        m_u16PulseTime1msNow[i] = m_pEEPROM_PAUSE_INFO_DATA.u16PulseTime1msNow[i];
        m_fPulseRefCurrent[i] = m_pEEPROM_PAUSE_INFO_DATA.fPulseRefCurrent[i];
        //m_fPulseChargeCurrentPreStep[i] = m_pEEPROM_PAUSE_INFO_DATA.fPulseChargeCurrentPreStep[i];  
        //m_fPulseDisChargeCurrentPreStep[i] = m_pEEPROM_PAUSE_INFO_DATA.fPulseDisChargeCurrentPreStep[i];  
        m_fPulseCurrentPre[i] = m_pEEPROM_PAUSE_INFO_DATA.fPulseCurrentPre[i];   
        //m_fPulseCurrent[i] = m_pEEPROM_PAUSE_INFO_DATA.fPulseCurrent[i];         
        m_ucPulseIndex[i] = m_pEEPROM_PAUSE_INFO_DATA.ucPulseIndex[i];
      }
    }
  }
}

void SetPauseInfoData ( unsigned char ucCh )
{
  m_pEEPROM_PAUSE_INFO_DATA.ucIsPauseDataValid[ucCh] = 1;
  m_pEEPROM_PAUSE_INFO_DATA.ucIsPauseDataWritten[ucCh] = 0;
   
  m_pEEPROM_PAUSE_INFO_DATA.uiStepTimeNow[ucCh] = m_uiStepTimeNow[ucCh];
  m_pEEPROM_PAUSE_INFO_DATA.uiStepCV_TimeNow[ucCh] = m_uiStepCV_TimeNow[ucCh];
  
  //m_pEEPROM_PAUSE_INFO_DATA.ucNowRestChargeDisCharge[ucCh] = m_ucNowRestChargeDisCharge[ucCh];
  m_pEEPROM_PAUSE_INFO_DATA.ucPauseStatus[ucCh] = m_ucPauseStatus[ucCh];
  m_pEEPROM_PAUSE_INFO_DATA.ucStartStepIndex[ucCh] = m_ucStartStepIndex[ucCh];  

  m_pEEPROM_PAUSE_INFO_DATA.dblChargeCapacity[ucCh] = m_dblChargeCapacity[ucCh];
  m_pEEPROM_PAUSE_INFO_DATA.dblDisChargeCapacity[ucCh]= m_dblDisChargeCapacity[ucCh];
  m_pEEPROM_PAUSE_INFO_DATA.dblChargeWattHour[ucCh] = m_dblChargeWattHour[ucCh];
  m_pEEPROM_PAUSE_INFO_DATA.dblDisChargeWattHour[ucCh] = m_dblDisChargeWattHour[ucCh];

  //PULSE
  m_pEEPROM_PAUSE_INFO_DATA.uiPauseStepTimeNow[ucCh] = m_uiPauseStepTimeNow[ucCh];
  m_pEEPROM_PAUSE_INFO_DATA.u16PausePulseTime1msNow[ucCh] = m_u16PausePulseTime1msNow[ucCh];
  m_pEEPROM_PAUSE_INFO_DATA.u16PulseTime1msNow[ucCh] = m_u16PulseTime1msNow[ucCh];
  m_pEEPROM_PAUSE_INFO_DATA.ucPulseIndex[ucCh] = m_ucPulseIndex[ucCh];
  m_pEEPROM_PAUSE_INFO_DATA.fPulseRefCurrent[ucCh] = m_fPulseRefCurrent[ucCh];
  //m_pEEPROM_PAUSE_INFO_DATA.fPulseChargeCurrentPreStep[ucCh] = m_fPulseChargeCurrentPreStep[ucCh];
  //m_pEEPROM_PAUSE_INFO_DATA.fPulseDisChargeCurrentPreStep[ucCh] = m_fPulseDisChargeCurrentPreStep[ucCh];
  m_pEEPROM_PAUSE_INFO_DATA.fPulseCurrentPre[ucCh] = m_fPulseCurrentPre[ucCh];
  //m_pEEPROM_PAUSE_INFO_DATA.fPulseCurrent[ucCh] = m_fPulseCurrent[ucCh];
}
#endif

void LoadMcuNumber( void )
{
	m_ucMcuNumber = 
		( gpio_get_pin_value(_CH_ID_00&0xFFF) << 0 ) |
		( gpio_get_pin_value(_CH_ID_01&0xFFF) << 1 ) |
		( gpio_get_pin_value(_CH_ID_02&0xFFF) << 2 ) |
		( gpio_get_pin_value(_CH_ID_03&0xFFF) << 3 ) |
		( gpio_get_pin_value(_CH_ID_04&0xFFF) << 4 ) |
		( gpio_get_pin_value(_CH_ID_05&0xFFF) << 5 ) ;
	m_ucMcuNumber = m_ucMcuNumber & 0x3f;
}

void EEPRomDummySet( void )
{
	U8 i;
	m_pEEPROM_DATA.uiCalibrationDate[0] = 0x11111111;
	m_pEEPROM_DATA.uiCalibrationDate[1] = 0x22222222;
	m_pEEPROM_DATA.uiCalibrationDate[2] = 0x33333333;
	m_pEEPROM_DATA.uiCalibrationDate[3] = 0x44444444;
	for( i=0; i<4; i++ )
	{
		m_pEEPROM_DATA.fContactSpec[0][i] = i*10+1;
		m_pEEPROM_DATA.fContactSpec[1][i] = i*100+1;
		m_pEEPROM_DATA.fContactCalibration[0][i] = 1000*i+1;
		m_pEEPROM_DATA.fContactCalibration[1][i] = 2000*i+1;
	}
	for( i=0; i<11; i++ )
	{
		m_pEEPROM_DATA.s16Dac_ChargePower[0][i] = 0x3000 | i;
		m_pEEPROM_DATA.s16Dac_ChargePower[1][i] = 0x4000 | i;
	}
	for( i=0; i<12; i++ )
	{
		m_pEEPROM_DATA.ucMaxStep[i] = 1+i*2;
	}
	for( i=0; i<11; i++ )
	{
		m_pEEPROM_DATA.iAdc[0][i] = 0x1000|i;
		m_pEEPROM_DATA.iAdc[1][i] = 0x2000|i;
		m_pEEPROM_DATA.iAdc[2][i] = 0x3000|i;
		m_pEEPROM_DATA.iAdc[3][i] = 0x4000|i;
		m_pEEPROM_DATA.iAdc[4][i] = 0x5000|i;
		m_pEEPROM_DATA.iAdc[5][i] = 0x6000|i;
		m_pEEPROM_DATA.iAdc[6][i] = 0x7000|i;
		m_pEEPROM_DATA.iAdc[7][i] = 0x8000|i;
		m_pEEPROM_DATA.iAdc[8][i] = 0x9000|i;
		m_pEEPROM_DATA.iAdc[9][i] = 0xa000|i;
		m_pEEPROM_DATA.iAdc[10][i] = 0xb000|i;
		m_pEEPROM_DATA.iAdc[11][i] = 0xc000|i;
		
		m_pEEPROM_DATA.s16Dac[0][i] = 0x100|i;
		m_pEEPROM_DATA.s16Dac[1][i] = 0x200|i;
		m_pEEPROM_DATA.s16Dac[2][i] = 0x300|i;
		m_pEEPROM_DATA.s16Dac[3][i] = 0x400|i;
		m_pEEPROM_DATA.s16Dac[4][i] = 0x500|i;
		m_pEEPROM_DATA.s16Dac[5][i] = 0x600|i;
		m_pEEPROM_DATA.s16Dac[6][i] = 0x700|i;
		m_pEEPROM_DATA.s16Dac[7][i] = 0x800|i;
		m_pEEPROM_DATA.s16Dac[8][i] = 0x900|i;
		m_pEEPROM_DATA.s16Dac[9][i] = 0xa00|i;
		m_pEEPROM_DATA.s16Dac[10][i] = 0xb00|i;
		m_pEEPROM_DATA.s16Dac[11][i] = 0xc00|i;

		m_pEEPROM_DATA.fReferance[0][i] = i*0.5+0.5;
		m_pEEPROM_DATA.fReferance[1][i] = i*0.5+0.5+1000;
		m_pEEPROM_DATA.fReferance[2][i] = i*0.5+0.5+2000;
		m_pEEPROM_DATA.fReferance[3][i] = i*0.5+0.5+3000;
		m_pEEPROM_DATA.fReferance[4][i] = i*0.5+0.5+4000;
		m_pEEPROM_DATA.fReferance[5][i] = i*0.5+0.5+5000;
		m_pEEPROM_DATA.fReferance[6][i] = i*0.5+0.5+6000;
		m_pEEPROM_DATA.fReferance[7][i] = i*0.5+0.5+7000;
		m_pEEPROM_DATA.fReferance[8][i] = i*0.5+0.5+8000;
		m_pEEPROM_DATA.fReferance[9][i] = i*0.5+0.5+9000;
		m_pEEPROM_DATA.fReferance[10][i] = i*0.5+0.5+10000;
		m_pEEPROM_DATA.fReferance[11][i] = i*0.5+0.5+11000;
	}
}

void memcpy2( char *pDest, char *pSource, unsigned char ucSize )
{
	unsigned char i;
	for( i=0; i<ucSize; i++ )
	{
		*pDest = *pSource;
		pDest++;
		pSource++;		
	}
}
