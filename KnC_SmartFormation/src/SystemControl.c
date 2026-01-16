/*
 * SystemControl.c
 *
 * Created: 2021-11-26 AM 00:05:00
 *  Author: KnC Corporation, bgyu
 */ 


#include "KnC_IncludeFile.h"


void HardWareInitialize( void )
{
	unsigned char ucChannel=0;

	for(ucChannel=0; ucChannel< _MAX_CHANNEL; ucChannel++)
		if(m_bChannelRunning[ucChannel] == TRUE)
			return;

	InitializeVariable();
	PowerUp_First_Initialize();
}

void InitializeVariable( void )
{
	unsigned char ucChannel=0;
	
	for(ucChannel=0; ucChannel< _MAX_CHANNEL; ucChannel++)
	{
		m_ui1SecRingCountVoltage[ucChannel] = m_ui1SecRingCountCurrent[ucChannel] = m_ui1SecRingCountVoltage2[ucChannel] = 0;
		m_uiRecordSecond[ucChannel] = 1;
		m_uiRecordMilliSecond[ucChannel] = 1000;
		//LJK 2024.07.01 Cell Temp
		m_shCurrentCellTemp[ucChannel] = 0;	//ADM Temp Default 0
		m_shMaxCellTemp[ucChannel] = 700;	//Max Cell Temp 70.0		
		m_bHeatSinkTemperaturError[ucChannel] = FALSE;	//LJK 2024.07.17
		m_bTrTf_Delay[ucChannel] = FALSE;			//LJK 2025.03.11
		m_fHeatSinkTemp[ucChannel] = DEFAULT_HEAT_SINK_TEMP;	// LJK 2025.04.25
		m_fAirTemp = DEFAULT_AIR_TEMP;							// LJK 2025.04.25		

        #ifdef SUPPORT_PROTECTION_CONDITION
        m_pProtectionType[ucChannel].fAirTemp      = DEFAULT_AIR_TEMP;
        m_pProtectionType[ucChannel].fCellTemp     = DEFAULT_CELL_TEMP * 10;
		m_pProtectionType[ucChannel].fHeatSinkTemp = DEFAULT_HEAT_SINK_TEMP;
		#endif
	}
		
	m_bFanError = m_bAirTemperaturError = FALSE;
}

void PowerUp_First_Initialize( void )
{
	unsigned char ucChannel;
	
	// DAC ADC Reset 
	_ADC_RESET_ENABLE;
	_DAC_RESET_ENABLE;
	
	// Voltage2 Mode Enable4
	for(ucChannel=0; ucChannel<_MAX_CHANNEL; ucChannel++)
	{
		gpio_set_pin_high(m_uiVoltage2ModeAddress[ucChannel]);	
		gpio_set_pin_high(m_uiChargePullupAddress[ucChannel]);	//LJK 2023.04.04
		m_bVoltage2Mode[ucChannel] = TRUE;
		StepSequence_ClearLoopCounterAll(ucChannel);
	}

	//20221228 DJL	
	_CH1_CHARGE_ENABLE_OFF;		//_CH1_CHARGE_ENABLE_ON;
	_CH1_DISCHARGE_ENABLE_OFF;
	_CH2_CHARGE_ENABLE_OFF;		//_CH2_CHARGE_ENABLE_ON;
	_CH2_DISCHARGE_ENABLE_OFF;
	_CH3_CHARGE_ENABLE_OFF;		//_CH3_CHARGE_ENABLE_ON;
	_CH3_DISCHARGE_ENABLE_OFF;
	_CH4_CHARGE_ENABLE_OFF;		//_CH4_CHARGE_ENABLE_ON;
	_CH4_DISCHARGE_ENABLE_OFF;

	_CH1_MC_OFF;
	_CH2_MC_OFF;
	_CH3_MC_OFF;
	_CH4_MC_OFF;	
}

void McControl( unsigned char ucCh )
{
	U8 ucLdac = FALSE;
	U8 bMcControlAllCompleteCheck = FALSE;
	
	if ( m_uiMcChangeDelay[ucCh] != 0 )
	{
		if ( m_uiMcChangeDelay[ucCh] == MC_CONTROL_MAX_TIME )
		{
			m_bMcControlAllComplete[ucCh] = FALSE;
			if ( m_ucStepIndexNow[ucCh] >= _MAX_SEQUENCE_STEP || m_bMcOff[ucCh] )
			{
				m_bMcOnOffNow[ucCh] = 0;
				m_bMcOff[ucCh] = 0;
			}
			else
			{
				if ( m_pSeqNow[ucCh]->ucState == _STATE_REST )
					//m_bMcOnOffNow[ucCh] = ( m_pSeqNow[ucCh]->ucMode == _MODE_PHYSICAL_REST ) ? 0: m_bPreMcOnOff[ucCh];
					m_bMcOnOffNow[ucCh] = ( m_pSeqNow[ucCh]->ucMode == _MODE_PHYSICAL_REST ) ? 0: 1;	//LJK 2025.01.17 Bug
				else
					m_bMcOnOffNow[ucCh] = 1;
			}
			if ( m_bMcOnOffNow[ucCh] == m_bPreMcOnOff[ucCh] )
				m_uiMcChangeDelay[ucCh] = 2;
			else
				m_bPreMcOnOff[ucCh] = m_bMcOnOffNow[ucCh];
		}
		m_uiMcChangeDelay[ucCh]--;
		if ( m_uiMcChangeDelay[ucCh] == MC_CONTROL_TIME )
		{
			if ( m_bMcOnOffNow[ucCh] )
			{
				switch( ucCh )
				{
					case 0:
						_CH1_MC_ON;
						SetDacWriteData( _DAC_CH1_CC, 0x0000 );	// bgyu 20250528
						_LDAC_ENABLE;
						break;
					case 1:
						_CH2_MC_ON;
						SetDacWriteData( _DAC_CH2_CC, 0x0000 );	// bgyu 20250528
						_LDAC_ENABLE;
						break;						
					case 2:
						_CH3_MC_ON;
						SetDacWriteData( _DAC_CH3_CC, 0x0000 );	// bgyu 20250528
						_LDAC_ENABLE;
						break;						
					case 3:
						_CH4_MC_ON;
						SetDacWriteData( _DAC_CH4_CC, 0x0000 );	// bgyu 20250528
						_LDAC_ENABLE;
						break;
					/*					
					case 1:
						_CH2_MC_ON;
						if ( m_ucNowRestChargeDisCharge[ucCh] == _NOW_STATE_CHARGE )
							SetDacWriteData( 2, 0x1000 );
						else
						if ( m_ucNowRestChargeDisCharge[ucCh] == _NOW_STATE_DISCHARGE )
							SetDacWriteData( 2, 0xf000 );
						else
							SetDacWriteData( 2, 0x0000 );
						break;
					case 2:
						_CH3_MC_ON;
						if ( m_ucNowRestChargeDisCharge[ucCh] == _NOW_STATE_CHARGE )
							SetDacWriteData( 4, 0x1000 );
						else
						if ( m_ucNowRestChargeDisCharge[ucCh] == _NOW_STATE_DISCHARGE )
							SetDacWriteData( 4, 0xf000 );
						else
							SetDacWriteData( 4, 0x0000 );
						break;
					case 3:
						_CH4_MC_ON;
						if ( m_ucNowRestChargeDisCharge[ucCh] == _NOW_STATE_CHARGE )
							SetDacWriteData( 6, 0x1000 );
						else
						if ( m_ucNowRestChargeDisCharge[ucCh] == _NOW_STATE_DISCHARGE )
							SetDacWriteData( 6, 0xf000 );
						else
							SetDacWriteData( 6, 0x0000 );
						break;
					*/
				}
				
				ucLdac = TRUE;
			}
			else
			{
				switch( ucCh )
				{
					case 0:	
						DacReset( ucCh );	// bgyu 20250528
						PortReset( ucCh );
						_LDAC_ENABLE;
						_CH1_MC_OFF;
						break;
					case 1:
						DacReset( ucCh );	// bgyu 20250528
						PortReset( ucCh );
						_LDAC_ENABLE;
						_CH2_MC_OFF;
						break;
					case 2:
						DacReset( ucCh );	// bgyu 20250528
						PortReset( ucCh );
						_LDAC_ENABLE;
						_CH3_MC_OFF;
						break;
					case 3:
						DacReset( ucCh );	// bgyu 20250528
						PortReset( ucCh );
						_LDAC_ENABLE;
						_CH4_MC_OFF;
						break;																
					/*
					case 1:	
						SetDacWriteData( 2, 0x0000 );
						_LDAC_ENABLE;
						_CH2_MC_OFF;	break;
					case 2:	
						SetDacWriteData( 4, 0x0000 );
						_LDAC_ENABLE;
						_CH3_MC_OFF;	break;
					case 3: 
						SetDacWriteData( 6, 0x0000 );
						_LDAC_ENABLE;
						_CH4_MC_OFF;	break;
					*/
				}
			}
		}
		// 2023.04.04 LJK MC 제어 후 Pull-up
		else if(m_uiMcChangeDelay[ucCh] == 0 )
		{
			// 20230111 djl MC 제어 후 Pull-up, 2023.04.18 LJK m_ucNowState == _STATE_NONE
			if(m_bMcOnOffNow[ucCh] == FALSE && m_ucNowState[ucCh] == _STATE_NONE )
			{
				gpio_set_pin_high(m_uiChargePullupAddress[ucCh]);
				/*
				switch (ucCh)
				{
					case 0:
						_CH1_CHARGE_ENABLE_ON;
						break;
					case 1:
						_CH2_CHARGE_ENABLE_ON;
						break;
					case 2:
						_CH3_CHARGE_ENABLE_ON;
						break;
					case 3:
						_CH4_CHARGE_ENABLE_ON;
						break;
				}
				*/
			}
			
		}
		bMcControlAllCompleteCheck =TRUE;
	}
	
	if ( !m_bMcControlAllComplete[ucCh] && bMcControlAllCompleteCheck)
	{
		m_bMcControlAllComplete[ucCh] = TRUE;
		if ( m_bChannelRunning[ucCh] && m_uiMcChangeDelay[ucCh] != 1)
		{
			m_bMcControlAllComplete[ucCh] = FALSE;
		}
	}
	if ( ucLdac )
	{
		ucLdac = FALSE;		
		_LDAC_ENABLE;
	}
}

void StepEnd( unsigned char ucCh, int ucEndKind )
{
	m_bAlarmParameterSet[ucCh] = FALSE;
	m_ucNowPulseMode[ucCh] = _PULSE_MODE_NONE;
	m_ucNowMode[ucCh] = _MODE_NONE;
	//기록조건과 시간에의한 종료 조건이 같을때 2개기록 막음
	if((m_uiStepTimeNow[ucCh]-1) % m_uiRecordMilliSecond[ucCh] != 0)
	{
	    
        #ifdef MODIFY_DCIR_10MSEC_MODE
		if((m_pSeqNow[ucCh]->ucMode == _MODE_10ms_DCIR) && (m_ucNowRestChargeDisCharge[0] == _NOW_STATE_DISCHARGE) && (ucEndKind == _STEP_END_TIME))
        {
            /* RawData 기록 안되도록 disable 시킴 */           
            //PushStepRecordData( ucCh, ucEndKind );
        }
        else
		#endif
		{
		    PushStepRecordData( ucCh, ucEndKind );
		}
    }
    
	SaveStepEndData( ucCh, ucEndKind );
}


void FormationEnd( unsigned char ucCh )
{
	m_bStepStarted[ucCh] = FALSE;
	m_bChannelRunning[ucCh] = FALSE;
	
	// 20221206 djl 추가
	m_ucNowState[ucCh] = _STATE_NONE;
	
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

	MC_Off( ucCh );
	
	if ( m_bChannelRunning[ucCh] )
		return;

	// Voltage2 Mode 
	gpio_set_pin_high(m_uiVoltage2ModeAddress[ucCh]);
	m_bVoltage2Mode[ucCh] = TRUE;
	
	//ChxChargePullUp
	gpio_set_pin_high(m_uiChargePullupAddress[ucCh]);	//LJK 2023.04.04
	
	FanControl( FALSE );

	m_fHeatSinkTemp[ucCh] = DEFAULT_HEAT_SINK_TEMP;	// LJK 2025.04.25
	m_fAirTemp = DEFAULT_AIR_TEMP;					// LJK 2025.04.25	
}

void MC_Off( unsigned char ucCh )
{
	m_uiMcChangeDelay[ucCh] = MC_CONTROL_MAX_TIME;
	m_bMcOff[ucCh] = TRUE;	// MC Off
}

void DacReset( unsigned char ucCh )
{
	SetDacWriteData( ucCh*2+0, 0 );
	SetDacWriteData( ucCh*2+1, 0 );
	_LDAC_ENABLE;
	// 20230111 djl pulse end시 슈트 방지
	//delay_us(10);	// bgyu 20250528
}

void PortReset( unsigned char ucCh )
{
	switch( ucCh )
	{
		case 0:	_CH1_CHARGE_ENABLE_OFF; _CH1_DISCHARGE_ENABLE_OFF;	break;
		case 1:	_CH2_CHARGE_ENABLE_OFF; _CH2_DISCHARGE_ENABLE_OFF;	break;
		case 2:	_CH3_CHARGE_ENABLE_OFF; _CH3_DISCHARGE_ENABLE_OFF;	break;
		case 3:	_CH4_CHARGE_ENABLE_OFF; _CH4_DISCHARGE_ENABLE_OFF;	break;
	}
}

bool SetDacWriteData( int iDacChannel, unsigned int uiData )
{
	uiData &= 0xffff;
	int iRetry = 3;	// 1time 60us, V I * 2 = 4, 4*12=720us Max(Max Error)
	int i;

	for( i=0; i<iRetry;i++ )
	{
		if ( SetDacWriteDataRetry( iDacChannel, uiData ) )
			return true;
		m_uiSystemInformation[_DacWriteFatalError]++;
	}
	PushStepRecordData( iDacChannel, _STEP_DAC_ERROR );
	PortReset( (unsigned char)iDacChannel );
	return false;
}

bool SetDac2WriteData( int iDacChannel, unsigned int uiData )
{
	int i;
	volatile int iData;
	
	if ( ( iDacChannel < 0 ) || ( iDacChannel > 1 ) )
		return false;
	
	PDB_LOW = _PORT_B_POWER_DAC_LAT;
	PDB_LOW = _PORT_B_POWER_DAC_CS_;
	PDC_LOW = _PORT_C_POWER_DAC_SCK;
	
	iData = iDacChannel << 19;    // Write address : 0 or 1
	iData |= 0x0 << 17;   // Write command
	iData |= uiData << 0;    // Write data
	
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
	
	PDB_LOW = _PORT_B_POWER_DAC_CS_;
	__asm__ __volatile__("nop");
	PDB_LOW = _PORT_B_POWER_DAC_LAT;
	__asm__ __volatile__("nop");
	
	PDB_HIGH = _PORT_B_POWER_DAC_CS_;
	__asm__ __volatile__("nop");
	PDB_HIGH = _PORT_B_POWER_DAC_LAT;
	__asm__ __volatile__("nop");
	
	return true;
}

void SetDacSpiWriteBits( unsigned long ucDacSdiWriteData )
{
	Union32 data;
	volatile int i;
	data.u32 = 	ucDacSdiWriteData;
	
	DAC_SPI->mr |= AVR32_SPI_MR_PCS_MASK;
	if (DAC_SPI->mr & AVR32_SPI_MR_PCSDEC_MASK)
		DAC_SPI->mr &= ~AVR32_SPI_MR_PCS_MASK |	(0 << AVR32_SPI_MR_PCS_OFFSET);
	else
		DAC_SPI->mr &= ~(1 << (AVR32_SPI_MR_PCS_OFFSET + 0));

	for( i=0; i<DAC_SPI_WAIT_LOOPN&&!spi_is_tx_ready(DAC_SPI); i++ );
		DAC_SPI->tdr = ( data.u8[1] << AVR32_SPI_TDR_TD_OFFSET);
	for( i=0; i<DAC_SPI_WAIT_LOOPN&&!spi_is_tx_ready(DAC_SPI); i++ );
		DAC_SPI->tdr = ( data.u8[2] << AVR32_SPI_TDR_TD_OFFSET);
	for( i=0; i<DAC_SPI_WAIT_LOOPN&&!spi_is_tx_ready(DAC_SPI); i++ );
		DAC_SPI->tdr = ( data.u8[3] << AVR32_SPI_TDR_TD_OFFSET);

	DAC_SPI->mr |= AVR32_SPI_MR_PCS_MASK;
	DAC_SPI->cr = AVR32_SPI_CR_LASTXFER_MASK;
}

int GetDacSpiReadBits( void )
{
	unsigned int timeout;
	timeout = SPI_TIMEOUT;
	Union32 data;
	data.u32 = 	0;
	volatile int i;
	
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	DAC_SPI->mr |= AVR32_SPI_MR_PCS_MASK;												// DAC_SYNC LOW
	if (DAC_SPI->mr & AVR32_SPI_MR_PCSDEC_MASK)
		DAC_SPI->mr &= ~AVR32_SPI_MR_PCS_MASK |	(0 << AVR32_SPI_MR_PCS_OFFSET);
	else
		DAC_SPI->mr &= ~(1 << (AVR32_SPI_MR_PCS_OFFSET + 0));

	for( i=0; i<DAC_SPI_WAIT_LOOPN&&!spi_is_tx_ready(DAC_SPI); i++ );
	

	DAC_SPI->tdr = CONFIG_SPI_MASTER_DUMMY << AVR32_SPI_TDR_TD_OFFSET;					// spi_write_single(spi,CONFIG_SPI_MASTER_DUMMY);
	for( i=0; i<DAC_SPI_WAIT_LOOPN&&!spi_is_rx_ready(DAC_SPI); i++ );
		data.u8[1] = DAC_SPI->rdr >> AVR32_SPI_RDR_RD_OFFSET;								// spi_read_single(spi,&val);
	DAC_SPI->tdr = CONFIG_SPI_MASTER_DUMMY << AVR32_SPI_TDR_TD_OFFSET;					// spi_write_single(spi,CONFIG_SPI_MASTER_DUMMY);
	for( i=0; i<DAC_SPI_WAIT_LOOPN&&!spi_is_rx_ready(DAC_SPI); i++ );
		data.u8[2] = DAC_SPI->rdr >> AVR32_SPI_RDR_RD_OFFSET;								// spi_read_single(spi,&val);
	DAC_SPI->tdr = CONFIG_SPI_MASTER_DUMMY << AVR32_SPI_TDR_TD_OFFSET;					// spi_write_single(spi,CONFIG_SPI_MASTER_DUMMY);
	for( i=0; i<DAC_SPI_WAIT_LOOPN&&!spi_is_rx_ready(DAC_SPI); i++ );
		data.u8[3] = DAC_SPI->rdr >> AVR32_SPI_RDR_RD_OFFSET;								// spi_read_single(spi,&val);
	
	DAC_SPI->tdr = CONFIG_SPI_MASTER_DUMMY << AVR32_SPI_TDR_TD_OFFSET;	 // add edit
	DAC_SPI->mr |= AVR32_SPI_MR_PCS_MASK;												// DAC_SYNC HIGH
	DAC_SPI->cr = AVR32_SPI_CR_LASTXFER_MASK;
	return data.u32;
}

bool SetDacWriteDataRetry( int iDacChannel, unsigned int uiData )
{
	int iSdiModeBits = ( (_DAC_MODE_WRITE&0x3)<<22 );
	int iSdiAddrBits = ( (0x3&iDacChannel)<<16 )|((iDacChannel<4 ? 1 : 2)<<19);
	int iSdiData = uiData & 0xffff;
	int iDacSdiWriteData;

	iDacSdiWriteData = 	 iSdiModeBits|iSdiAddrBits|iSdiData;

	SetDacSpiWriteBits( iDacSdiWriteData );

	m_iDacReadBack = ReadDacData( iDacChannel, uiData );
	if ( m_iDacReadBack!=uiData )
	{
		m_uiSystemInformation[_DacWriteRetryError]++;
		return FALSE;
	}
	return TRUE;
}



#define		_DAC_MODE_READ	(0b00000101<<16)

int ReadDacData( int iDacChannel, int iWritedData )
{
	PDB_HIGH = _PORT_B_DAC_LDAC;


	int iSdiModeBits = _DAC_MODE_READ;
	int iDacSdiWriteData=0;
	int iReadBackChannel;

	if ( iDacChannel>3 )
		iReadBackChannel = (0b010000|(iDacChannel&0x3))<<7;
	else
		iReadBackChannel = (0b001000|(iDacChannel&0x3))<<7;

	iDacSdiWriteData = 	 iSdiModeBits|iReadBackChannel;
	SetDacSpiWriteBits( iDacSdiWriteData );
	
	return  GetDacSpiReadBits();
}


void PushStepRecordData( unsigned char ucCh, unsigned char ucEndKind )
{
	_FORMATIONM_STEP_RECORD_DATA* p;
	if ( ((u32RecordTail[ucCh]+1)%_FORMATIONM_STEP_RECORD_MAX) == u32RecordHead[ucCh] )	// Full ?
		u32RecordHead[ucCh] = (u32RecordHead[ucCh]+1)%_FORMATIONM_STEP_RECORD_MAX;	// Data 1 Lost...
	
	p = & m_pFORMATION_STEP_RECORD_DATA_STRUCTURE->stRecord[ucCh][u32RecordTail[ucCh]];
	//2023.04.25 LJK
	p->ucRcodrdType = _NOR_STEP_RECORD;
	p->usLoopCountNow = m_pSeqNow[ucCh]->usLoopCountNow;
	p->ucEndCondition = 0;

	p->ucStepNow = m_ucStepIndexNow[ucCh];
	
	// 20230120 djl RunTime
	
	#ifdef MODIFY_DCIR_10MSEC_MODE
	if((m_pSeqNow[ucCh]->ucMode == _MODE_10ms_DCIR) && (m_ucNowRestChargeDisCharge[0] == _NOW_STATE_DISCHARGE))
    {
        p->uiNowmsSecond32 = m_uiStepTimeNow[ucCh] == 0 ? 0 :  (m_uiStepTimeNow[ucCh] - 2);
    }
    else
    #endif
    {
	    p->uiNowmsSecond32 = m_uiStepTimeNow[ucCh] == 0 ? 0 :  m_uiStepTimeNow[ucCh] - 1;
	}
	
	p->uiCV_msSecond32 = m_uiStepCV_TimeNow[ucCh];
	
	p->StepEndCondition.fChargeCapacity = (float)m_dblChargeCapacity[ucCh];
	p->StepEndCondition.fDisChargeCapacity = (float)m_dblDisChargeCapacity[ucCh];
	p->StepEndCondition.fChargeWattHour = (float)m_dblChargeWattHour[ucCh];
	p->StepEndCondition.fDisChargeWattHour = (float)m_dblDisChargeWattHour[ucCh];
		
	if ( m_ucNowMode[ucCh] == _MODE_2POINT_PULSE_CHARGE || m_ucNowMode[ucCh] == _MODE_2POINT_PULSE_DISCHARGE || m_ucNowMode[ucCh] == _MODE_10POINT_PULSE_CHARGE 
		|| m_ucNowMode[ucCh] == _MODE_10POINT_PULSE_DISCHARGE || m_ucNowMode[ucCh] == _MODE_Sec_DCIR || m_ucNowMode[ucCh] == _MODE_10ms_DCIR )
	{
		p->StepEndCondition.fChargeVoltage = m_fChargeVoltage[ucCh];
		p->StepEndCondition.fDisChargeVoltage = m_fDisChargeVoltage[ucCh];
		p->StepEndCondition.fChargeCurrent = m_fChargeCurrent[ucCh];
		p->StepEndCondition.fDisChargeCurrent = m_fDisChargeCurrent[ucCh];
		if( ucEndKind == _STEP_TESTING ) // LJK 2023.04.07 스텝의 첫번째 레코드 전압
		{
			p->StepEndCondition.fChargeVoltage = m_fStepStartVoltage[ucCh];
			p->StepEndCondition.fDisChargeVoltage = m_fStepStartVoltage[ucCh];
			/*
			//LJK 2024.04.05 DischargeCurrent, ChargeCurrent 처음시작시 0값일때 설정값으로 적용
			if( m_fChargeCurrent[ucCh] == 0)
			{
				m_fChargeCurrent[ucCh] = m_pSeqNow[ucCh]->fPulseCurrent[0];
				p->StepEndCondition.fChargeCurrent = m_fChargeCurrent[ucCh];
			}
			if(m_fDisChargeCurrent[ucCh] == 0)
			{
				m_fDisChargeCurrent[ucCh] = m_pSeqNow[ucCh]->fPulseCurrent[0];
				p->StepEndCondition.fDisChargeCurrent = m_fDisChargeCurrent[ucCh];
			}
			*/
		}		
		#ifdef MODIFY_DCIR_10MSEC_MODE
        else if(ucEndKind == 0)
        {
            if((m_ucNowMode[ucCh] == _MODE_10ms_DCIR) && (m_uiStepTimeNow[ucCh] == 3002))
            {
			    p->StepEndCondition.fDisChargeVoltage = 0.0f;
		        p->StepEndCondition.fDisChargeCurrent = 0.0f;
            }
        }
        #endif	
	}
	else
	{
		if( ucEndKind == 0 ) //2023 03 16 LJK 정상기록 조건일때 현재 V/I 사용
		{
			p->StepEndCondition.fChargeVoltage =
			p->StepEndCondition.fDisChargeVoltage = m_fRecordVoltage[ucCh];
			p->StepEndCondition.fChargeCurrent =
			p->StepEndCondition.fDisChargeCurrent = m_fRecordCurrent[ucCh];
			//2025.02.04 LJK 1초레코딩 V/I 평균값 낮음, 1초일때만 현재 V/I 적용
			if(m_uiStepTimeNow[ucCh] <= 2004)
			{
				p->StepEndCondition.fChargeVoltage = m_fNowVoltage[ucCh];
				p->StepEndCondition.fDisChargeVoltage = m_fNowVoltage[ucCh];
				p->StepEndCondition.fChargeCurrent = m_fNowCurrent[ucCh];
				p->StepEndCondition.fDisChargeCurrent = m_fNowCurrent[ucCh];
			}			
		}
		else
		if( ucEndKind == _STEP_TESTING ) // LJK 2023.04.07 스텝의 첫번째 레코드 전압/젆류
		{
			//p->StepEndCondition.fChargeVoltage = m_fStepStartVoltage[ucCh];
			//p->StepEndCondition.fDisChargeVoltage = m_fStepStartVoltage[ucCh];
			//현재전압으로 2025.01.17
			p->StepEndCondition.fChargeVoltage = m_fNowVoltage[ucCh];
			p->StepEndCondition.fDisChargeVoltage = m_fNowVoltage[ucCh];
			
			p->StepEndCondition.fChargeCurrent = m_fChargeCurrent[ucCh];
			p->StepEndCondition.fDisChargeCurrent = m_fDisChargeCurrent[ucCh];			
		}		
		else //2023 03 16 LJK EndCondition, USER, PAUSE 조건일때 현재 V/I 사용
		{
			p->StepEndCondition.fChargeVoltage = m_fChargeVoltage[ucCh];
			p->StepEndCondition.fDisChargeVoltage = m_fDisChargeVoltage[ucCh];
			p->StepEndCondition.fChargeCurrent = m_fChargeCurrent[ucCh];
			p->StepEndCondition.fDisChargeCurrent = m_fDisChargeCurrent[ucCh];
		}

		//2023.04.12 LJK REST 전류 0 추가
		//2024.03.28 LJK REST Ah, Wh 0 추가
		if( m_ucNowState[ucCh] == _STATE_REST )
		{
			p->StepEndCondition.fChargeCurrent = p->StepEndCondition.fDisChargeCurrent = 0.0;
			p->StepEndCondition.fChargeCapacity = p->StepEndCondition.fDisChargeCapacity = 0.0;
			p->StepEndCondition.fChargeWattHour = p->StepEndCondition.fDisChargeWattHour = 0.0;
		}		
		//2023 03 08 LJK
		if( m_ucNowState[ucCh] == _STATE_CHARGE || (m_ucNowState[ucCh] == _STATE_BALANCE && m_ucNowRestChargeDisCharge[ucCh] == _NOW_STATE_CHARGE ))
			p->StepEndCondition.fDisChargeVoltage = p->StepEndCondition.fDisChargeCurrent = 0.0;
		if( m_ucNowState[ucCh] == _STATE_DISCHARGE || (m_ucNowState[ucCh] == _STATE_BALANCE && m_ucNowRestChargeDisCharge[ucCh] == _NOW_STATE_DISCHARGE ))	
			p->StepEndCondition.fChargeVoltage = p->StepEndCondition.fChargeCurrent = 0.0;
		
	}

	u32RecordTail[ucCh] = (u32RecordTail[ucCh]+1)%_FORMATIONM_STEP_RECORD_MAX;
}

int GetStepRecordRemainNumber( unsigned char ucCh )
{
	int iRet;
	if ( u32RecordHead[ucCh]==u32RecordTail[ucCh] )
		return 0;
	if ( u32RecordTail[ucCh]<u32RecordHead[ucCh] )
		iRet = u32RecordTail[ucCh]+_FORMATIONM_STEP_RECORD_MAX-u32RecordHead[ucCh];
	else
		iRet = u32RecordTail[ucCh]-u32RecordHead[ucCh];
	return iRet;
}

_FORMATIONM_STEP_RECORD_DATA* GetStepRecordHeadPointer( unsigned char ucCh )
{
	_FORMATIONM_STEP_RECORD_DATA* p;
	
	p = & m_pFORMATION_STEP_RECORD_DATA_STRUCTURE->stRecord[ucCh][u32RecordHead[ucCh]];
	return p;
}

void DeletePopStepRecordDataAll( unsigned char ucCh )
{
	u32RecordHead[ucCh] = u32RecordTail[ucCh] = 0;
}

U32 DeletePopStepRecordData( unsigned char ucCh )
{
	if ( u32RecordHead[ucCh]==u32RecordTail[ucCh] )
		return 0;
	u32RecordHead[ucCh] = (u32RecordHead[ucCh]+1)%_FORMATIONM_STEP_RECORD_MAX;

	return GetStepRecordNumber( ucCh );
}

U32 GetStepRecordNumber( unsigned char ucCh )
{
	if ( u32RecordTail[ucCh]>=u32RecordHead[ucCh] )
		return u32RecordTail[ucCh]-u32RecordHead[ucCh];
	return (u32RecordTail[ucCh]+_FORMATIONM_STEP_RECORD_MAX)-u32RecordHead[ucCh];
}

void MakeStepDataOne_Channel( unsigned char ucCh, unsigned char ucEndCondition )
{
	_FORMATIONM_STEP_END_DATA* p;
	p = & m_DataNow[ucCh];
	p->ucEndCondition = ucEndCondition;
	p->uiTotalTestmsSecond32 = m_uiStepTimeNow[ucCh] == 0 ? 0 :  m_uiStepTimeNow[ucCh] - 1;
	p->uiCV_msSecond32 = m_uiStepCV_TimeNow[ucCh];
	
	p->StepEndCondition.fChargeVoltage = m_fChargeVoltage[ucCh];
	p->StepEndCondition.fDisChargeVoltage = m_fDisChargeVoltage[ucCh];

	#ifdef MODIDY_MONITORING_DATA		
	//In the idle state, display by 0 .
	if(m_ucNowState[ucCh] == _STATE_NONE)
	{
	    if(m_ucNowRestChargeDisCharge[ucCh] == _NOW_STATE_CHARGE)
	    {
        	p->StepEndCondition.fChargeVoltage = m_fChargeVoltage[ucCh];
        	p->StepEndCondition.fDisChargeVoltage = 0.0f;
	    }
	    else if(m_ucNowRestChargeDisCharge[ucCh] == _NOW_STATE_DISCHARGE)
	    {
        	p->StepEndCondition.fChargeVoltage = 0.0f;
        	p->StepEndCondition.fDisChargeVoltage = m_fDisChargeVoltage[ucCh];
	    }
	
    	p->StepEndCondition.fChargeCurrent     = 0.0f;
    	p->StepEndCondition.fDisChargeCurrent  = 0.0f;
    	p->StepEndCondition.fChargeCapacity    = 0.0f;
    	p->StepEndCondition.fDisChargeCapacity = 0.0f;
    	p->StepEndCondition.fChargeWattHour    = 0.0f;
    	p->StepEndCondition.fDisChargeWattHour = 0.0f;
    }
	#else	
	p->StepEndCondition.fChargeCurrent = m_fChargeCurrent[ucCh];
	p->StepEndCondition.fDisChargeCurrent = m_fDisChargeCurrent[ucCh];

	//2023 03 30 LJK Stop/Pause 마지막데이터 V/I
	if( m_ucNowState[ucCh] == _STATE_NONE && m_ucNowRestChargeDisCharge[ucCh] == _NOW_STATE_CHARGE )
		p->StepEndCondition.fDisChargeVoltage = p->StepEndCondition.fDisChargeCurrent = 0.0;
	if( m_ucNowState[ucCh] == _STATE_NONE && m_ucNowRestChargeDisCharge[ucCh] == _NOW_STATE_DISCHARGE )
		p->StepEndCondition.fChargeVoltage = p->StepEndCondition.fChargeCurrent = 0.0;


	p->StepEndCondition.fChargeCapacity = m_dblChargeCapacity[ucCh];
	p->StepEndCondition.fDisChargeCapacity = m_dblDisChargeCapacity[ucCh];
	p->StepEndCondition.fChargeWattHour = m_dblChargeWattHour[ucCh];
	p->StepEndCondition.fDisChargeWattHour = m_dblDisChargeWattHour[ucCh];
	#endif
}

void SaveStepEndData( unsigned char ucCh, unsigned char ucEndCondition )
{
	_FORMATIONM_STEP_RECORD_DATA* p;
	if ( ((u32RecordTail[ucCh]+1)%_FORMATIONM_STEP_RECORD_MAX) == u32RecordHead[ucCh] )	// Full ?
	u32RecordHead[ucCh] = (u32RecordHead[ucCh]+1)%_FORMATIONM_STEP_RECORD_MAX;	// Data 1 Lost...
	
	p = & m_pFORMATION_STEP_RECORD_DATA_STRUCTURE->stRecord[ucCh][u32RecordTail[ucCh]];
	p->ucRcodrdType = _END_STEP_RECORD;
	p->usLoopCountNow = m_pSeqNow[ucCh]->usLoopCountNow;
	p->ucEndCondition = ucEndCondition;
	p->ucStepNow = m_ucStepIndexNow[ucCh];
	
	#ifdef MODIFY_DCIR_10MSEC_MODE
	if((m_pSeqNow[ucCh]->ucMode == _MODE_10ms_DCIR) && (m_ucNowRestChargeDisCharge[0] == _NOW_STATE_DISCHARGE))
    {
	    p->uiNowmsSecond32 = m_uiStepTimeNow[ucCh] == 0 ? 0 :  m_uiStepTimeNow[ucCh] - 4;
	}
	else
	#endif
	{
	    p->uiNowmsSecond32 = m_uiStepTimeNow[ucCh] == 0 ? 0 :  m_uiStepTimeNow[ucCh] - 1;
	}
	
	p->uiCV_msSecond32 = m_uiStepCV_TimeNow[ucCh];


	p->StepEndCondition.fChargeVoltage = m_fChargeVoltage[ucCh];
	p->StepEndCondition.fDisChargeVoltage = m_fDisChargeVoltage[ucCh];
	p->StepEndCondition.fChargeCurrent = m_fChargeCurrent[ucCh];
	p->StepEndCondition.fDisChargeCurrent = m_fDisChargeCurrent[ucCh];

	p->StepEndCondition.fChargeWattHour = m_dblChargeWattHour[ucCh];
	p->StepEndCondition.fDisChargeWattHour = m_dblDisChargeWattHour[ucCh];

	//2023.04.12 LJK REST 전류 0 처리 추가
	if( m_ucNowState[ucCh] == _STATE_REST )
	{
		p->StepEndCondition.fChargeCurrent = p->StepEndCondition.fDisChargeCurrent = 0.0;
	}
	if( m_ucNowState[ucCh] == _STATE_CHARGE || (m_ucNowState[ucCh] == _STATE_BALANCE && m_ucNowRestChargeDisCharge[ucCh] == _NOW_STATE_CHARGE ))
		p->StepEndCondition.fDisChargeVoltage = p->StepEndCondition.fDisChargeCurrent = 0.0;
	if( m_ucNowState[ucCh] == _STATE_DISCHARGE || (m_ucNowState[ucCh] == _STATE_BALANCE && m_ucNowRestChargeDisCharge[ucCh] == _NOW_STATE_DISCHARGE ))
		p->StepEndCondition.fChargeVoltage = p->StepEndCondition.fChargeCurrent = 0.0;


	if ( m_bDCIR_StepStop[ucCh] )
	{
		p->StepEndCondition.fChargeCapacity = m_fDCIR_Voltage1[ucCh]-m_fDCIR_Voltage2[ucCh][11];	// DCIR시는 변화된 전압
		p->StepEndCondition.fDisChargeCapacity = 0;	// DCIR시는 적분된 전류
		if( m_pSeqNow[ucCh]->ucMode == _MODE_10ms_DCIR)
		{
			for( U8 i=2; i<12; i++ )
				p->StepEndCondition.fDisChargeCapacity += m_fDCIR_Current[ucCh][i];
			p->StepEndCondition.fDisChargeCapacity /= 10.0f;			
		}
		else if( m_pSeqNow[ucCh]->ucMode == _MODE_Sec_DCIR )
		{
			for( U8 i=0; i < m_pSecDcirBuffer->usDCIR_Count[ucCh]; i++ )
				p->StepEndCondition.fDisChargeCapacity += m_pSecDcirBuffer->fDCIR_Current[ucCh][i];
			p->StepEndCondition.fDisChargeCapacity /= m_pSecDcirBuffer->usDCIR_Count[ucCh];			
			
		}
		
		// 20230119 djl * 1000 추가(단위 통일)
		p->StepEndCondition.fChargeWattHour = (p->StepEndCondition.fChargeCapacity/p->StepEndCondition.fDisChargeCapacity) * 1000.0f;
		
		#ifdef MODIFY_DCIR_STEP_DATA
		p->StepEndCondition.fChargeVoltage     = m_fDCIR_Voltage1[ucCh];
		p->StepEndCondition.fDisChargeVoltage  = m_fDCIR_Voltage2[ucCh][11];
		p->StepEndCondition.fDisChargeCurrent  = p->StepEndCondition.fDisChargeCapacity;
		p->StepEndCondition.fDisChargeWattHour = 0;		
		#endif
	}
	else
	{
		// _MODE_CHARGE_CONTACT
		// fChargeCapacity = Error Resistor
		// return :			p->StepEndCondition.fChargeCapacity = V2
		//					p->StepEndCondition.fDisChargeCapacity = Contact Resistor
		
		// 20230126 djl 버그수정 (ReadyToStartForSafety에서 걸리면 쓰레기값 Write)
		if ( m_pSeqNow[ucCh]->ucMode == _MODE_CHARGE_CONTACT && m_uiStepTimeNow[ucCh] > 1000 )
		{
			p->StepEndCondition.fChargeCapacity = m_fVoltage2_1Sec[ucCh];
			p->StepEndCondition.fDisChargeCapacity = (m_fVoltage2_1Sec[ucCh]-m_fVoltage1Sec[ucCh])/m_fCurrent1Sec[ucCh]*1000.0f;
			if ( m_pSeqNow[ucCh]->u32StepEndConditionsEnable & (1<<_STEP_END_CONDITION_CONTACT_RESISTANCE) )
			{
				if ( p->StepEndCondition.fDisChargeCapacity < 0.0f || p->StepEndCondition.fDisChargeCapacity>m_pSeqNow[ucCh]->StepEndCondition.fChargeCapacity )
				{
					p->ucEndCondition = _TEST_END_CONTACT_OVER_RESISTANCE;
					FormationEnd( ucCh );
				}
			}
		}
		else
		if ( m_pSeqNow[ucCh]->ucMode == _MODE_DISCHARGE_CONTACT && m_uiStepTimeNow[ucCh] > 1000 )
		{
			p->StepEndCondition.fChargeCapacity = m_fVoltage2_1Sec[ucCh];
			p->StepEndCondition.fDisChargeCapacity = (m_fVoltage1Sec[ucCh]-m_fVoltage2_1Sec[ucCh])/m_fCurrent1Sec[ucCh]*1000.0f;
			if ( m_pSeqNow[ucCh]->u32StepEndConditionsEnable & (1<<_STEP_END_CONDITION_CONTACT_RESISTANCE) )
			{
				if ( p->StepEndCondition.fDisChargeCapacity < 0.0f || p->StepEndCondition.fDisChargeCapacity>m_pSeqNow[ucCh]->StepEndCondition.fChargeCapacity )
				{
					p->ucEndCondition = _TEST_END_CONTACT_OVER_RESISTANCE;
					FormationEnd( ucCh );
				}
			}
		}
		else
		{
			p->StepEndCondition.fChargeCapacity = m_dblChargeCapacity[ucCh];	// DCIR시는 변화된 전압
			p->StepEndCondition.fDisChargeCapacity = m_dblDisChargeCapacity[ucCh];	// DCIR시는 적분된 전류
		}
	}
	
	u32RecordTail[ucCh] = (u32RecordTail[ucCh]+1)%_FORMATIONM_STEP_RECORD_MAX;
	
	//2023.04.26 LJK (SOC, DOD stStepEndData에 복사
	memcpy(&m_pFORMATION_STEP_END_DATA_STRUCTURE->stStepEndData[ucCh][m_ucStepIndexNow[ucCh]].StepEndCondition, &p->StepEndCondition, sizeof(_RESULT_DATA));
	m_pFORMATION_STEP_END_DATA_STRUCTURE->stStepEndData[ucCh][m_ucStepIndexNow[ucCh]].uiTotalTestmsSecond32 = p->uiNowmsSecond32;
	
	//2024.10.25 LJK SoC, DoD Adaptive
	if( m_pSeqNow[ucCh]->ucSocDodIndex != 0 )
	{
		m_pFORMATION_STEP_END_DATA_STRUCTURE->stStepEndData[ucCh][m_pSeqNow[ucCh]->ucSocDodIndex].usLoopCountNow = m_pSeqNow[ucCh]->usLoopCountNow;
		m_pFORMATION_STEP_END_DATA_STRUCTURE->stStepEndData[ucCh][m_ucStepIndexNow[ucCh]].usLoopCountNow = m_pSeqNow[ucCh]->usLoopCountNow;
	}
}

void ClearStepEndData( unsigned char ucCh, unsigned char ucStep )
{
	memset(&m_pFORMATION_STEP_END_DATA_STRUCTURE->stStepEndData[ucCh][ucStep], 
		0, 
		sizeof(m_pFORMATION_STEP_END_DATA_STRUCTURE->stStepEndData[0][0]));
	/*
	_FORMATIONM_STEP_END_DATA* p;
	p = & m_pFORMATION_STEP_END_DATA_STRUCTURE->stStepEndData[ucCh][ucStep];
	p->usLoopCountNow = 0;		//LJK 2024.11.01
	p->ucEndCondition = 0;
	p->uiTotalTestmsSecond32 = 0;
	p->uiCV_msSecond32 = 0;
	p->StepEndCondition.fChargeVoltage = 0;
	p->StepEndCondition.fDisChargeVoltage = 0;
	p->StepEndCondition.fChargeCurrent = 0;
	p->StepEndCondition.fDisChargeCurrent = 0;
	p->StepEndCondition.fChargeCapacity = 0;
	p->StepEndCondition.fDisChargeCapacity = 0;
	p->StepEndCondition.fChargeWattHour = 0;
	p->StepEndCondition.fDisChargeWattHour = 0;
	*/
}

void ClearStepEndDataAll( unsigned char ucCh )
{
	memset(&m_pFORMATION_STEP_END_DATA_STRUCTURE->stStepEndData[ucCh], 
		0, 
		sizeof(m_pFORMATION_STEP_END_DATA_STRUCTURE->stStepEndData[0]));	
	/*
	unsigned char i;
	for( i=0;i<_MAX_SEQUENCE_STEP; i++ )
		ClearStepEndData( ucCh, i );
	*/
}
	
void SystemInfoCleanAndFwVersion(void)
{
	memset( m_uiSystemInformation, 0x0 , sizeof( m_uiSystemInformation ) );
	m_uiSystemInformation[_FW_Version] = 0x20251217;
	/*
	Ver : 0x2023051130
		1: Internal ADC BoardAirTemperature 추가 (테스트 완료)
		   GetHeatSinkTemperaturError()	
		   GetAirTemperature()
		2: EEPROM 24LC512(64KByte) 테스트 완료
		3: EEPROM SDRAM 영역에서 SRAM영역으로
		   m_pEEPROM_DATA(1968 Byte) 구조체 SDRAM에서 SRAM으로 사용
		4: STAT_NONE 일때 CV Time 체크 제외
		   GetRingBufferVoltage  case _STAT_NONE		
	ver : 0x20230517
		1: BALANCE일때 충/방 모드에따라 EndConditions 마스크, NowModeStateSet() 추가
		   Discharge 일때 Charge EndConditions Mask
		   Charge 일때  Discharge EndConditions Mask		
	ver : 0x20230523
		1: BALANCE 일때 UCP, OCP, UWP, OWP, URP ORP 버그
		   AlarmParameterSet() _STATE_BALANCE 모드 추가
		   AlarmCheckProcess() _NOW_STATE_CHARGE, _NOW_STATE_DISCHARGE 조건
		2: Resume Pulse Mode 버그 수정, StepStart()
		   Resume 시작시 첫번째 레코드 제외
		   Pulse 모드일때 시간 초기화 없음
		3: GetRingBufferVoltage() 버그 수정
		   m_fVoltage1Sec 평균 오류  m_ui1SecRingCountVoltage[ch]+1 추가
		4: CV Time 버그 GetRingBufferVoltage()
		   AlarmParameterSet 확인 후 CV Time 증가
		   _STATE_PULSE && (_MODE_2POINT_PULSE_DISCHARGE || _MODE_10POINT_PULSE_DISCHARGE) 임시처리 원인 찾아야함
		5: SetPulseModeChange()  pull-up
		   CHARGE, pullup- HIGH
	ver : 0x20230530
		1: GetRingBufferVoltage() 버그 수정
		   m_fVoltage1Sec 평균 오류  m_ui1SecRingCountVoltage[ch]+1 삭제
		   _STATE_PULSE 모드 2POINT_PULSE, 10POINT_PULSE 충전/방전 조건  m_fVoltage1Sec[] 제외
		2: CPCR 모드 Tolerance 수정
		   _ALARM_RESISTER_TOLERANCE( MAX_SYSTEM_VOLTAGE / CHARGE_MINIMUM_CURREN * 0.02)	//	2%
		3: 0.04A 이하일때 MAX 전류값으로 CCCV, CCCV_DP 에적용
		   SetNextPulse() _PULSE_MODE_CCCV, _PULSE_MODE_CCCV_DP
	ver : 0x20230622
		1: Rest에서 전압 회복을 전압 이상으로 인식하는 에러	 (2.7V 종료 후 수 분 내에 3.4V 회복 시  ALARM_OVP로 정지)
		  Rest시 OVP, UVP 알람마스크 추가
		  _STEP_ALARM_MASK_REST_OVP, _STEP_ALARM_MASK_REST_UVP 추가(유부사장님)
		  GetAlarmMask(), AlarmCheckProcess() 참조
		2: 첫번째 레코드 m_uiStepTimeNow 이전 시간문제
		   StepStart() , 위치변경
	ver : 0x20231127, 유병길 부사장님 요청사항
		1: Alarm Mask ( MINUS_VOLTAGE, ISENSOR_VOLTAGE_MINUS ) StartAlarmCheck 함수에서 임시처리에서 정상처리
		   SequenceManager.c -> StartAlarmCheck 함수 참조		 
	ver : 0x20240215 유병길 부사장님 요청사항
		1: 처음시작시 간헐적 발생 _STEP_ALARM_VOLTAGE_BALANCE_ERROR, _STEP_ALARM_UCP, _STEP_ALARM_OCP Alarm 발생
			StartAlarmCheck(), _STEP_ALARM_VOLTAGE_BALANCE_ERROR, 15mV -> 30mV
			AlarmCheckProcess(), _STEP_ALARM_UCP, _STEP_ALARM_OCP,  1000 -> 5000	
		2: _PULSE_MODE_CCCV_DP 완료후 _MODE_2POINT_PULSE_DISCHARGE, _MODE_2POINT_PULSE_CHARGE 일때 이전 전류값 기록됨
			AdcReadCycleInterruptHandler(), 조건삭제
			if ( m_u16PulseTime1ms[0] - 1 == m_u16PulseTime1msNow[0] )//+4 && m_u16PulseTime1msNow>4 )
	ver : 0x20240409. ver : 0x20240215. 버전 2: 항목 원복 , 주석해제
		1: 전류가 0으로 나오는 현장
		   AdcReadCycleInterruptHandler()
		2: AlarmCheckProcess() -> _STEP_ALARM_MINUS_VOLTAGE 기존 -0.005에서 -0.3 으로 조정 (부사장님 요청사항)
	ver : 0x20240419
		1: REST CV_Time 2ms 증가 오류 수정
			GetRingBufferVoltage()   _STATE_REST 일때 CV Time 제외		
	ver : 0x20240704
		1: CELL_TEMP_ERROR Alarm 추가
			Pc_Comm_SetCellTemp 명령어 추가, m_shMaxCellTemp 초기화 (Default 70.0 도 * 10)
	ver : 0x20240726
		1: Heat Sink Error 채널별 적용(LM35DT)
			GetHeatSinkTemperaturError(), EtcAlarmSet(), InitializeVariable(), ReadyToStartForSafety()
	ver : 0x20240806
		1: CCCV_CP, CPCV_CP 기능 추가
			SetNextPulse()
	ver : 0x20240909
		1: Rest 상한, 하한 값입력 절대값, 상대값 적용
			AlarmParameterSet(), AlarmCheckProcess()			
	ver : 0x20241108
		1: DCIR 측정 최대 60초 기능 추가 ( _MODE_Sec_DCIR ), 하나기술 요청사항
			PushStepRecordData(), SaveStepEndData(), GetRingBufferVoltage(), 
			GetRingBufferCurrent(), StepStart(), SDRAM_AddressSet()
	ver : 0x20250106
		1: Step 종료시 전류 Ringing 현상 제거
			StepEndCheck()
	ver : 0x20250114
		1: HeatSink Alarm 온도 80 -> 100도 변경 (Req 유부사장님, 박희규 부장님)
			EtcAlarmSet()			
	ver : 0x20250117
		1: Rest 초기 0~1초 저장시 전압흔들림 안정화
			PushStepRecordData(), GetRingBufferVoltage()
		2: HeatSink Alarm 온도 100 -> 90도(최대값) LM35DT 저항분배 0.020mV 
			EtcAlarmSet()
		3: REST시 Physical(MC Off), Logical(MC On) 버그 수정
			McControl()		
	ver : 0x20250204
		1: 1초레코딩 V/I 평균값 낮음, 1초일때만 현재 V/I 적용 1초이후부터 평균값 적용(하나기술 FAT중 발견)
			PushStepRecordData();				
	ver : 0x20250205
		1: 2초레코딩 V/I 평균값 낮음, 2초까지 현재 V/I 적용 2초이후부터 평균값 적용(하나기술 FAT중 발견)
			PushStepRecordData();
	ver : 0x20250317
		1: 200A 시정수 변경 후 TrTf Overshoot, Undershoot 보정(200A 전용)
			NewStepSetting(), CaptainTimerCounterInterrupHandler(), AdcReadCycleInterruptHandler()
	ver : 0x20250318
		1: 사용자 정지후 다음레시피에 StepEnd, Record데이터 남는버그 수정
			m_bStepRunning[ch] = false,	CaptainTimerCounterInterrupHandler()
	ver : 0x20250328
		1: REST Ah, Wh 0 추가
			PushStepRecordData()
	ver : 0x20250404
		1: _STEP_ALARM_VSENSOR_MINUS, _STEP_ALARM_ISENSOR_VOLTAGE_MINUS, -0.005 -> -0.1V 변경 (LGE JF2 부사장님 요청사항)
			StartAlarmCheck()
	ver : 0x20250414
		1: CPCV Watt Max Limit 버그 수정 (하나기술 FAT)
			CpCrMode()
	ver : 0x20250428
		1: Alarm Control (알람 설정, 알람처리)
			AlarmParameterSet()
				_MODE_CRCV 입력저항의 ±2%
				_MODE_CPCV 입력와트의 ±2%
			AlarmCheckProcess()
				_MODE_CRCV -> URP, ORP
		2: Alarm Control (프로토콜 및 초기화)
			InterpretPcCommunication() : PC_Comm_SetVoltage, PC_Comm_SetCurrent, PC_Comm_SetResistance, PC_Comm_SetWatt, PC_Comm_SeHeatSink, PC_Comm_SetAirTemp
			HeatSink, AirTemp : InitializeVariable() 초기화, FormationEnd() 복귀 
	ver : 0x20250429
		1: 채널별 Boost, Soft 자동 기능추가
			NowModeStateSet(), #define _SOFT_AUTO_
	ver : 0x20250507
		1: Alarm Control (알람 설정 수정)
		   UCP, OCP 전류 컨트롤 방식으로 변경 CV_Time 100ms
		   InterpretPcCommunication(), PC_Comm_SetCurrent
	ver : 0x20250512
		1: Alarm Control (알람 설정 수정)
		   UWP 오류 카운터 증가 버그 수정, AlarmCheckProcess()
	ver : 0x20250521
		1: CPCV, CRCV Shoot 계속증가하는 버그 수정(bgyu)
		  CpCrMode()	
		2: 최소제어전류는 0.001A or -0.001A
		  KnC_Define.h : CHARGE_MINIMUM_CURREN, DISCHARGE_MINIMUM_CURREN
		3: 최소전류시 강제 셋 기능 삭제
		  SetNextPulse() : //LJK 2023.05.30 입력값이 0.04A 이하일때 MAX 전류값으로 기능 삭제(_PULSE_RECTANGLE, _PULSE_TRIANGLE)
		4: HeatSink 상시에서 Alarm Mask 영역으로 변경(bgyu 요청사항)
		  EtcAlarmSet(), AlarmCheckProcess()
	ver : 0x20250522
		1: 최소제어전류는 0.001A or -0.001A 버그 수정(CCCV_DP, CCCVCP 최대전류 버그)
		  SetNextPulse()
	ver : 0x20250528		  	  
		1: Tr/Tf Overshoot , Undershoot 수정 (bgyu)
		2: 충방전환시 shoot 수정 (bgyu)
	ver : 0x20250529
		1: Tr/Tf Overshoot , Undershoot 2차 수정 (bgyu)
		2: 충방전환시 shoot 2차 수정 (bgyu)
		  충/방 전시 적분기 처박기 시간지연, CaptainTimerCounterInterrupHandler(), m_ucNextCurrentDelay[]
	ver : 0x20250604
		1: Tr/Tf Overshoot , Undershoot 3차 수정 (bgyu)
		  NewStepSetting() 2~3채널 Delay 
	ver : 0x20250611
		1: CCCV_DP, CCCV_CP 증가시는 Max Limit, 감소시는 Min Limit (bgyu)
		  SetNextPulse()
		2: Voltage balance Error 수정보완 (bgyu)
		  FormationEndCheck()
	ver : 0x20250617
		1: PCCV_DP2, PPCV_DP2, PRCV_CP2 이전 Pulse Step의 마지막 전류값을 Pulse 시작 전류값으로 사용 (bgyu)
	ver : 0x20250722
		1: Pulse CPCV, DP 버그 수정 (bgyu)
	ver : 0x20250723
		1: _5V_350A_ define 추가
	ver : 0x20250826
		1: CpCrMode() 버그 수정
	ver : 0x20250827
		1: SetNextPulse() 버그 수정
	ver : 0x20250828
		1: MakeStepDataOne_Channel() 수정
		   IDLE 상태에서 전류는 0으로 표시.
	ver : 0x20250905
		1: Pulse Overshoot 수정
	ver : 0x20250908
		1: Protection Condition 설정 기능 추가
	ver : 0x20250909
		1: Reverse Cell Alarm 수정
		2: Alarm 동작 수정
	ver : 0x20250911
		1: Alarm 동작 수정
	ver : 0x20250919
		1: Pulse CP/CR 계산 복원시킴
	ver : 0x20251217
		1: DCIR 버그 수정
	*/
}

unsigned short FanAlarmAirTempCheck( void )
{
	unsigned short usRet = 0;
	
	return usRet;
}

void GetRingBufferVoltage( unsigned char ucCh )
{
	if ( m_ucNowState[ucCh] == _STATE_DCIR  && m_ucPulseIndex[ucCh] == 1 && m_u16PulseTime1msNow[ucCh] > 0 )	// DCIR Pulse Index 0
	{
		if( m_u16PulseTime1msNow[ucCh] > 2 ) m_fDisChargeVoltage[ucCh] = m_fVoltage1Sec[ucCh];
		if(m_ucNowMode[ucCh] == _MODE_10ms_DCIR && m_u16PulseTime1msNow[ucCh] < 16 )
		{
			m_fDisChargeVoltage[ucCh] = m_fDCIR_Voltage2[ucCh][m_u16PulseTime1msNow[ucCh]-1] = GetHexToVoltageCurrent( VOLTAGE_RANGE_INDEX+ucCh, m_i1msVoltage[ucCh]*1000 );
			if ( m_u16PulseTime1msNow[ucCh] == 14)
				m_bDCIR_StepStop[ucCh] = TRUE;			
		}
		else if( m_ucNowMode[ucCh] == _MODE_Sec_DCIR && m_pSeqNow[ucCh]->u16PulseTime1ms[1] / 1000 <= m_pSecDcirBuffer->usDCIR_Count[ucCh] )
		{			
			m_fDCIR_Voltage2[ucCh][11] = GetHexToVoltageCurrent( VOLTAGE_RANGE_INDEX+ucCh, m_i1msVoltage[ucCh]*1000 );
			m_bDCIR_StepStop[ucCh] = TRUE;
			m_fDisChargeVoltage[ucCh] = m_fDCIR_Voltage2[ucCh][11];
		}
		
	}
	if ( m_ui1SecRingCountVoltage[ucCh]<MAX_1Sec_BUFFER )
	{
		if( m_ui1SecRingCountVoltage[ucCh] == 0)
		{
			m_fRecordVoltage[ucCh] = m_fNowVoltage[ucCh];
			m_iVoltage1SecTotal[ucCh] = m_i1msVoltage[ucCh];
		}
		else
		{
			m_iVoltage1SecTotal[ucCh] += m_i1msVoltage[ucCh];
			m_fRecordVoltage[ucCh] = m_fNowVoltage[ucCh];
		}
		m_p1SecBuffer->iVoltage[ucCh][m_ui1SecRingCountVoltage[ucCh]] = m_i1msVoltage[ucCh];
	}
	else
	{
		U16 u16Index;
		u16Index = m_ui1SecRingCountVoltage[ucCh] % MAX_1Sec_BUFFER;
		m_iVoltage1SecTotal[ucCh] -= m_p1SecBuffer->iVoltage[ucCh][u16Index];
		m_iVoltage1SecTotal[ucCh] += m_i1msVoltage[ucCh];
		m_p1SecBuffer->iVoltage[ucCh][u16Index] = m_i1msVoltage[ucCh];
	}
	//LJK 2023.05.23 m_ui1SecRingCountVoltage[ucCh]+1, CV Time 증가원인
	if ( m_ui1SecRingCountVoltage[ucCh]<MAX_1Sec_BUFFER )
		m_fVoltage1Sec[ucCh] = GetHexToVoltageCurrent( VOLTAGE_RANGE_INDEX + ucCh, (int)((double)m_iVoltage1SecTotal[ucCh]/((double)m_ui1SecRingCountVoltage[ucCh]/(double)MAX_1Sec_BUFFER)) );
	else
		m_fVoltage1Sec[ucCh] = GetHexToVoltageCurrent( VOLTAGE_RANGE_INDEX + ucCh, m_iVoltage1SecTotal[ucCh] );
		
	if ( m_ucNowState[ucCh] == _STATE_DCIR && m_ucPulseIndex[ucCh] == 0 )	// DCIR Pulse Index 0
	{
		m_fDCIR_Voltage1[ucCh] = m_fChargeVoltage[ucCh]= m_fVoltage1Sec[ucCh];
	}

	
	switch ( m_ucNowState[ucCh] )
	{
		case _STATE_CHARGE:
			m_fChargeVoltage[ucCh] = m_fVoltage1Sec[ucCh];
			if ( (m_fChargeVoltage[ucCh]>=m_fCvVoltageLow[ucCh] && m_fChargeVoltage[ucCh]<=m_fCvVoltageHigh[ucCh]) || m_fVoltage1Sec[ucCh] > m_fCvVoltageHigh[ucCh] )
				if( m_bAlarmParameterSet[ucCh] ) m_uiStepCV_TimeNow[ucCh]++;
			break;
			
		case _STATE_DISCHARGE:
			m_fDisChargeVoltage[ucCh] = m_fVoltage1Sec[ucCh];
			if ( (m_fDisChargeVoltage[ucCh]>=m_fCvVoltageLow[ucCh] && m_fDisChargeVoltage[ucCh]<=m_fCvVoltageHigh[ucCh]) || m_fVoltage1Sec[ucCh] < m_fCvVoltageLow[ucCh])
				if( m_bAlarmParameterSet[ucCh] ) m_uiStepCV_TimeNow[ucCh]++;
			break;
		
		case _STATE_PULSE:
			if ( m_ucNowMode[ucCh] == _MODE_2POINT_PULSE_CHARGE ||  m_ucNowMode[ucCh] == _MODE_10POINT_PULSE_CHARGE )
			{
				if ( (m_fChargeVoltage[ucCh]>=m_fCvVoltageLow[ucCh] && m_fChargeVoltage[ucCh]<=m_fCvVoltageHigh[ucCh] ))// || m_fVoltage1Sec[ucCh] > m_fCvVoltageHigh[ucCh] )
					if( m_bAlarmParameterSet[ucCh] ) m_uiStepCV_TimeNow[ucCh]++;
			}
			if ( m_ucNowMode[ucCh] == _MODE_2POINT_PULSE_DISCHARGE ||  m_ucNowMode[ucCh] == _MODE_10POINT_PULSE_DISCHARGE )
			{
				//if ( m_uiStepTimeNow[ucCh] - m_uiPauseTimeNow[ucCh] <= 1001 ) 
				//	m_fVoltage1Sec[ucCh] = m_fNowVoltage[ucCh];//LJK 2023.05.24 원인 찾아야함 일단꼼수
				if ( (m_fDisChargeVoltage[ucCh]>=m_fCvVoltageLow[ucCh] && m_fDisChargeVoltage[ucCh]<=m_fCvVoltageHigh[ucCh]))// || m_fVoltage1Sec[ucCh] < m_fCvVoltageLow[ucCh] )
					if( m_bAlarmParameterSet[ucCh] ) m_uiStepCV_TimeNow[ucCh]++;
			}
			break;
			
		case _STATE_NONE:	//LJK 2023.05.11  _STATE_NONE 일때 CV Time 제외
		case _STATE_REST:	//LJK 2024.04.19  _STATE_REST 일때 CV Time 제외
			m_fChargeVoltage[ucCh] = m_fDisChargeVoltage[ucCh] = m_fVoltage1Sec[ucCh];
			//LJK 2024.01.17
			if( m_ucNowState[ucCh] == _STATE_REST && m_uiStepTimeNow[ucCh] <= 1004)
				m_fRecordVoltage[ucCh] = m_fNowVoltage[ucCh];			
			break;
		case _STATE_DCIR:	//LJK 2024.10.17 
			if( m_ucNowMode[ucCh] == _MODE_Sec_DCIR )
			{
				if( m_ucPulseIndex[ucCh] == 0 ) 
				{
					m_fChargeVoltage[ucCh] = m_fVoltage1Sec[ucCh];
					//m_fDisChargeVoltage[ucCh] = m_fVoltage1Sec[ucCh];
				}	
			}
			break;	
		default:
			m_fChargeVoltage[ucCh] = m_fDisChargeVoltage[ucCh] = m_fVoltage1Sec[ucCh];
			if ( m_fDisChargeVoltage[ucCh]>=m_fCvVoltageLow[ucCh] && m_fDisChargeVoltage[ucCh]<=m_fCvVoltageHigh[ucCh] )
				if( m_bAlarmParameterSet[ucCh] ) m_uiStepCV_TimeNow[ucCh]++;
			break;
	}
	
	if ( (m_ui1SecRingCountVoltage[ucCh]++%MAX_1Sec_BUFFER)!=(MAX_1Sec_BUFFER-1) )
		return;
		
	if ( m_uiRecordRingCountVoltage[ucCh] == 0 )
	{
		m_f3SecVoltage[ucCh][0] = m_f3SecVoltage[ucCh][1] = m_f3SecVoltage[ucCh][2] = m_fVoltage1Sec[ucCh];
	}
	
	switch( m_uiRecordSecond[ucCh] )
	{
		case 1:
			m_fRecordVoltage[ucCh] = m_f3SecVoltage[ucCh][0] = m_fVoltage1Sec[ucCh];
			break;
			
		case 2:
			if ( m_uiRecordRingCountVoltage[ucCh] < 2 )
				m_f3SecVoltage[ucCh][0] = m_f3SecVoltage[ucCh][1] = m_fVoltage1Sec[ucCh];
			else
				m_f3SecVoltage[ucCh][m_uiRecordRingCountVoltage[ucCh]%2] =  m_fVoltage1Sec[ucCh];
			
			m_fRecordVoltage[ucCh] = (m_f3SecVoltage[ucCh][0]+m_f3SecVoltage[ucCh][1])/2.0f;
			break;
			
		default:
			if ( m_uiRecordRingCountVoltage[ucCh]<3 )
				m_f3SecVoltage[ucCh][0] = m_f3SecVoltage[ucCh][1] = m_f3SecVoltage[ucCh][2] = m_fVoltage1Sec[ucCh];
			else
				m_f3SecVoltage[ucCh][m_uiRecordRingCountVoltage[ucCh]%3] = m_fVoltage1Sec[ucCh];
			m_fRecordVoltage[ucCh] = (m_f3SecVoltage[ucCh][0]+m_f3SecVoltage[ucCh][1]+m_f3SecVoltage[ucCh][2])/3.0f;
			break;
	}
	//LJK 2024.01.17
	if( m_ucNowState[ucCh] == _STATE_REST && m_uiStepTimeNow[ucCh] <= 1004)
		m_fRecordVoltage[ucCh] = m_fNowVoltage[ucCh];
	
	m_uiRecordRingCountVoltage[ucCh]++;

	if ( m_fVoltage1SecPre[ucCh] > m_fVoltage1Sec[ucCh] + 0.010f )
	{
		if ( ++m_u16SafeErrorNumbers[ucCh][RTY_VOLTAGE_FAST_DOWN]>=10 )			// 10000ms At 10mV*10
				m_bVoltageDeltaError[0][ucCh] = TRUE;
	}
	else
		m_u16SafeErrorNumbers[ucCh][RTY_VOLTAGE_FAST_DOWN] = 0;

	if ( m_fVoltage1SecPre[ucCh]+0.010f<m_fVoltage1Sec[ucCh] )
	{
		if ( ++m_u16SafeErrorNumbers[ucCh][RTY_VOLTAGE_FAST_UP]>=10 )			// 10000ms At 10mV*10
			m_bVoltageDeltaError[1][ucCh] = TRUE;
	}
	else
		m_u16SafeErrorNumbers[ucCh][RTY_VOLTAGE_FAST_UP] = 0;

	m_fVoltage1SecPre[ucCh] = m_fVoltage1Sec[ucCh];
}

void GetRingBufferCurrent( unsigned char ucCh )
{
	if ( m_ucNowState[ucCh] == _STATE_DCIR && m_ucPulseIndex[ucCh] == 1 && m_u16PulseTime1msNow[ucCh] > 0 )	// DCIR Pulse Index 0
	{
		if( m_u16PulseTime1msNow[ucCh] > 2 ) m_fChargeCurrent[ucCh] = 0;
		if( m_ucNowMode[ucCh] == _MODE_10ms_DCIR  && m_u16PulseTime1msNow[ucCh] < 15 )
		{
			m_fDisChargeCurrent[ucCh] = m_fDCIR_Current[ucCh][m_u16PulseTime1msNow[ucCh] - 1] = GetHexToVoltageCurrent( CURRENT_RANGE_INDEX+ucCh*2+0, m_i1msCurrent[ucCh]*1000 );
		}
		else if( m_ucNowMode[ucCh] == _MODE_Sec_DCIR && m_pSeqNow[ucCh]->u16PulseTime1ms[0] <= m_uiStepTimeNow[ucCh])		
		{
			if( m_u16PulseTime1msNow[ucCh] > 4 && m_uiStepTimeNow[ucCh] % 1000 == 0)//m_u16PulseTime1msNow[ucCh]-1 % 1000 == 0 )
			{
				m_pSecDcirBuffer->fDCIR_Current[ucCh][m_pSecDcirBuffer->usDCIR_Count[ucCh]] = m_fDisChargeCurrent[ucCh] = m_fCurrent1Sec[ucCh];
				//m_pSecDcirBuffer->fDCIR_Current[ucCh][m_pSecDcirBuffer->usDCIR_Count[ucCh]] = GetHexToVoltageCurrent( CURRENT_RANGE_INDEX+ucCh*2+0, m_i1msCurrent[ucCh]*1000 );
				m_pSecDcirBuffer->usDCIR_Count[ucCh]++;
			}

		}
	}
	if ( m_ui1SecRingCountCurrent[ucCh] < MAX_1Sec_BUFFER )
	{
		if( m_ui1SecRingCountCurrent[ucCh] == 0)
		{
			m_fRecordCurrent[ucCh] = m_fNowCurrent[ucCh];
			m_iCurrent1SecTotal[ucCh] = m_i1msCurrent[ucCh];
		}
		else
		{
			m_iCurrent1SecTotal[ucCh] += m_i1msCurrent[ucCh];
			m_fRecordCurrent[ucCh] = m_fNowCurrent[ucCh];
		}

		m_p1SecBuffer->iCurrent[ucCh][m_ui1SecRingCountCurrent[ucCh]] = m_i1msCurrent[ucCh];
	}
	else
	{
		U16 u16Index;

		u16Index = m_ui1SecRingCountCurrent[ucCh] % MAX_1Sec_BUFFER;
		m_iCurrent1SecTotal[ucCh] -= m_p1SecBuffer->iCurrent[ucCh][u16Index];
		m_iCurrent1SecTotal[ucCh] += m_i1msCurrent[ucCh];
		m_p1SecBuffer->iCurrent[ucCh][u16Index] = m_i1msCurrent[ucCh];
	}
	
	if( m_iCurrent1SecTotal[ucCh] & 0x80000000)
	{
		if ( m_ui1SecRingCountCurrent[ucCh] < MAX_1Sec_BUFFER )
			m_fCurrent1Sec[ucCh] = GetHexToVoltageCurrent( CURRENT_RANGE_INDEX+ucCh*2+0, (int)((double)m_iCurrent1SecTotal[ucCh]/((double)m_ui1SecRingCountCurrent[ucCh]/(double)MAX_1Sec_BUFFER) ) );
		else
			m_fCurrent1Sec[ucCh] = GetHexToVoltageCurrent( CURRENT_RANGE_INDEX+ucCh*2+0, m_iCurrent1SecTotal[ucCh] );
	}
	else
	{
		if ( m_ui1SecRingCountCurrent[ucCh] < MAX_1Sec_BUFFER )
			m_fCurrent1Sec[ucCh] = GetHexToVoltageCurrent( CURRENT_RANGE_INDEX+ucCh*2+1, (int)((double)m_iCurrent1SecTotal[ucCh]/((double)m_ui1SecRingCountCurrent[ucCh]/(double)MAX_1Sec_BUFFER) ) );
		else
			m_fCurrent1Sec[ucCh] = GetHexToVoltageCurrent( CURRENT_RANGE_INDEX+ucCh*2+1, m_iCurrent1SecTotal[ucCh] );
	}

	switch ( m_ucNowState[ucCh] )
	{
		case _STATE_CHARGE:
			m_fChargeCurrent[ucCh] = m_fCurrent1Sec[ucCh];
			break;
		
		case _STATE_DISCHARGE:
			m_fDisChargeCurrent[ucCh] = m_fCurrent1Sec[ucCh];
			break;
		
		case _STATE_PULSE:	
			break;
		case _STATE_DCIR:	//LJK 2024.10.17 
			if( m_ucPulseIndex[ucCh] == 0 ) 
			{
				m_fChargeCurrent[ucCh] = m_fCurrent1Sec[ucCh];
				//m_fDisChargeCurrent[ucCh] = 0;
			}
			break;			
		default:
			m_fChargeCurrent[ucCh] = m_fDisChargeCurrent[ucCh] = m_fCurrent1Sec[ucCh];
			break;
	}
	
	if ( (m_ui1SecRingCountCurrent[ucCh]++ % MAX_1Sec_BUFFER) != (MAX_1Sec_BUFFER - 1) )
		return;

	if ( m_ucNowState[ucCh] == _STATE_CONTACT && m_ui1SecRingCountCurrent[ucCh] == MAX_1Sec_BUFFER && m_bStepStarted[ucCh] )
	{
		gpio_set_pin_high(m_uiVoltage2ModeAddress[ucCh]);
		m_bVoltage2Mode[ucCh] = TRUE;
	}
		
	if ( m_uiRecordRingCountCurrent[ucCh] == 0 )
	{
		m_f3SecCurrent[ucCh][0] = m_f3SecCurrent[ucCh][1] = m_f3SecCurrent[ucCh][2] = m_fCurrent1Sec[ucCh];
	}

	switch ( m_uiRecordSecond[ucCh] )
	{
		case 1:
			m_fRecordCurrent[ucCh] = m_f3SecCurrent[ucCh][0] = m_fCurrent1Sec[ucCh];
			break;
				
		case 2:
			m_f3SecCurrent[ucCh][m_uiRecordRingCountCurrent[ucCh]%2] =  m_fCurrent1Sec[ucCh];
			m_fRecordCurrent[ucCh] = (m_f3SecCurrent[ucCh][0]+m_f3SecCurrent[ucCh][1])/2.0f;
			break;
			
		default:
			m_f3SecCurrent[ucCh][m_uiRecordRingCountCurrent[ucCh]%3] = m_fCurrent1Sec[ucCh];
			m_fRecordCurrent[ucCh] = (m_f3SecCurrent[ucCh][0]+m_f3SecCurrent[ucCh][1]+m_f3SecCurrent[ucCh][2])/3.0f;
			break;	
	}
	
	m_uiRecordRingCountCurrent[ucCh]++;
}

void GetRingBufferVoltage2( unsigned char ucCh )
{
	if ( m_ui1SecRingCountVoltage2[ucCh] < MAX_1Sec_BUFFER )
	{
		if( m_ui1SecRingCountVoltage2[ucCh] == 0)
		{
			m_fRecordVoltage2[ucCh] = m_fNowVoltage2[ucCh];
			m_iVoltage2_1SecTotal[ucCh] = m_i1msVoltage2[ucCh];
		}
		else
		{
			m_iVoltage2_1SecTotal[ucCh] += m_i1msVoltage2[ucCh];
			m_fRecordVoltage2[ucCh] = m_fNowVoltage2[ucCh];
		}

		m_p1SecBuffer->iVoltage2[ucCh][m_ui1SecRingCountVoltage2[ucCh]] = m_i1msVoltage2[ucCh];
	}
	else
	{
		U16 u16Index;
		u16Index = m_ui1SecRingCountVoltage2[ucCh] % MAX_1Sec_BUFFER;
		m_iVoltage2_1SecTotal[ucCh] -= m_p1SecBuffer->iVoltage2[ucCh][u16Index];
		m_iVoltage2_1SecTotal[ucCh] += m_i1msVoltage2[ucCh];
		m_p1SecBuffer->iVoltage2[ucCh][u16Index] = m_i1msVoltage2[ucCh];
	}

	if ( m_ui1SecRingCountVoltage2[ucCh] < MAX_1Sec_BUFFER )
		m_fVoltage2_1Sec[ucCh] = GetHexToVoltageCurrent( VOLTAGE2_RANGE_INDEX+ucCh, (int)((double)m_iVoltage2_1SecTotal[ucCh]/((double)m_ui1SecRingCountVoltage2[ucCh]/(double)MAX_1Sec_BUFFER)) );
	else
		m_fVoltage2_1Sec[ucCh] = GetHexToVoltageCurrent( VOLTAGE2_RANGE_INDEX+ucCh, m_iVoltage2_1SecTotal[ucCh] );
	
	if ( (m_ui1SecRingCountVoltage2[ucCh]++ % MAX_1Sec_BUFFER) != (MAX_1Sec_BUFFER-1) )
		return;

	if ( m_uiRecordRingCountVoltage2[ucCh] == 0 )
	{
		m_f3SecVoltage2[ucCh][0] = m_f3SecVoltage2[ucCh][1] = m_f3SecVoltage2[ucCh][2] = m_fVoltage2_1Sec[ucCh];
	}
	
	switch( m_uiRecordSecond[ucCh] )
	{
		case 1:
			m_fRecordVoltage2[ucCh] = m_f3SecVoltage2[ucCh][0] = m_fVoltage2_1Sec[ucCh];
			break;
			
		case 2:
			if ( m_uiRecordRingCountVoltage2[ucCh] < 2 )
				m_f3SecVoltage2[ucCh][0] = m_f3SecVoltage2[ucCh][1] = m_fVoltage2_1Sec[ucCh];
			else
				m_f3SecVoltage2[ucCh][m_uiRecordRingCountVoltage2[ucCh]%2] =  m_fVoltage2_1Sec[ucCh];
			
			m_fRecordVoltage2[ucCh] = (m_f3SecVoltage2[ucCh][0]+m_f3SecVoltage2[ucCh][1])/2.0f;
			break;
			
		default:
			if ( m_uiRecordRingCountVoltage2[ucCh] < 3 )
				m_f3SecVoltage2[ucCh][0] = m_f3SecVoltage2[ucCh][1] = m_f3SecVoltage2[ucCh][2] = m_fVoltage2_1Sec[ucCh];
			else
				m_f3SecVoltage2[ucCh][m_uiRecordRingCountVoltage2[ucCh]%3] = m_fVoltage2_1Sec[ucCh];
			
			m_fRecordVoltage2[ucCh] = (m_f3SecVoltage2[ucCh][0]+m_f3SecVoltage2[ucCh][1]+m_f3SecVoltage2[ucCh][2])/3.0f;
			break;
	}

	m_uiRecordRingCountVoltage2[ucCh]++;
}


bool ReadAdcSpi8One( int* pVoltage, int* pCurrent )
{	
	unsigned char  i;
	short j;
	unsigned short cReadBuf[16];
	short shData[8];
	
	//PDB_LOW = _PORT_B_ADC_DOUTB;

	PDA_LOW = _PORT_A_ADC_CONVST;
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	ADC_SPI->mr |= AVR32_SPI_MR_PCS_MASK;												// ADC_SYNC LOW
	if (ADC_SPI->mr & AVR32_SPI_MR_PCSDEC_MASK) 
		ADC_SPI->mr &= ~AVR32_SPI_MR_PCS_MASK |	(0 << AVR32_SPI_MR_PCS_OFFSET);
	else 
		ADC_SPI->mr &= ~(1 << (AVR32_SPI_MR_PCS_OFFSET + 0));
	PDA_HIGH = _PORT_A_ADC_CONVST;

	for( i=0; i<16; i++ )
	{
		ADC_SPI->tdr = CONFIG_SPI_MASTER_DUMMY << AVR32_SPI_TDR_TD_OFFSET;					// spi_write_single(spi,CONFIG_SPI_MASTER_DUMMY);
		for( j=0; j<DAC_SPI_WAIT_LOOPN; j++ )
			if ( spi_is_rx_ready(ADC_SPI) )
				break;
		cReadBuf[i] = ADC_SPI->rdr >> AVR32_SPI_RDR_RD_OFFSET;								// spi_read_single(spi,&val);
	}
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//DEBUG_TP2_L;
	
	ADC_SPI->mr |= AVR32_SPI_MR_PCS_MASK;												// ADC_SYNC HIGH
	ADC_SPI->cr = AVR32_SPI_CR_LASTXFER_MASK;
	for( i=0; i<8; i++ )
		shData[i] = (cReadBuf[i*2]<<8)|cReadBuf[i*2+1];

	pCurrent[0] = shData[0];
	pCurrent[1] = shData[2];
	pCurrent[2] = shData[4];
	pCurrent[3] = shData[6];

	pVoltage[0] = shData[1];
	pVoltage[1] = shData[3];
	pVoltage[2] = shData[5];
	pVoltage[3] = shData[7];
	//DEBUG_TP2_L;
	return true;
}

bool ReadAdcData( char* cReadData )
{
		/*
		//unsigned char ucReadData[3];								// Read
		//spi_select_device(DAC_SPI,&SPI_DEVICE_EXAMPLE);
		//spi_read_packet( DAC_SPI, ucReadData, 3 );
		//spi_deselect_device(DAC_SPI,&SPI_DEVICE_EXAMPLE);
	*/
	unsigned char  i;
	short j;
	unsigned short cReadBuf[16];
	short shData[8];
	
	
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	ADC_SPI->mr |= AVR32_SPI_MR_PCS_MASK;												// ADC_SYNC LOW
	if (ADC_SPI->mr & AVR32_SPI_MR_PCSDEC_MASK) 
		ADC_SPI->mr &= ~AVR32_SPI_MR_PCS_MASK |	(0 << AVR32_SPI_MR_PCS_OFFSET);
	else 
		ADC_SPI->mr &= ~(1 << (AVR32_SPI_MR_PCS_OFFSET + 0));
/*
	for( i=0; i<8; i++ )
	{
		ADC_SPI->tdr = CONFIG_SPI_MASTER_DUMMY << AVR32_SPI_TDR_TD_OFFSET;					// spi_write_single(spi,CONFIG_SPI_MASTER_DUMMY);
		for( j=0; j<DAC_SPI_WAIT_LOOPN; j++ )
			if ( spi_is_rx_ready(ADC_SPI) )
				break;
		cReadData[i] = ADC_SPI->rdr >> AVR32_SPI_RDR_RD_OFFSET;								// spi_read_single(spi,&val);
	}
*/
	for( i=0; i<16; i++ )
	{
		ADC_SPI->tdr = CONFIG_SPI_MASTER_DUMMY << AVR32_SPI_TDR_TD_OFFSET;					// spi_write_single(spi,CONFIG_SPI_MASTER_DUMMY);
		for( j=0; j<DAC_SPI_WAIT_LOOPN; j++ )
			if ( spi_is_rx_ready(ADC_SPI) )
				break;
		cReadBuf[i] = ADC_SPI->rdr >> AVR32_SPI_RDR_RD_OFFSET;								// spi_read_single(spi,&val);
	}
	for( i=0; i<8; i++ )
		shData[i] = (cReadBuf[i*2]<<8)|cReadBuf[i*2+1];
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	ADC_SPI->mr |= AVR32_SPI_MR_PCS_MASK;												// ADC_SYNC HIGH
	ADC_SPI->cr = AVR32_SPI_CR_LASTXFER_MASK;
	return true;
}


float GetHexToVoltageCurrent( unsigned char iRange, int iAdc )
{
	U8 iCenter;
	U8 iLeft;
	U8 iRight;
	U8 iMax;
	U8 iError;
	float fAdcPercent;
	float fReferance;
	U8 iDelta;

	iMax = m_pEEPROM_DATA.ucMaxStep[iRange]-1;
	if ( iMax>10 )
		iMax = 10;
	iError = 6;
	iLeft = 0;
	iRight = iMax;

	if ( m_pEEPROM_DATA.iAdc[iRange][0]>m_pEEPROM_DATA.iAdc[iRange][iMax] )
	{
		if ( m_pEEPROM_DATA.iAdc[iRange][0]<=iAdc )
		{
			iCenter = 0;
			iDelta = 1;
		}
		else
		if ( m_pEEPROM_DATA.iAdc[iRange][iMax]>=iAdc )
		{
			iCenter = iMax-1;
			iDelta = iMax;
		}
		else
		{
			for(;iError;)
			{
				iError--;
				iCenter = (iLeft+iRight)/2;
				if ( iCenter==0 || iCenter>=iMax-1 )
					break;
				if ( m_pEEPROM_DATA.iAdc[iRange][iCenter]>=iAdc && m_pEEPROM_DATA.iAdc[iRange][iCenter+1]<=iAdc )
					break;
				if ( iAdc > m_pEEPROM_DATA.iAdc[iRange][iCenter] )
					iRight = iCenter;
				else
					iLeft = iCenter;
			}
			iDelta = iCenter+1;
		}
		fAdcPercent =	((float)( m_pEEPROM_DATA.iAdc[iRange][iCenter]-iAdc ))
			/( m_pEEPROM_DATA.iAdc[iRange][iCenter]-m_pEEPROM_DATA.iAdc[iRange][iDelta] );
		fReferance = m_pEEPROM_DATA.fReferance[iRange][iCenter] -
			(m_pEEPROM_DATA.fReferance[iRange][iCenter] - m_pEEPROM_DATA.fReferance[iRange][iDelta]) * fAdcPercent ;
	}
	else
	{
		if ( m_pEEPROM_DATA.iAdc[iRange][0]>=iAdc )
		{
			iCenter = 0;
			iDelta = 1;
		}
		else
		if ( m_pEEPROM_DATA.iAdc[iRange][iMax]<=iAdc)
		{
			iCenter = iMax-1;
			iDelta = iMax;
		}
		else
		{
			for(;iError;)
			{
				iError--;
				iCenter = (iLeft+iRight)/2;
				if ( iCenter==0 || iCenter>=iMax-1 )
					break;
				if ( m_pEEPROM_DATA.iAdc[iRange][iCenter]<=iAdc && m_pEEPROM_DATA.iAdc[iRange][iCenter+1]>=iAdc )
					break;
				if ( iAdc < m_pEEPROM_DATA.iAdc[iRange][iCenter] )
					iRight = iCenter;
				else
					iLeft = iCenter;
			}
			iDelta = iCenter+1;
		}
		fAdcPercent =	((float)( iAdc - m_pEEPROM_DATA.iAdc[iRange][iCenter])) /
			( m_pEEPROM_DATA.iAdc[iRange][iDelta]-m_pEEPROM_DATA.iAdc[iRange][iCenter] );
		
		fReferance = m_pEEPROM_DATA.fReferance[iRange][iCenter] +
			(m_pEEPROM_DATA.fReferance[iRange][iDelta] - m_pEEPROM_DATA.fReferance[iRange][iCenter]) * fAdcPercent ;
	}
	if ( iError==0 )
		return 0.0f;
	return fReferance;
}

short GetVoltageCurrentToDacHex( unsigned char iRange, float fData )
{
	U8 iCenter;
	U8 iLeft;
	U8 iRight;
	U8 iMax;
	U8 iError;
	float fDacPercent;
	float fDacSet;
	U8 iDelta;
	short s16Dac;


	iMax = m_pEEPROM_DATA.ucMaxStep[iRange]-1;
	if ( iMax>10 )
		iMax = 10;
	iError = 6;
	iLeft = 0;
	iRight = iMax;

	if ( m_pEEPROM_DATA.fReferance[iRange][0]>m_pEEPROM_DATA.fReferance[iRange][iMax] )
	{
		if ( m_pEEPROM_DATA.fReferance[iRange][0]<=fData )
		{
			iCenter = 0;
			iDelta = 1;
		}
		else
		if ( m_pEEPROM_DATA.fReferance[iRange][iMax]>=fData )
		{
			iCenter = iMax-1;
			iDelta = iMax;
		}
		else
		{
			for(;iError;)
			{
				iError--;
				iCenter = (iLeft+iRight)/2;
				if ( iCenter==0 || iCenter>=iMax-1 )
					break;
				if ( m_pEEPROM_DATA.fReferance[iRange][iCenter]>=fData && m_pEEPROM_DATA.fReferance[iRange][iCenter+1]<=fData )
					break;
				if ( fData > m_pEEPROM_DATA.fReferance[iRange][iCenter] )
					iRight = iCenter;
				else
					iLeft = iCenter;
			}
			iDelta = iCenter+1;
		}
		fDacPercent = ( m_pEEPROM_DATA.fReferance[iRange][iCenter]-fData )
		/( m_pEEPROM_DATA.fReferance[iRange][iCenter]-m_pEEPROM_DATA.fReferance[iRange][iDelta] );
		fDacSet = m_pEEPROM_DATA.s16Dac[iRange][iCenter] -
		(m_pEEPROM_DATA.s16Dac[iRange][iCenter] - m_pEEPROM_DATA.s16Dac[iRange][iDelta]) * fDacPercent ;
	}
	else
	{
		if ( m_pEEPROM_DATA.fReferance[iRange][0]>=fData )
		{
			iCenter = 0;
			iDelta = 1;
		}
		else
		if ( m_pEEPROM_DATA.fReferance[iRange][iMax]<=fData )
		{
			iCenter = iMax-1;
			iDelta = iMax;
		}
		else
		{
			for(;iError;)
			{
				iError--;
				iCenter = (iLeft+iRight)/2;
				if ( iCenter==0 || iCenter>=iMax-1 )
					break;
				if ( m_pEEPROM_DATA.fReferance[iRange][iCenter]<=fData && m_pEEPROM_DATA.fReferance[iRange][iCenter+1]>=fData )
					break;
				if ( fData < m_pEEPROM_DATA.fReferance[iRange][iCenter] )
					iRight = iCenter;
				else
					iLeft = iCenter;
			}
			iDelta = iCenter+1;
		}
		fDacPercent = ( fData - m_pEEPROM_DATA.fReferance[iRange][iCenter]) /
		( m_pEEPROM_DATA.fReferance[iRange][iDelta]-m_pEEPROM_DATA.fReferance[iRange][iCenter] );
		
		fDacSet = m_pEEPROM_DATA.s16Dac[iRange][iCenter] +
		(m_pEEPROM_DATA.s16Dac[iRange][iDelta] - m_pEEPROM_DATA.s16Dac[iRange][iCenter]) * fDacPercent ;
	}
	
	if ( fDacSet<-32768.0f )
		fDacSet = -32768.0;
	else
	if ( fDacSet>32767.0f )
		fDacSet = 32767.0;
	if ( iError == 0 )
		return 0.0f;
	s16Dac = ( fDacSet<0.0f) ? fDacSet-0.5f : fDacSet+0.5f;

	return s16Dac;
}

void ChargePowerControl( void )
{
	if ( m_ucChargePowerControl==_CHARGE_POWER_FIRST )
	{
		SetChargePowerVoltage( 0, 7.5f );
		SetChargePowerVoltage( 1, 7.5f );
		return;
	}
	if ( m_ucChargePowerControl==_CHARGE_POWER_SET && m_bChargePowerControlOne )
	{
		m_bChargePowerControlOne = FALSE;
		if ( m_bStepRunning[0] || m_bStepRunning[1] )
			ChargePowerSet( 0 );
		if ( m_bStepRunning[2] || m_bStepRunning[3] )
			ChargePowerSet( 1 );
	}
}

void SetChargePowerVoltage( U8 ucCh, float fVoltage )
{
	static float fVoltagePre[2];
	
	if ( fVoltagePre[ucCh]==fVoltage )
		return;
	fVoltagePre[ucCh] = fVoltage;
	// 완전임시 윤차장 김부장
	ChargePowerSet( ucCh );
}

void ChargePowerSet( U8 ucCh )
{
	float fFetVoltage1, fFetVoltage2;
	
	fFetVoltage1 = GetFetVoltage( ucCh*2 );
	fFetVoltage2 = GetFetVoltage( ucCh*2+1 );
	if ( fFetVoltage2>fFetVoltage1 )
		fFetVoltage1 = fFetVoltage2;
	SetChargePowerVoltage( ucCh, fFetVoltage1+0.8f );	// 40A at 0.008Ohm x 2.5
}

float GetFetVoltage( U8 ucCh )
{
	// 완전임시 윤차장 김부장	
	//return 5.0f;
	return m_fInternalADCReadVoltage[4+ucCh] * 3.1f;
}

void SetGate( U8 ucBits )
{
	if ( ucBits&(1<<0) )
		PDD_HIGH = _PORT_D_GATE1;
	else
		PDD_LOW = _PORT_D_GATE1;
		
	if ( ucBits&(1<<1) )
		PDD_HIGH = _PORT_D_GATE2;
	else
		PDD_LOW = _PORT_D_GATE2;
		
	if ( ucBits&(1<<2) )
		PDD_HIGH = _PORT_D_GATE3;
	else
		PDD_LOW = _PORT_D_GATE3;
		
	if ( ucBits&(1<<3) )
		PDD_HIGH = _PORT_D_GATE4;
	else
		PDD_LOW = _PORT_D_GATE4;
		
	if ( ucBits&(1<<4) )
		PDD_HIGH = _PORT_D_GATE5;
	else
		PDD_LOW = _PORT_D_GATE5;
		
	if ( ucBits&(1<<5) )
		PDC_HIGH = _PORT_C_GATE6;
	else
		PDC_LOW = _PORT_C_GATE6;
		
	if ( ucBits&(1<<6) )
		PDC_HIGH = _PORT_C_GATE7;
	else
		PDC_LOW = _PORT_C_GATE7;
}

void ClearLoopNowCount(unsigned char ucChannel, uint startIndex, uint endIndex)
{
	int i;
	
	for(i = startIndex ; i< endIndex; i++)
		m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[ucChannel][i].usLoopCountNow = 0;
}

void GetFetTempNVolt2( void )
{
	//#define _VOLTAGE_FACTOR 0.000966796875f //0.6*3.3/2048
	#define _VOLTAGE_FACTOR 0.00088857421875f //0.6*3.033/2048
	static int iMuxSelect = 0;
	int16_t adc_values;
	
	if( ADCIFA_is_eos_sequencer_0() == TRUE)
	{
		adc_values = ADCIFA_read_resx_sequencer_0(0);
		m_fInternalADCAirTempVoltage = adc_values * _VOLTAGE_FACTOR;
		
		adc_values = ADCIFA_read_resx_sequencer_0(1);
		m_fInternalADCReadVoltage[iMuxSelect] = adc_values * _VOLTAGE_FACTOR;
		
		if(++iMuxSelect >= _MUX_MAX_CHANNEL)
		iMuxSelect = 0;
		
		// Mux Select
		iMuxSelect & 0x01 ? (PDA_HIGH = _PORT_A_HEAT_SINK_TEMPERATURE_A0) : (PDA_LOW = _PORT_A_HEAT_SINK_TEMPERATURE_A0);
		iMuxSelect & 0x02 ? (PDA_HIGH = _PORT_A_HEAT_SINK_TEMPERATURE_A1) : (PDA_LOW = _PORT_A_HEAT_SINK_TEMPERATURE_A1);
		iMuxSelect & 0x04 ? (PDA_HIGH = _PORT_A_HEAT_SINK_TEMPERATURE_A2) : (PDA_LOW = _PORT_A_HEAT_SINK_TEMPERATURE_A2);
		
	}
	
	delay_us(1);
	/* Start ADCIFA sequencer 0 */
	AVR32_ADCIFA.cr = AVR32_ADCIFA_CR_SOC0_MASK;	
}
#ifdef SUPPORT_BLACK_OUT
void BlackOutStepEnd( unsigned char ucCh, int ucEndKind )
{
	m_bAlarmParameterSet[ucCh] = FALSE;
	m_ucNowPulseMode[ucCh] = _PULSE_MODE_NONE;
	m_ucNowMode[ucCh] = _MODE_NONE;
}

void PauseResumeControl ( void )
{
  unsigned char i = 0;
  unsigned char eepromWriteFlag = 0;
  
  for (i=0; i<_MAX_CHANNEL; i++)
  {
    if (m_ucPauseSequenceDelay[i] == _DELAY_PAUSE_ON || m_ucPauseSequenceDelay[i] == _DELAY_RESUME_ON || m_ucPauseSequenceDelay[i] == _DELAY_CLEAR)
    {
      eepromWriteFlag = 1;
      break;
    }
  }

  if (eepromWriteFlag == 1)
  {
    for (i=0; i<_MAX_CHANNEL; i++)
    {
      if ( m_bChannelRunning[i] )
      {
        eepromWriteFlag = 0;
      }
    }
  }

  if (eepromWriteFlag == 1)
  {
    m_ucEEPRomControl = _EEPRom_WRITE_CLEAR_PAUSE_INFO;
   
    for (i=0; i<_MAX_CHANNEL; i++)
    {
      if ( m_ucPauseSequenceDelay[i] == _DELAY_PAUSE_ON )
      {
        m_ucEEPRomControl = _EEPRom_WRITE_PAUSE_SYS_RESUME_INFO;
        break;
      }
    }
    
    for (i=0; i<_MAX_CHANNEL; i++)
    {
      if ( m_ucPauseSequenceDelay[i] == _DELAY_PAUSE_ON )
      {
        if ( m_pEEPROM_PAUSE_INFO_DATA.ucIsPauseDataValid[i] == 1 ) m_pEEPROM_PAUSE_INFO_DATA.ucIsPauseDataWritten[i] = 1; 
      }
      else if ( m_ucPauseSequenceDelay[i] == _DELAY_RESUME_ON || m_ucPauseSequenceDelay[i] == _DELAY_CLEAR )
      {
        m_pEEPROM_PAUSE_INFO_DATA.ucIsPauseDataValid[i] = 0;
        m_pEEPROM_PAUSE_INFO_DATA.ucIsPauseDataWritten[i] = 1;  
      }
      m_ucPauseSequenceDelay[i] = _EEPRom_NONE;
    }
  }   
}
#endif
