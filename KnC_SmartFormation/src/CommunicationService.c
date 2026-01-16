/*
 * CommunicationService.c
 *
 * Created: 2021-11-26 AM 00:05:00
 *  Author: 4N Corporation, bgyu
 */ 


#include "KnC_IncludeFile.h"

void PcCommunicationService( void )
{
	int i;
	int iRxTcr;
	int iReceiveCount;
	int iSum;
	int iSumRecv;
	static int iReceiveCountOld;

	iRxTcr = PC_COMM_RX_PDCA.tcr;
	if ( iRxTcr==PC_COMM_RX_PDCA_BUFFER_SIZE )
		return;

	iReceiveCount = PC_COMM_RX_PDCA_BUFFER_SIZE-iRxTcr;
	if ( iReceiveCountOld!=iReceiveCount )
	{
		iReceiveCountOld = iReceiveCount;
		_LED_ALARM_TOGGLE;
	}

	if ( iReceiveCount<(1+1+1+1+1+5) )// size1+mcu1+ch1+cmd1+sum1+etx5
		return;
	
	if (
			m_ucPcCommPDCA_RxBuffer[iReceiveCount-1]!=PC_KPU_COMM_ETX4 ||
			m_ucPcCommPDCA_RxBuffer[iReceiveCount-2]!=PC_KPU_COMM_ETX3 ||
			m_ucPcCommPDCA_RxBuffer[iReceiveCount-3]!=PC_KPU_COMM_ETX2 ||
			m_ucPcCommPDCA_RxBuffer[iReceiveCount-4]!=PC_KPU_COMM_ETX1 ||
			m_ucPcCommPDCA_RxBuffer[iReceiveCount-5]!=PC_KPU_COMM_ETX0 )
	{
		if ( iReceiveCount >= PC_COMM_RX_PDCA_BUFFER_SIZE )
		{
			m_uiSystemInformation[_SystemErrorFlagCounter] += 0x1;
			m_uiSystemInformation[_SystemErrorFlag] |= 1<<SystemErrorFlagPcCommunicationSumCheck2;
			PDCA_ReLoadForUsart( PC_COMM_RX_PDCA_CHANNEL_USART, m_ucPcCommPDCA_RxBuffer, sizeof( m_ucPcCommPDCA_RxBuffer ) );
		}
		return;
	}

	int iCsr = PC_COMM_USART->csr;
	if ( iCsr & (AVR32_USART_CSR_OVRE_MASK | AVR32_USART_CSR_FRAME_MASK | AVR32_USART_CSR_PARE_MASK | AVR32_USART_CSR_MANERR_MASK ) )
	{
		m_uiSystemInformation[_SystemErrorFlagCounter] += 0x10;
		m_uiSystemInformation[_SystemErrorFlag] |= 1<<SystemErrorFlagPcCommunicationParityEtc;
		PC_COMM_USART->cr = AVR32_USART_CR_RSTSTA_MASK;
		PDCA_ReLoadForUsart( PC_COMM_RX_PDCA_CHANNEL_USART, m_ucPcCommPDCA_RxBuffer, sizeof( m_ucPcCommPDCA_RxBuffer ) );
		return;
	}

	if( iReceiveCount-6 != m_ucPcCommPDCA_RxBuffer[0])
	{
		//PC_COMM_USART->cr = AVR32_USART_CR_RSTSTA_MASK;
		m_uiSystemInformation[_SystemErrorFlagCounter] += 0x100;
		m_uiSystemInformation[_SystemErrorFlag] |= 1<<SystemErrorFlagPcCommunicationReceiveContError;
		PDCA_ReLoadForUsart( PC_COMM_RX_PDCA_CHANNEL_USART, m_ucPcCommPDCA_RxBuffer, sizeof( m_ucPcCommPDCA_RxBuffer ) );
		return;
	}

	// 20221124 CheckSum 변경
	
	iSum = 0;
	iReceiveCount -= 6;
	for( i=0; i<iReceiveCount; i++ )
	{
		iSum ^= m_ucPcCommPDCA_RxBuffer[i];
	}
	//iSum &= 0xff;
	iSumRecv = m_ucPcCommPDCA_RxBuffer[i++];
	
	if ( m_ucPcCommPDCA_RxBuffer[1]==m_ucMcuNumber )
	{
		if ( iSumRecv!= iSum )
		{
			m_uiSystemInformation[_SystemErrorFlagCounter] += 0x1000;
			m_uiSystemInformation[_SystemErrorFlag] |= 1<<SystemErrorFlagPcCommunicationSumCheck;
			PDCA_ReLoadForUsart( PC_COMM_RX_PDCA_CHANNEL_USART, m_ucPcCommPDCA_RxBuffer, sizeof( m_ucPcCommPDCA_RxBuffer ) );
			return;
		}
		InterpretPcCommunication( iReceiveCount );		// Communication Ok
	}

	PDCA_ReLoadForUsart( PC_COMM_RX_PDCA_CHANNEL_USART, m_ucPcCommPDCA_RxBuffer, sizeof( m_ucPcCommPDCA_RxBuffer ) );
	_LED_ALARM_OFF;
}

bool SetMcuGpioData( unsigned int uiAddress, U16 usData )
{
	
	bool bRet = true;

	switch( uiAddress )
 	{
		case ADD_SDRAM_CHECK_ENALBE:
			m_uiSdramCheckEnalbe = TRUE;
			m_uiSdramCheckPassFail = 0x1000;
			break;
		
		case ADD_SDRAM_CHECK_DISABLE:
			m_uiSdramCheckEnalbe = FALSE;
			m_uiSdramCheckPassFail = 0x3333;
			break;
		case ADD_FAN_CONTROL_DIRECT:
			if ( usData )
				PDC_LOW = _PORT_C_FAN_OFF_H;
			else
				PDC_HIGH = _PORT_C_FAN_OFF_H;
			break;
		case ADD_FAN_OFF_DELAY:
			FanControl( 0 );
			break;
		default:
			if( usData & 0x1 )
				gpio_set_pin_high( uiAddress & 0xFFF);
			else
				gpio_set_pin_low(uiAddress & 0xFFF);
			break;
		
	}
	return bRet;
}

int  GetMcuGpioData( unsigned int uiAddress )
{
	union32 unionReturn;
	unionReturn.i = 0;
	 
	switch( uiAddress )
	{
		
	case ADD_SDRAM_CHECK_STATUS:
		unionReturn.i = m_uiSdramCheckPassFail;	// 0xF0F0;
		return unionReturn.i;
		
	default:
		unionReturn.i = gpio_get_pin_value( uiAddress & 0xFFF);
		return unionReturn.i;
	}
	 
	return gpio_get_pin_value( uiAddress & 0xFFF);
}

void InterpretPcCommunication( int iReceiveCount )
{
	unsigned short iIndex;
	unsigned short i;
	unsigned char ucChannel;
	unsigned char ucCommand;
	unsigned char ucSum = 0;
	unsigned char ucLastIndex = 4;
	unsigned char ucTemp;
	union ReturnFlagUnion ReturnFlag;
	union ReturnFlagUnion AddressData;
	unsigned char ucAutoReturnFlag = 1; // 0 = None , 1 = 32bit Return, 2 = struct return
	unsigned char ucSize =0;
	unsigned char* pBuf;
	char bError = FALSE;	// Error Is Nothing Communication Data
	union32	u32RedocdCount;		// 2023.04.25	LJK
	#ifdef SUPPORT_BLACK_OUT
  unsigned char ucAllChannelFlag = 0;
  unsigned char j;
  #endif
	
	ReturnFlag.iReturn = 0x12345678;
	ucChannel = m_ucPcCommPDCA_RxBuffer[2];
	ucCommand = m_ucPcCommPDCA_RxBuffer[3];
	if ( ucChannel>=_MAX_CHANNEL )
		ucCommand = 0xff;	// For Error
		
	// 통신 Return Code 0일시 무조건 Error
	switch( ucCommand )	// CMD ?
	{
	case Pc_Comm_GetRunStopAllChannel:	// MSB=Step Index Now(+1,Zero방지), LSB = Running Channel Bits
		//2023.05.02  KYH 남은레코드 갯수 U32 추가
		m_ucPcCommPDCA_TxBuffer[4] = 0x00;	ucSum ^= m_ucPcCommPDCA_TxBuffer[ucLastIndex++];
		m_ucPcCommPDCA_TxBuffer[5] = m_ucStepIndexNow[ucChannel];	ucSum ^= m_ucPcCommPDCA_TxBuffer[ucLastIndex++];
		
		m_ucPcCommPDCA_TxBuffer[6] = 0x00;	ucSum ^= m_ucPcCommPDCA_TxBuffer[ucLastIndex++];
		m_ucPcCommPDCA_TxBuffer[7] = m_bChannelRunning[ucChannel];			//0.bit 0:stop, 1:run
		m_ucPcCommPDCA_TxBuffer[7] |= (m_ucPauseStatus[ucChannel]<<1);		//1.bit 1: pause
		m_ucPcCommPDCA_TxBuffer[7] |= (m_ucCompletStatus[ucChannel]<<2);	//2.bit 1: Complete
		#ifdef SUPPORT_BLACK_OUT
    m_ucPcCommPDCA_TxBuffer[7] |= (m_ucBlackOutFlag[ucChannel]<<2);	//3.bit 1: blackout
    #endif
		ucSum ^= m_ucPcCommPDCA_TxBuffer[ucLastIndex++];
		
		ReturnFlag.uiReturn = GetStepRecordRemainNumber( ucChannel );
		break;
		
	case Pc_Comm_RunControlBitsChannel:
		// 0     1    2				3    4					5
		// Size1 Mcu1 Channel1(x)	Cmd1 ChannelControlBits	StartStepIndex
		ReturnFlag.iReturn = 0;
		if(m_bChannelRunning[ucChannel] == TRUE)
			break;		// 이미 채널이 Run
		
		// 임시로 Start Index 대입 및 조건 검사	
		ucTemp = m_ucPcCommPDCA_RxBuffer[4];
		
		if ( ucTemp >= _MAX_SEQUENCE_STEP )	// SEQUENCE_MAX_STEP Over Error
			break;
		if ( m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[ucChannel][ucTemp].ucState == _STATE_NONE )	// Empty Step Is Error
			break;
		
		ReturnFlag.iReturn = 1;
		
		// Sequence Loop Count Clear
		for( i=0; i<_MAX_SEQUENCE_STEP; i++ )
			m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[ucChannel][i].usLoopCountNow = 0;
		
		m_ucStartStepIndex[ucChannel] = m_ucPcCommPDCA_RxBuffer[4];
		m_bRunSequence[ucChannel] = TRUE;
		//LJK 2023.03.27 일시정지 초기화
		m_ucPauseStatus[ucChannel] = FALSE;
		m_ucCompletStatus[ucChannel] = FALSE;
		
		
		//DCIR Sec Test
		//if( m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[ucChannel][0].ucMode == _MODE_10ms_DCIR)
		//{
		//	m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[ucChannel][0].ucMode = _MODE_Sec_DCIR;
		//	m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[ucChannel][0].uiTestSecond32 = 65 * 10; //65초
		//	m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[ucChannel][0].uiDataRecordSecond32 = 1; //1초 기록
		//	m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[ucChannel][0].u16PulseTime1ms[1] = ( 10 * 1000 ) +6 ; //10초			
		//}
		
		break;
		
	case Pc_Comm_StopControlBitsChannel:	// Stop은 Error 없음
		// 0     1    2        3    4
		// Size1 Mcu1 Channel1 Cmd1 ChannelControlBits
		ReturnFlag.iReturn = 1;
		m_ucStopSequence[ucChannel] = 1;
    #ifdef SUPPORT_BLACK_OUT
    if ( m_ucPauseStatus[ucChannel] == TRUE ) PauseResumeHandler(ucChannel, _PAUSE_NONE);
    m_ucBlackOutFlag[ucChannel] = 0;
    #endif
		//LJK 2023.03.27 일시정지 초기화
		m_ucPauseStatus[ucChannel] = FALSE;	
		break;
		
	case Pc_Comm_ResetStepSequence:
		// 0     1    2        3
		// Size1 Mcu1 Channel1 Cmd1
		ReturnFlag.iReturn = 0;
		
		if ( m_bChannelRunning[ucChannel] )
			break;	// Run시는 Error

    #ifdef SUPPORT_BLACK_OUT
    if ( m_ucPauseStatus[ucChannel] == TRUE ) PauseResumeHandler(ucChannel, _PAUSE_NONE);
    m_ucBlackOutFlag[ucChannel] = 0;
    #endif
		
		ReturnFlag.iReturn = 1;
		m_uiStepDownloadSum[ucChannel] = 0;
		
		for( i=0; i<_MAX_SEQUENCE_STEP; i++ )
			m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[ucChannel][i].ucState = _STATE_NONE;
			
		break;
		
	case Pc_Comm_SetOneStepSequence:
		// 0     1    2        3	4		  5
		// Size1 Mcu1 Channel1 Cmd1 StepIndex Structure
		ReturnFlag.iReturn = 0;
		ucTemp = m_ucPcCommPDCA_RxBuffer[4];
		
		if ( m_bChannelRunning[ucChannel] )
			break;	// Run시는 Error
		if ( ucTemp >= _MAX_SEQUENCE_STEP )	// SEQUENCE_MAX_STEP Over Error
			break;
			
		ReturnFlag.iReturn = 1;
		pBuf = (unsigned char *) & m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[ucChannel][ucTemp];
		memcpy( (char*)pBuf, (char*)&m_ucPcCommPDCA_RxBuffer[5], sizeof(_FORMATIONM_STEP_SEQUENCE) );
		
		//SoC & DoD Test LJK 2024.10.24
		memset(&m_pFORMATION_STEP_END_DATA_STRUCTURE->stStepEndData[ucChannel][ucTemp], 
				0, 
				sizeof(m_pFORMATION_STEP_END_DATA_STRUCTURE->stStepEndData[0][0]));
		if(m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[ucChannel][ucTemp].usLoopCountNow == 0xAA)
		{
			m_pFORMATION_STEP_END_DATA_STRUCTURE->stStepEndData[ucChannel][ucTemp].uiCV_msSecond32 = 0xAA;
			m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[ucChannel][ucTemp].usLoopCountNow = 0;
		}
		//m_pFORMATION_STEP_END_DATA_STRUCTURE->stStepEndData[ucChannel][ucTemp].usLoopCountNow = 0;
			
		if ( ucTemp == 0 )	// First Step At Clear
			m_uiStepDownloadSum[ucChannel] = 0;
			
		for( i=0; i<sizeof(_FORMATIONM_STEP_SEQUENCE); i++ )
			m_uiStepDownloadSum[ucChannel] += pBuf[i];
		
		break;
		
	case Pc_Comm_GetStepSum32:
		// 0     1    2        3
		// Size1 Mcu1 Channel1 Cmd1
		ReturnFlag.uiReturn = m_uiStepDownloadSum[ucChannel];
		break;
		
	case Pc_Comm_Get_EEPROM_Pointer200:
		// 0     1    2        3	4
		// Size1 Mcu1 Channel1 Cmd1 x200Index
		ReturnFlag.iReturn = 0;
		
		iIndex = 200*(unsigned short)m_ucPcCommPDCA_RxBuffer[4];
		if ( iIndex>=sizeof(_EEPROM_DATA) )	// Start Size Over
			break;
		ucSize = ( iIndex+200>=sizeof(_EEPROM_DATA) ) ? sizeof(_EEPROM_DATA)-iIndex : 200;
		if ( !ucSize )
			break;
			
		ReturnFlag.iReturn = 1;
		ucAutoReturnFlag = 2;
		pBuf =((unsigned char*)&m_pEEPROM_DATA+iIndex);
		break;
		
	case Pc_Comm_Set_EEPROM_Pointer200:
		// 0     1    2        3	4		  5
		// Size1 Mcu1 Channel1 Cmd1 x200Index PointerData
		ReturnFlag.iReturn = 0;
		iIndex = 200*(unsigned short)m_ucPcCommPDCA_RxBuffer[4];
		
		if ( iIndex>=sizeof(_EEPROM_DATA) )	// Start Size Over
			break;
		ucSize = ( iIndex+200>=sizeof(_EEPROM_DATA) ) ? sizeof(_EEPROM_DATA)-iIndex : 200;
		if ( !ucSize )
			break;
			
		ReturnFlag.iReturn = 1;
		
		AVR32_ENTER_CRITICAL_REGION();
		memcpy( ((char *)&m_pEEPROM_DATA)+iIndex, (char*)&m_ucPcCommPDCA_RxBuffer[5], ucSize );
		AVR32_LEAVE_CRITICAL_REGION();
		
		break;
		
	case Pc_Comm_GetLastOneSecondCondition:
		// 0     1    2        3	4
		// Size1 Mcu1 Channel1 Cmd1	4Channel Data	// return 4-Channel Data 4(H)+228(D)+1(S)+5(E)
		MakeStepDataOne_Channel( ucChannel, 0 );
		pBuf =(unsigned char*)(&m_DataNow[ucChannel]);
		ucSize = sizeof(m_DataNow[ucChannel]);
		
		if ( !ucSize )
			break;
			
		ucAutoReturnFlag = 2;
		ReturnFlag.iReturn = 1;
		break;
		
	case Pc_Comm_GetDataRecordOne:
		// 0     1    2        3	4		5
		// Size1 Mcu1 Channel1 Cmd1	Remain# Pointer
		ReturnFlag.iReturn = 1;
		u32RedocdCount.ui = GetStepRecordRemainNumber( ucChannel );
		
		m_ucPcCommPDCA_TxBuffer[4] = u32RedocdCount.uc[0];	ucSum ^= m_ucPcCommPDCA_TxBuffer[ucLastIndex++];
		m_ucPcCommPDCA_TxBuffer[5] = u32RedocdCount.uc[1];	ucSum ^= m_ucPcCommPDCA_TxBuffer[ucLastIndex++];
		m_ucPcCommPDCA_TxBuffer[6] = u32RedocdCount.uc[2];	ucSum ^= m_ucPcCommPDCA_TxBuffer[ucLastIndex++];
		m_ucPcCommPDCA_TxBuffer[7] = u32RedocdCount.uc[3];	ucSum ^= m_ucPcCommPDCA_TxBuffer[ucLastIndex++];
		
		pBuf =(unsigned char*) GetStepRecordHeadPointer( ucChannel );
		ucSize = sizeof( _FORMATIONM_STEP_RECORD_DATA );
		ucAutoReturnFlag = 2;
		break;
		
    case Pc_Comm_DeleteDataRecordOne:
		// 0     1    2        3	4
		// Size1 Mcu1 Channel1 Cmd1	Remain#
		ReturnFlag.iReturn = GetStepRecordRemainNumber( ucChannel );
		DeletePopStepRecordData( ucChannel );
		u32RedocdCount.ui = GetStepRecordRemainNumber( ucChannel );
		
		m_ucPcCommPDCA_TxBuffer[4] = u32RedocdCount.uc[0];	ucSum ^= m_ucPcCommPDCA_TxBuffer[ucLastIndex++];
		m_ucPcCommPDCA_TxBuffer[5] = u32RedocdCount.uc[1];	ucSum ^= m_ucPcCommPDCA_TxBuffer[ucLastIndex++];
		m_ucPcCommPDCA_TxBuffer[6] = u32RedocdCount.uc[2];	ucSum ^= m_ucPcCommPDCA_TxBuffer[ucLastIndex++];
		m_ucPcCommPDCA_TxBuffer[7] = u32RedocdCount.uc[3];	ucSum ^= m_ucPcCommPDCA_TxBuffer[ucLastIndex++];
		break;
		
	case Pc_Comm_DeleteDataRecordAll:
		// 0     1    2        3	
		// Size1 Mcu1 Channel1 Cmd1	
		ReturnFlag.iReturn = 1;
		DeletePopStepRecordDataAll( ucChannel );
		break;
		
	case Pc_Comm_MaskAlarmCondition:
		// 0     1    2        3	4
		// Size1 Mcu1 Channel1 Cmd1	int32 
		ReturnFlag.iReturn = 1;
		memcpy( (char*)&m_uiAlarmMask[ucChannel], (char*)&m_ucPcCommPDCA_RxBuffer[4], 4 );
		break;
		
	case Pc_Comm_EEPRomControl:
		// 0     1    2        3	4
		// Size1 Mcu1 Channel1 Cmd1	EEProm Cmd1
		ReturnFlag.iReturn = 1;
		if ( m_ucPcCommPDCA_RxBuffer[4] == _HARDWARE_INITIALIZE )
			m_bHardWareInit = TRUE;
		else
			m_ucEEPRomControl = m_ucPcCommPDCA_RxBuffer[4];
		break;
		
	case Pc_Comm_SetGpIo:
		// 0     1    2        3	4		 8
		// Size1 Mcu1 Channel1 Cmd1	Address4 Data1
		AddressData.u8Return[0] = m_ucPcCommPDCA_RxBuffer[4];
		AddressData.u8Return[1] = m_ucPcCommPDCA_RxBuffer[5];
		AddressData.u8Return[2] = m_ucPcCommPDCA_RxBuffer[6];
		AddressData.u8Return[3] = m_ucPcCommPDCA_RxBuffer[7];
		SetMcuGpioData( AddressData.iReturn, m_ucPcCommPDCA_RxBuffer[8] );
		ReturnFlag.iReturn = GetMcuGpioData( AddressData.iReturn );
		break;
		
	case Pc_Comm_GetGpIo:
		// 0     1    2        3	4
		// Size1 Mcu1 Channel1 Cmd1	Address4
		AddressData.u8Return[0] = m_ucPcCommPDCA_RxBuffer[4];
		AddressData.u8Return[1] = m_ucPcCommPDCA_RxBuffer[5];
		AddressData.u8Return[2] = m_ucPcCommPDCA_RxBuffer[6];
		AddressData.u8Return[3] = m_ucPcCommPDCA_RxBuffer[7];
		ReturnFlag.iReturn = GetMcuGpioData( AddressData.iReturn );
		break;
		
	case Pc_Comm_SetChargePower:
		// 0     1    2        3	4
		// Size1 Mcu1 Channel1 Cmd1	Voltage4
		AddressData.u8Return[0] = m_ucPcCommPDCA_RxBuffer[4];
		AddressData.u8Return[1] = m_ucPcCommPDCA_RxBuffer[5];
		AddressData.u8Return[2] = m_ucPcCommPDCA_RxBuffer[6];
		AddressData.u8Return[3] = m_ucPcCommPDCA_RxBuffer[7];
		ReturnFlag.iReturn = 1;
		SetChargePowerVoltage( (m_ucPcCommPDCA_RxBuffer[2]/2)%2, AddressData.fReturn );
		break;
		
	case Pc_Comm_GetChargePowerFet:
		// 0     1    2        3
		// Size1 Mcu1 Channel1 Cmd1
		ReturnFlag.fReturn = GetFetVoltage( m_ucPcCommPDCA_RxBuffer[2]%_MAX_CHANNEL );
		break;
		
	case Pc_Comm_SetGate:
		// 0     1    2        3	4
		// Size1 Mcu1 Channel1 Cmd1 ControlByte1
		ReturnFlag.iReturn = 1;
		SetGate( m_ucPcCommPDCA_RxBuffer[4] );
		break;
		
	case Pc_Comm_SetDac:
		// 0     1    2        3	4		5
		// Size1 Mcu1 Channel1 Cmd1	DAC Ch	S16
		AddressData.u8Return[0] = m_ucPcCommPDCA_RxBuffer[5];
		AddressData.u8Return[1] = m_ucPcCommPDCA_RxBuffer[6];
		SetDacWriteData( m_ucPcCommPDCA_RxBuffer[4], AddressData.u16Return[0] );
		_LDAC_ENABLE;
		ucAutoReturnFlag = 3;
		ReturnFlag.iReturn = m_iDacReadBack;
		break;
		
	case Pc_Comm_StartCalibrationMode:
		// 0     1    2        3
		// Size1 Mcu1 Channel1 Cmd1
		FanControl( 1 );
		//m_ui1SecRingCountVoltage = m_ui1SecRingCountCurrent = m_ui1SecRingCountVoltage2 = 0;
		ReturnFlag.iReturn = 1;
		break;
		
	case Pc_Comm_GetCalibrationAdc:
		// 0     1    2        3	4	 8    12
		// Size1 Mcu1 Channel1 Cmd1 VADC IADC V2ADC
		ucTemp = m_ucPcCommPDCA_RxBuffer[2]%_MAX_CHANNEL;
		ReturnFlag.iReturn = m_iVoltage1SecTotal[ucTemp];
		//ReturnFlag.iReturn = m_ui1SecRingCountVoltage;
		m_ucPcCommPDCA_RxBuffer[4] = ReturnFlag.u8Return[0];
		m_ucPcCommPDCA_RxBuffer[5] = ReturnFlag.u8Return[1];
		m_ucPcCommPDCA_RxBuffer[6] = ReturnFlag.u8Return[2];
		m_ucPcCommPDCA_RxBuffer[7] = ReturnFlag.u8Return[3];
		ReturnFlag.iReturn = m_iCurrent1SecTotal[ucTemp];
		m_ucPcCommPDCA_RxBuffer[8] = ReturnFlag.u8Return[0];
		m_ucPcCommPDCA_RxBuffer[9] = ReturnFlag.u8Return[1];
		m_ucPcCommPDCA_RxBuffer[10] = ReturnFlag.u8Return[2];
		m_ucPcCommPDCA_RxBuffer[11] = ReturnFlag.u8Return[3];
		ReturnFlag.iReturn = m_iVoltage2_1SecTotal[ucTemp];
		m_ucPcCommPDCA_RxBuffer[12] = ReturnFlag.u8Return[0];
		m_ucPcCommPDCA_RxBuffer[13] = ReturnFlag.u8Return[1];
		m_ucPcCommPDCA_RxBuffer[14] = ReturnFlag.u8Return[2];
		m_ucPcCommPDCA_RxBuffer[15] = ReturnFlag.u8Return[3];
		pBuf = &m_ucPcCommPDCA_RxBuffer[4];
		ucSize = 12;
		ucAutoReturnFlag = 2;
		break;
		
	case Pc_Comm_GetSystemInformationData:
		pBuf = (unsigned char*)m_uiSystemInformation;
		ucSize = sizeof( m_uiSystemInformation );
		ucAutoReturnFlag = 2;
		break;
		
	case Pc_Comm_AllChannelReset:	// Stop은 Error 없음
		// 0     1    2        3 
		// Size1 Mcu1 Channel1 Cmd1
		ReturnFlag.iReturn = 1;
		for( i=0; i<_MAX_CHANNEL; i++ )
		{
			m_bChannelRunning[i] = TRUE;
			m_bPreMcOnOff[i] = TRUE;
		}
		m_ucStopSequence[ucChannel] = TRUE;
		break;
		
	case Pc_Comm_SetHardWareInitialize:
		m_bHardWareInit = TRUE;
		ucAutoReturnFlag = 1;
		break;
		
	case Pc_Comm_GetDacValue:
		// 0     1    2        3	4							5~8
		// Size1 Mcu1 Channel1 Cmd1	CALIBRATION_MAX_RANGE Index float
		// 0     1    2        3	4~5		// Return
		// Size1 Mcu1 Channel1 Cmd1	Short Dac
		AddressData.u8Return[0] = m_ucPcCommPDCA_RxBuffer[5];
		AddressData.u8Return[1] = m_ucPcCommPDCA_RxBuffer[6];
		AddressData.u8Return[2] = m_ucPcCommPDCA_RxBuffer[7];
		AddressData.u8Return[3] = m_ucPcCommPDCA_RxBuffer[8];
		ReturnFlag.iReturn = GetVoltageCurrentToDacHex( m_ucPcCommPDCA_RxBuffer[4], AddressData.fReturn );
		m_ucPcCommPDCA_RxBuffer[4] = 0;
		m_ucPcCommPDCA_RxBuffer[5] = 0;
		m_ucPcCommPDCA_RxBuffer[6] = ReturnFlag.u8Return[2];
		m_ucPcCommPDCA_RxBuffer[7] = ReturnFlag.u8Return[3];
		pBuf = &m_ucPcCommPDCA_RxBuffer[4];
		ucSize = 4;
		ucAutoReturnFlag = 2;
		break;
		
	case Pc_Comm_GetVIV2:
		// 0     1    2        3	4	    8		12
		// Size1 Mcu1 Channel1 Cmd1 Voltage Current Voltage2
		ucTemp = m_ucPcCommPDCA_RxBuffer[2]%_MAX_CHANNEL;
//		ReturnFlag.fReturn = m_fRecordVoltage[ucTemp];
		ReturnFlag.fReturn = m_fVoltage1Sec[ucTemp];
		m_ucPcCommPDCA_RxBuffer[4] = ReturnFlag.u8Return[0];
		m_ucPcCommPDCA_RxBuffer[5] = ReturnFlag.u8Return[1];
		m_ucPcCommPDCA_RxBuffer[6] = ReturnFlag.u8Return[2];
		m_ucPcCommPDCA_RxBuffer[7] = ReturnFlag.u8Return[3];
		ReturnFlag.fReturn = m_fCurrent1Sec[ucTemp];
		m_ucPcCommPDCA_RxBuffer[8] = ReturnFlag.u8Return[0];
		m_ucPcCommPDCA_RxBuffer[9] = ReturnFlag.u8Return[1];
		m_ucPcCommPDCA_RxBuffer[10] = ReturnFlag.u8Return[2];
		m_ucPcCommPDCA_RxBuffer[11] = ReturnFlag.u8Return[3];
		ReturnFlag.fReturn = m_fVoltage2_1Sec[ucTemp];
		m_ucPcCommPDCA_RxBuffer[12] = ReturnFlag.u8Return[0];
		m_ucPcCommPDCA_RxBuffer[13] = ReturnFlag.u8Return[1];
		m_ucPcCommPDCA_RxBuffer[14] = ReturnFlag.u8Return[2];
		m_ucPcCommPDCA_RxBuffer[15] = ReturnFlag.u8Return[3];
		pBuf = &m_ucPcCommPDCA_RxBuffer[4];
		ucSize = 12;
		ucAutoReturnFlag = 2;
		break;
		
	case Pc_Comm_SetV2ModeCurrent:
		// 0     1    2        3 
		// Size1 Mcu1 Channel1 Cmd1
		for(i=0; i< _MAX_CHANNEL; i++)
		{
			m_bVoltage2Mode[i] = FALSE;
			gpio_set_pin_low(m_uiVoltage2ModeAddress[i]);
		}
		ReturnFlag.iReturn = 1;
		break;
		
	case Pc_Comm_SetV2ModeVoltage:
		// 0     1    2        3 
		// Size1 Mcu1 Channel1 Cmd1
		for(i=0; i< _MAX_CHANNEL; i++)
		{
			m_bVoltage2Mode[i] = TRUE;
			gpio_set_pin_high(m_uiVoltage2ModeAddress[i]);
		}
		ReturnFlag.iReturn = 1;
		break;
		
	case Pc_Comm_ClearLoopCounterAll:
		StepSequence_ClearLoopCounterAll(ucChannel);
		ReturnFlag.iReturn = 1;
		break;
		
	case Pc_Comm_SetStartMode:
		ReturnFlag.iReturn = 0;
		
		for( i=0; i<_MAX_CHANNEL; i++ )
			if ( m_bChannelRunning[i] )
				break;
		
		if(m_ucPcCommPDCA_RxBuffer[4] == 0)
		{
			PDC_LOW =_PORT_C_I_TR_Soft0_Fast1;
			_LED_RUN_OFF;
		}
		else 
		{
			PDC_HIGH = _PORT_C_I_TR_Soft0_Fast1;
			_LED_RUN_ON;
		}
		
		ReturnFlag.iReturn = 1;
		break;
		
	case Pc_Comm_GetStepEndData:
		//// 0     1    2        3	4		5
		//// Size1 Mcu1 Channel1 Cmd1	Remain# Pointer
		ucTemp = m_ucPcCommPDCA_RxBuffer[4];
		
		pBuf = (unsigned char*) & m_pFORMATION_STEP_END_DATA_STRUCTURE->stStepEndData[ucChannel][ucTemp];
		ucSize = sizeof(_FORMATIONM_STEP_END_DATA);
		ucAutoReturnFlag = 2;
		break;
		
	case Pc_Comm_InitSystmeInformationData:
		// Version을 제외한 나머지 Clear
		for(i=1; i< _SystemParameterIndexLast ; i++)
			memset(&m_uiSystemInformation[i], 0, sizeof(m_uiSystemInformation[i]));
		
		ReturnFlag.iReturn = 1;
		break;
	
	case PC_Comm_SendTemperature:
		//memcpy( ((char *)m_pEEPROM_DATA)+iIndex, (char*)&m_ucPcCommPDCA_RxBuffer[5], ucSize );
		memcpy(&m_shCurrentCellTemp[ucChannel] ,&m_ucPcCommPDCA_RxBuffer[4], sizeof(short));
		
		ReturnFlag.iReturn = 1;
		break;

	case Pc_Comm_PauseResume:
		//LJK 2023.03.20
		if( m_ucPcCommPDCA_RxBuffer[4] == m_ucPauseSequence[ucChannel] )
		{
			ReturnFlag.iReturn = m_ucPauseSequence[ucChannel];
			break;
		}
		else if( m_ucPcCommPDCA_RxBuffer[4] == _PAUSE_ON )
		{
		  #ifdef SUPPORT_BLACK_OUT
      PauseResumeHandler(ucChannel, m_ucPcCommPDCA_RxBuffer[4]);
      #else
			m_ucPauseSequence[ucChannel] = m_ucPcCommPDCA_RxBuffer[4];
			m_ucPauseStatus[ucChannel] = TRUE;	
      #endif
		}
		else if( m_ucPcCommPDCA_RxBuffer[4] == _RESUME_ON && m_ucPauseStatus[ucChannel] )
		{
		  #ifdef SUPPORT_BLACK_OUT
      PauseResumeHandler(ucChannel, m_ucPcCommPDCA_RxBuffer[4]);
      #else
			m_ucPauseSequence[ucChannel] = m_ucPcCommPDCA_RxBuffer[4];
			m_ucPauseStatus[ucChannel] = FALSE;
      #endif
		}
		else if( m_ucPcCommPDCA_RxBuffer[4] == _RESUME_NEXT_STEP && m_ucPauseStatus[ucChannel] )
		{
		  #ifdef SUPPORT_BLACK_OUT
      PauseResumeHandler(ucChannel, m_ucPcCommPDCA_RxBuffer[4]);
      #else
			m_ucFirstStep[ucChannel] = TRUE;
			m_bChannelRunning[ucChannel] = TRUE;
			m_ucPreState[ucChannel] = !m_ucNowRestChargeDisCharge[ucChannel];
			m_bMcControlAllComplete[ucChannel] = FALSE;
			m_bStepRunning[ucChannel] = FALSE;
			
			m_bContinueNextSequence[ucChannel] = 1;
			m_ucPauseStatus[ucChannel] = FALSE;
      #endif
		}		
		ReturnFlag.iReturn = m_ucPcCommPDCA_RxBuffer[4];
		break;
		
	case Pc_Comm_GetAirTemp:
		ReturnFlag.fReturn = m_fInternalADCAirTempVoltage;
		break;

	case Pc_Comm_GetHeatSink:
		pBuf =(unsigned char*)m_fInternalADCReadVoltage;
		ucSize = sizeof(m_fInternalADCReadVoltage);
		ucAutoReturnFlag = 2;
		break;	
		
	case Pc_Comm_SetCellTemp:
		memcpy(&m_shMaxCellTemp[ucChannel] ,&m_ucPcCommPDCA_RxBuffer[4], sizeof(short));
		m_shMaxCellTemp[0] = m_shMaxCellTemp[1] = m_shMaxCellTemp[2] = m_shMaxCellTemp[3] = m_shMaxCellTemp[ucChannel];
		ReturnFlag.iReturn = m_shMaxCellTemp[ucChannel];
		break;

	case PC_Comm_SetVoltage:	// 2025.04.23 Alarm Test
		ReturnFlag.iReturn = 0;
		if ( m_bChannelRunning[ucChannel] )
		{
			AddressData.u8Return[0] = m_ucPcCommPDCA_RxBuffer[4];
			AddressData.u8Return[1] = m_ucPcCommPDCA_RxBuffer[5];
			AddressData.u8Return[2] = m_ucPcCommPDCA_RxBuffer[6];
			AddressData.u8Return[3] = m_ucPcCommPDCA_RxBuffer[7];
		
			S16 s16DacVoltage = GetVoltageCurrentToDacHex( VOLTAGE_RANGE_INDEX+ucChannel,  AddressData.fReturn );		// Voltage
			SetDacWriteData( _DAC_CH1_CV+ucChannel*2, s16DacVoltage );
			_LDAC_ENABLE;
			ReturnFlag.iReturn = 1;
		}
		break;
	
	case PC_Comm_SetCurrent:	// 2025.04.23 Alarm Test
		ReturnFlag.iReturn = 0;
		if ( m_bChannelRunning[ucChannel])
		{
			AddressData.u8Return[0] = m_ucPcCommPDCA_RxBuffer[4];
			AddressData.u8Return[1] = m_ucPcCommPDCA_RxBuffer[5];
			AddressData.u8Return[2] = m_ucPcCommPDCA_RxBuffer[6];
			AddressData.u8Return[3] = m_ucPcCommPDCA_RxBuffer[7];
			
			//CCCV OCP, UCP Alarm Test
			//m_fErrorCurrentLow[ucChannel] = AddressData.fReturn - 0.125f;
			//m_fErrorCurrentHigh[ucChannel] = AddressData.fReturn + 0.125f;
			
			
			S16 s16DacCurrent;
			if(m_ucNowRestChargeDisCharge[ucChannel] == _NOW_STATE_DISCHARGE)
				s16DacCurrent = GetVoltageCurrentToDacHex( CURRENT_RANGE_INDEX+ucChannel*2, AddressData.fReturn );	// DisCharge Current
			else
				s16DacCurrent = GetVoltageCurrentToDacHex( CURRENT_RANGE_INDEX+ucChannel*2+1, AddressData.fReturn );	// Charge Current
			SetDacWriteData( _DAC_CH1_CC+ucChannel*2, s16DacCurrent );
			_LDAC_ENABLE;
			
			ReturnFlag.iReturn = 1;
		}
		break;
	
	case PC_Comm_SetResistance:	// 2025.04.23 Alarm Test
		ReturnFlag.iReturn = 0;
		if ( m_bChannelRunning[ucChannel] )
		{
			AddressData.u8Return[0] = m_ucPcCommPDCA_RxBuffer[4];
			AddressData.u8Return[1] = m_ucPcCommPDCA_RxBuffer[5];
			AddressData.u8Return[2] = m_ucPcCommPDCA_RxBuffer[6];
			AddressData.u8Return[3] = m_ucPcCommPDCA_RxBuffer[7];
            
            #ifdef MODIFY_ALARM_CONDITION
            if (m_ucNowMode[ucChannel] == _MODE_CRCV)
            {
		        m_pSeqNow[ucChannel]->fSettingCurrent = AddressData.fReturn;
			    ReturnFlag.iReturn = 1;
		    }
		    #else
			//CRCV Mode ORP, URP Alarm Test
			m_fAlarmResisterLow[ucChannel] = AddressData.fReturn * 0.98f;
			m_fAlarmResisterHigh[ucChannel] = AddressData.fReturn * 1.02f;
			ReturnFlag.iReturn = 1;
			#endif
		}
		break;
	
	case PC_Comm_SetWatt:	// 2025.04.23 Alarm Test
		ReturnFlag.iReturn = 0;
		if ( m_bChannelRunning[ucChannel] )
		{
			AddressData.u8Return[0] = m_ucPcCommPDCA_RxBuffer[4];
			AddressData.u8Return[1] = m_ucPcCommPDCA_RxBuffer[5];
			AddressData.u8Return[2] = m_ucPcCommPDCA_RxBuffer[6];
			AddressData.u8Return[3] = m_ucPcCommPDCA_RxBuffer[7];
		
            #ifdef MODIFY_ALARM_CONDITION
            if (m_ucNowMode[ucChannel] == _MODE_CPCV)
            {
		        m_pSeqNow[ucChannel]->fSettingCurrent = AddressData.fReturn;
			    ReturnFlag.iReturn = 1;
		    }
		    #else
			//CPCV Mode OWP, UWP Alarm Test
			m_fAlarmWattLow[ucChannel] = AddressData.fReturn * 0.98f;
			m_fAlarmWattHigh[ucChannel] = AddressData.fReturn * 1.02f;
			ReturnFlag.iReturn = 1;
			#endif
		}
		break;
	
	case PC_Comm_SetHeatSink:	// 2025.04.23 Alarm Test
		ReturnFlag.iReturn = 0;
		if ( m_bChannelRunning[ucChannel] )
		{
			AddressData.u8Return[0] = m_ucPcCommPDCA_RxBuffer[4];
			AddressData.u8Return[1] = m_ucPcCommPDCA_RxBuffer[5];
			AddressData.u8Return[2] = m_ucPcCommPDCA_RxBuffer[6];
			AddressData.u8Return[3] = m_ucPcCommPDCA_RxBuffer[7];
			//HeatSink
			m_fHeatSinkTemp[ucChannel] = AddressData.fReturn;
			ReturnFlag.iReturn = 1;
		}
		break;
	case PC_Comm_SetAirTemp:	// 2025.04.23 Alarm Test
		ReturnFlag.iReturn = 0;
		if ( m_bChannelRunning[ucChannel] )
		{
			AddressData.u8Return[0] = m_ucPcCommPDCA_RxBuffer[4];
			AddressData.u8Return[1] = m_ucPcCommPDCA_RxBuffer[5];
			AddressData.u8Return[2] = m_ucPcCommPDCA_RxBuffer[6];
			AddressData.u8Return[3] = m_ucPcCommPDCA_RxBuffer[7];
			//Air Temp
			m_fAirTemp = AddressData.fReturn;
			ReturnFlag.iReturn = 1;
		}
		break;

    #ifdef SUPPORT_PROTECTION_CONDITION
    case PC_Comm_SetProtection :        
        ReturnFlag.iReturn = 0; 
        
		if(m_bChannelRunning[ucChannel]) break;
		
		memcpy( &m_pProtectionType[ucChannel], &m_ucPcCommPDCA_RxBuffer[4], sizeof(m_pProtectionType[ucChannel]));		
        ReturnFlag.iReturn = 1;  //OK
        break;
    #endif

    #ifdef SUPPORT_BLACK_OUT
    case Pc_Comm_SetResumeStepSequence:
      // 0     1    2        3	   4		     5          6
  		// Size1 Mcu1 Channel1 Cmd1  AllCannel StepIndex  Structure
  		ReturnFlag.iReturn = 0;
      ucAllChannelFlag = m_ucPcCommPDCA_RxBuffer[4];
      ucTemp = m_ucPcCommPDCA_RxBuffer[5];

  		if ( ucTemp >= _MAX_SEQUENCE_STEP )	// SEQUENCE_MAX_STEP Over Error
  			break;
  			
  		ReturnFlag.iReturn = 1;
      if (1 == ucAllChannelFlag)
      {
        for ( j=0; j<_MAX_CHANNEL; j++ )
        {
          pBuf = (unsigned char *) & m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[j][ucTemp];
      		memcpy( (char*)pBuf, (char*)&m_ucPcCommPDCA_RxBuffer[6], sizeof(_FORMATIONM_STEP_SEQUENCE) );
      		
      		//SoC & DoD Test LJK 2024.10.24
      		memset(&m_pFORMATION_STEP_END_DATA_STRUCTURE->stStepEndData[j][ucTemp], 
      				0, 
      				sizeof(m_pFORMATION_STEP_END_DATA_STRUCTURE->stStepEndData[0][0]));
      		if(m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[j][ucTemp].usLoopCountNow == 0xAA)
      		{
      			m_pFORMATION_STEP_END_DATA_STRUCTURE->stStepEndData[j][ucTemp].uiCV_msSecond32 = 0xAA;
      			m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[j][ucTemp].usLoopCountNow = 0;
      		}
      		//m_pFORMATION_STEP_END_DATA_STRUCTURE->stStepEndData[ucChannel][ucTemp].usLoopCountNow = 0;
      			
      		if ( ucTemp == 0 )	// First Step At Clear
      			m_uiStepDownloadSum[j] = 0;
      			
      		for( i=0; i<sizeof(_FORMATIONM_STEP_SEQUENCE); i++ )
      			m_uiStepDownloadSum[j] += pBuf[i];
        }
      }
      else
      {
    		pBuf = (unsigned char *) & m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[ucChannel][ucTemp];
    		memcpy( (char*)pBuf, (char*)&m_ucPcCommPDCA_RxBuffer[6], sizeof(_FORMATIONM_STEP_SEQUENCE) );
    		
    		//SoC & DoD Test LJK 2024.10.24
    		memset(&m_pFORMATION_STEP_END_DATA_STRUCTURE->stStepEndData[ucChannel][ucTemp], 
    				0, 
    				sizeof(m_pFORMATION_STEP_END_DATA_STRUCTURE->stStepEndData[0][0]));
    		if(m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[ucChannel][ucTemp].usLoopCountNow == 0xAA)
    		{
    			m_pFORMATION_STEP_END_DATA_STRUCTURE->stStepEndData[ucChannel][ucTemp].uiCV_msSecond32 = 0xAA;
    			m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[ucChannel][ucTemp].usLoopCountNow = 0;
    		}
    		//m_pFORMATION_STEP_END_DATA_STRUCTURE->stStepEndData[ucChannel][ucTemp].usLoopCountNow = 0;
    			
    		if ( ucTemp == 0 )	// First Step At Clear
    			m_uiStepDownloadSum[ucChannel] = 0;
    			
    		for( i=0; i<sizeof(_FORMATIONM_STEP_SEQUENCE); i++ )
    			m_uiStepDownloadSum[ucChannel] += pBuf[i];
      }
  		
  		break;
      
    #endif
	}
	
	if ( bError )
		return;

	m_ucPcCommPDCA_TxBuffer[1] = m_ucPcCommPDCA_RxBuffer[1];	ucSum ^= m_ucPcCommPDCA_TxBuffer[1];
	m_ucPcCommPDCA_TxBuffer[2] = m_ucPcCommPDCA_RxBuffer[2];	ucSum ^= m_ucPcCommPDCA_TxBuffer[2];
	m_ucPcCommPDCA_TxBuffer[3] = m_ucPcCommPDCA_RxBuffer[3];	ucSum ^= m_ucPcCommPDCA_TxBuffer[3];
	
	if ( ucAutoReturnFlag==3 )
	{
		//m_ucPcCommPDCA_TxBuffer[ucLastIndex] = ReturnFlag.u8Return[3];		ucSum += m_ucPcCommPDCA_TxBuffer[ucLastIndex++];
		//m_ucPcCommPDCA_TxBuffer[ucLastIndex] = ReturnFlag.u8Return[2];		ucSum += m_ucPcCommPDCA_TxBuffer[ucLastIndex++];
		
		m_ucPcCommPDCA_TxBuffer[ucLastIndex] = ReturnFlag.u8Return[3];		ucSum ^= m_ucPcCommPDCA_TxBuffer[ucLastIndex++];
		m_ucPcCommPDCA_TxBuffer[ucLastIndex] = ReturnFlag.u8Return[2];		ucSum ^= m_ucPcCommPDCA_TxBuffer[ucLastIndex++];
	}
	else
	if ( ucAutoReturnFlag==1 )
	{
		//m_ucPcCommPDCA_TxBuffer[ucLastIndex] = ReturnFlag.u8Return[3];		ucSum += m_ucPcCommPDCA_TxBuffer[ucLastIndex++];
		//m_ucPcCommPDCA_TxBuffer[ucLastIndex] = ReturnFlag.u8Return[2];		ucSum += m_ucPcCommPDCA_TxBuffer[ucLastIndex++];
		//m_ucPcCommPDCA_TxBuffer[ucLastIndex] = ReturnFlag.u8Return[1];		ucSum += m_ucPcCommPDCA_TxBuffer[ucLastIndex++];
		//m_ucPcCommPDCA_TxBuffer[ucLastIndex] = ReturnFlag.u8Return[0];		ucSum += m_ucPcCommPDCA_TxBuffer[ucLastIndex++];
		
		m_ucPcCommPDCA_TxBuffer[ucLastIndex] = ReturnFlag.u8Return[0];		ucSum ^= m_ucPcCommPDCA_TxBuffer[ucLastIndex++];
		m_ucPcCommPDCA_TxBuffer[ucLastIndex] = ReturnFlag.u8Return[1];		ucSum ^= m_ucPcCommPDCA_TxBuffer[ucLastIndex++];
		m_ucPcCommPDCA_TxBuffer[ucLastIndex] = ReturnFlag.u8Return[2];		ucSum ^= m_ucPcCommPDCA_TxBuffer[ucLastIndex++];
		m_ucPcCommPDCA_TxBuffer[ucLastIndex] = ReturnFlag.u8Return[3];		ucSum ^= m_ucPcCommPDCA_TxBuffer[ucLastIndex++];
		
	}
	else
	if ( ucAutoReturnFlag == 2 )	// Send Struct
	{
		if( ucCommand == Pc_Comm_Get_EEPROM_Pointer200 )
		{
			AVR32_ENTER_CRITICAL_REGION();
			for(;ucSize>0; ucSize--)	// Contents
			{
				m_ucPcCommPDCA_TxBuffer[ucLastIndex] = *pBuf;
				ucSum ^= m_ucPcCommPDCA_TxBuffer[ucLastIndex++];
				pBuf++;
			}
			AVR32_LEAVE_CRITICAL_REGION();
		}
		else
		{
			for(;ucSize>0; ucSize--)	// Contents
			{
				m_ucPcCommPDCA_TxBuffer[ucLastIndex] = *pBuf;
				ucSum ^= m_ucPcCommPDCA_TxBuffer[ucLastIndex++];
				pBuf++;
			}
		}
	}

	m_ucPcCommPDCA_TxBuffer[0] = ucLastIndex;								
	ucSum ^= m_ucPcCommPDCA_TxBuffer[0];
			
	m_ucPcCommPDCA_TxBuffer[ucLastIndex++] = ucSum;
	m_ucPcCommPDCA_TxBuffer[ucLastIndex++] = PC_KPU_COMM_ETX0;
	m_ucPcCommPDCA_TxBuffer[ucLastIndex++] = PC_KPU_COMM_ETX1;
	m_ucPcCommPDCA_TxBuffer[ucLastIndex++] = PC_KPU_COMM_ETX2;
	m_ucPcCommPDCA_TxBuffer[ucLastIndex++] = PC_KPU_COMM_ETX3;
	m_ucPcCommPDCA_TxBuffer[ucLastIndex++] = PC_KPU_COMM_ETX4;
	
	PDCA_ReLoadForUsart( PC_COMM_TX_PDCA_CHANNEL_USART, m_ucPcCommPDCA_TxBuffer, ucLastIndex );
}

void StepSequence_ClearLoopCounterAll( unsigned char ucCh )
{
	int i;
	for( i=0; i<_MAX_SEQUENCE_STEP; i++ )
		m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[ucCh][i].usLoopCountNow = 0;
}

#ifdef SUPPORT_BLACK_OUT
void PauseResumeHandler ( unsigned char ucCh, unsigned char param )
{
  if( param == _PAUSE_ON )
	{
		m_ucPauseSequence[ucCh] = param;
		m_ucPauseStatus[ucCh] = TRUE;	
    
    m_pEEPROM_PAUSE_INFO_DATA.ucIsPauseDataValid[ucCh] = 1;
    m_pEEPROM_PAUSE_INFO_DATA.ucIsPauseDataWritten[ucCh] = 0;
    m_pEEPROM_PAUSE_INFO_DATA.ucPauseStatus[ucCh] = TRUE;
    m_ucPauseSequenceDelay[ucCh] = _DELAY_PAUSE_ON;
	}
	else if( param == _RESUME_ON && m_ucPauseStatus[ucCh] )
	{
		m_ucPauseSequence[ucCh] = param;
		m_ucPauseStatus[ucCh] = FALSE;
    //m_ucBlackoutCheckEnable[ucCh] = 1;
    m_ucBlackOutFlag[ucCh] = 0;
    //m_ucBlackOutPauseFlag[ucCh] = 0;

    m_pEEPROM_PAUSE_INFO_DATA.ucIsPauseDataValid[ucCh] = 0;
    m_pEEPROM_PAUSE_INFO_DATA.ucIsPauseDataWritten[ucCh] = 0;
    m_pEEPROM_PAUSE_INFO_DATA.ucPauseStatus[ucCh] = FALSE;
    m_ucPauseSequenceDelay[ucCh] = _DELAY_RESUME_ON;
	}
	else if( param == _RESUME_NEXT_STEP && m_ucPauseStatus[ucCh] )
	{
		m_ucFirstStep[ucCh] = TRUE;
		m_bChannelRunning[ucCh] = TRUE;
		m_ucPreState[ucCh] = !m_ucNowRestChargeDisCharge[ucCh];
		m_bMcControlAllComplete[ucCh] = FALSE;
		m_bStepRunning[ucCh] = FALSE;
		
		m_bContinueNextSequence[ucCh] = 1;
		m_ucPauseStatus[ucCh] = FALSE;

    //m_ucStepIndexNow[ucCh] = m_ucStartStepIndex[ucCh];
    m_pSeqNow[ucCh] = & m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[ucCh][m_ucStepIndexNow[ucCh]];
		m_pSeqNext[ucCh] = & m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[ucCh][(m_ucStepIndexNow[ucCh] + 1) % _MAX_SEQUENCE_STEP];
    m_ucPulseNextState[ucCh] = m_pSeqNext[ucCh]->ucState;
    m_ucPreState[ucCh] = !m_ucNowRestChargeDisCharge[ucCh];

    m_pEEPROM_PAUSE_INFO_DATA.ucIsPauseDataValid[ucCh] = 0;
    m_pEEPROM_PAUSE_INFO_DATA.ucIsPauseDataWritten[ucCh] = 0;
    m_pEEPROM_PAUSE_INFO_DATA.ucPauseStatus[ucCh] = FALSE;
    
    m_ucPauseSequenceDelay[ucCh] = _DELAY_RESUME_ON;
	}		
  else if ( param == _PAUSE_NONE && m_ucPauseStatus[ucCh] )
  {
    m_ucPauseSequence[ucCh] = param;
		m_ucPauseStatus[ucCh] = FALSE;
    
    m_pEEPROM_PAUSE_INFO_DATA.ucIsPauseDataValid[ucCh] = 0;
    m_pEEPROM_PAUSE_INFO_DATA.ucIsPauseDataWritten[ucCh] = 0;
    m_pEEPROM_PAUSE_INFO_DATA.ucPauseStatus[ucCh] = FALSE;
    m_ucPauseSequenceDelay[ucCh] = _DELAY_CLEAR;    
  }
}
#endif

