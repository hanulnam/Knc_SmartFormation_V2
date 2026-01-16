/*
 * KpuSystemConfig.c
 *
 * Created: 2021-11-26 AM 00:05:00
 *  Author: 4N Corporation, bgyu
 */ 


#include "KnC_IncludeFile.h"

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// New Function
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void KpuPinDefineAndInitialize( void )		// Pin Define은 윤차장이 좀 수고 해줘요~~ 20220223 bgyu // 20220305 Pin define 확인 smy
{
	int iPin;
	
	uint32_t uiPinConfigration[][2] =
	{
		#define		GPIO_NORMAL_DRIVE_OUTPUT	(GPIO_DIR_OUTPUT | GPIO_DRIVE_MIN )
		#define		GPIO_MAX_DRIVE_OUTPUT		(GPIO_DIR_OUTPUT | GPIO_DRIVE_MAX )
		#define		GPIO_NORMAL_INPUT			(GPIO_DIR_INPUT  | GPIO_DRIVE_MIN )
		

		{ AVR32_PIN_PD04, GPIO_MAX_DRIVE_OUTPUT | GPIO_INIT_HIGH,	}, // PD04 // Ch1 Debug Gate1					//n
		{ AVR32_PIN_PD16, GPIO_MAX_DRIVE_OUTPUT | GPIO_INIT_HIGH,	}, // PD16 // Ch1 Debug Gate2					//n
		{ AVR32_PIN_PD19, GPIO_MAX_DRIVE_OUTPUT | GPIO_INIT_HIGH,	}, // PD19 // Ch1 Debug Gate3					//n
		{ AVR32_PIN_PD22, GPIO_MAX_DRIVE_OUTPUT | GPIO_INIT_HIGH,	}, // PD22 // Ch1 Debug Gate4					//n
						
		{ AVR32_PIN_PD23, GPIO_MAX_DRIVE_OUTPUT | GPIO_INIT_HIGH,	}, // PD23 // Ch1 Debug Gate5					//n
		{ AVR32_PIN_PC10, GPIO_MAX_DRIVE_OUTPUT | GPIO_INIT_HIGH,	}, // PC10 // Ch1 Debug Gate6					//n
		{ AVR32_PIN_PC12, GPIO_MAX_DRIVE_OUTPUT | GPIO_INIT_HIGH,	}, // PC12 // Ch1 Debug Gate Select				//n			

		{ AVR32_PIN_PB18, GPIO_NORMAL_INPUT,						}, // ch id 0
		{ AVR32_PIN_PB19, GPIO_NORMAL_INPUT,						}, // ch id 1
		{ AVR32_PIN_PB20, GPIO_NORMAL_INPUT,						}, // ch id 2
		{ AVR32_PIN_PB21, GPIO_NORMAL_INPUT,						}, // ch id 3
		{ AVR32_PIN_PB25, GPIO_NORMAL_INPUT,						}, // ch id 4			
		{ AVR32_PIN_PB26, GPIO_NORMAL_INPUT,						}, // ch id 5									

		{ AVR32_PIN_PB31, GPIO_NORMAL_DRIVE_OUTPUT | GPIO_INIT_HIGH,}, // PB31 // Ch1MC_ON			//n
		{ AVR32_PIN_PB03, GPIO_NORMAL_DRIVE_OUTPUT | GPIO_INIT_HIGH,}, // PB03 // Ch2MC_ON			//n
		{ AVR32_PIN_PC02, GPIO_NORMAL_DRIVE_OUTPUT | GPIO_INIT_HIGH,}, // PC02 // Ch3MC_ON			//n
		{ AVR32_PIN_PC03, GPIO_NORMAL_DRIVE_OUTPUT | GPIO_INIT_HIGH,}, // PA03 // Ch4MC_On			//n		/	

		{ AVR32_PIN_PB27, GPIO_MAX_DRIVE_OUTPUT| GPIO_INIT_HIGH,	}, // PB27 // Ch1 ChargePullUp				//n
		{ AVR32_PIN_PB28, GPIO_MAX_DRIVE_OUTPUT| GPIO_INIT_HIGH,	}, // PB28 // Ch2 ChargePullUp				//n
		{ AVR32_PIN_PB29, GPIO_MAX_DRIVE_OUTPUT| GPIO_INIT_HIGH,	}, // PB29 // Ch3 ChargePullUp				//n
		{ AVR32_PIN_PC00, GPIO_MAX_DRIVE_OUTPUT| GPIO_INIT_HIGH,	}, // PC00 // Ch4 ChargePullUp				//n			

		{ AVR32_PIN_PA11, GPIO_NORMAL_DRIVE_OUTPUT,					}, // PA11 // Ch1 DisCharge0Charge1			//n
		{ AVR32_PIN_PA14, GPIO_NORMAL_DRIVE_OUTPUT,					}, // PA14 // Ch2 DisCharge0Charge1			//n
		{ AVR32_PIN_PA19, GPIO_NORMAL_DRIVE_OUTPUT,					}, // PA19 // Ch3 DisCharge0Charge1			//n
		{ AVR32_PIN_PA22, GPIO_NORMAL_DRIVE_OUTPUT,					}, // PA22 // Ch4 DisCharge0Charge1			//n

		{ AVR32_PIN_PA12, GPIO_NORMAL_DRIVE_OUTPUT | GPIO_INIT_LOW,	}, // PA12 // Ch1 DisChargeEnalble				//n
		{ AVR32_PIN_PA15, GPIO_NORMAL_DRIVE_OUTPUT | GPIO_INIT_LOW,	}, // PA15 // Ch2 DisChargeEnalble				//n
		{ AVR32_PIN_PA20, GPIO_NORMAL_DRIVE_OUTPUT | GPIO_INIT_LOW,	}, // PA20 // Ch3 DisChargeEnalble				//n
		{ AVR32_PIN_PA23, GPIO_NORMAL_DRIVE_OUTPUT | GPIO_INIT_LOW,	}, // PA23 // Ch4 DisChargeEnalble				//n

		{ AVR32_PIN_PA13, GPIO_NORMAL_DRIVE_OUTPUT | GPIO_INIT_LOW,	}, // PA13 // Ch1 ChargeEnalble				//n
		{ AVR32_PIN_PA16, GPIO_NORMAL_DRIVE_OUTPUT | GPIO_INIT_LOW,	}, // PA16 // Ch2 ChargeEnalble				//n
		{ AVR32_PIN_PA21, GPIO_NORMAL_DRIVE_OUTPUT | GPIO_INIT_LOW,	}, // PA21 // Ch3 ChargeEnalble				//n
		{ AVR32_PIN_PA24, GPIO_NORMAL_DRIVE_OUTPUT | GPIO_INIT_LOW,	}, // PA24 // Ch4 ChargeEnalble				//n

		{ AVR32_PIN_PA25, GPIO_NORMAL_INPUT,						}, // PA25 // FAN Error
		{ AVR32_PIN_PC01, GPIO_NORMAL_DRIVE_OUTPUT | GPIO_INIT_HIGH,}, // PC01 // FAN_OFF High

		{ AVR32_PIN_PB07, GPIO_NORMAL_DRIVE_OUTPUT, 				}, // PB07 // DAC SYNC
		{ AVR32_PIN_PB06, GPIO_NORMAL_DRIVE_OUTPUT, 				}, // PB06 // DAC SCLK
		{ AVR32_PIN_PB04, GPIO_NORMAL_DRIVE_OUTPUT, 				}, // PB04 // DAC SDI
		{ AVR32_PIN_PB05, GPIO_NORMAL_INPUT, 						}, // PB05 // DAC SDO
		{ AVR32_PIN_PB08, GPIO_NORMAL_DRIVE_OUTPUT | GPIO_INIT_HIGH,}, // PB08 // DAC LDAC
		{ AVR32_PIN_PC04, GPIO_NORMAL_DRIVE_OUTPUT, 				}, // PC04 // DAC nReset

		{ AVR32_PIN_PC07, GPIO_NORMAL_DRIVE_OUTPUT, 				}, // PC07 // DAC2 SCLK
		{ AVR32_PIN_PC06, GPIO_NORMAL_DRIVE_OUTPUT, 				}, // PC06 // DAC2 SDI
		{ AVR32_PIN_PC08, GPIO_NORMAL_INPUT, 						}, // PC08 // DAC2 SDO
		{ AVR32_PIN_PB22, GPIO_NORMAL_DRIVE_OUTPUT| GPIO_INIT_HIGH,	}, // PB22 // DAC2 CS_
		{ AVR32_PIN_PB23, GPIO_NORMAL_DRIVE_OUTPUT | GPIO_INIT_LOW,	}, // PB23 // DAC2 LAT_

		{ AVR32_PIN_PA10, GPIO_NORMAL_DRIVE_OUTPUT | GPIO_INIT_HIGH,},// PA10 // ADC Convert Start							//n		
		{ AVR32_PIN_PD30, GPIO_NORMAL_DRIVE_OUTPUT | GPIO_INIT_LOW,	},// PD30 // Adc Reset									//n
		{ AVR32_PIN_PD28, GPIO_NORMAL_DRIVE_OUTPUT,					},// PD28 // Adc Busy									//n	
		{ AVR32_PIN_PB12, GPIO_MAX_DRIVE_OUTPUT | GPIO_INIT_HIGH,	},// PB12 // ADC CLK									//n
		{ AVR32_PIN_PB13, GPIO_NORMAL_DRIVE_OUTPUT,					},// PB13 // CS_										//n
		{ AVR32_PIN_PB15, GPIO_NORMAL_INPUT,						},// PB15 // ADC First Data								//n
		{ AVR32_PIN_PB11, GPIO_NORMAL_INPUT,						},// PB11 // Adc Data Out A								//n
		{ AVR32_PIN_PB09, GPIO_NORMAL_DRIVE_OUTPUT,					},// PB09 // Adc Data Out B// Data Out Select			//n
			
		{ AVR32_PIN_PA04, GPIO_NORMAL_INPUT,						}, // PA04 // BoardAirTemp						//n
		{ AVR32_PIN_PA05, GPIO_NORMAL_INPUT, 						}, // PA05 // DAC Temperature					//n
		{ AVR32_PIN_PA06, GPIO_NORMAL_INPUT,						}, // PA06 // F.E.T_HTemp						//n
		{ AVR32_PIN_PA07, GPIO_NORMAL_DRIVE_OUTPUT,					}, // PA07 // HTemp_A0							//n
		{ AVR32_PIN_PA08, GPIO_NORMAL_DRIVE_OUTPUT,					}, // PA08 // Htemp_A1							//n
		{ AVR32_PIN_PA09, GPIO_NORMAL_DRIVE_OUTPUT,					}, // PA09 // HTemp_A2							//n


		{ AVR32_PIN_PA29, GPIO_NORMAL_DRIVE_OUTPUT,					}, // PA29 // Debug0							//n
		{ AVR32_PIN_PA26, GPIO_NORMAL_DRIVE_OUTPUT,					}, // PA26 // Debug1							//n
		{ AVR32_PIN_PA27, GPIO_NORMAL_DRIVE_OUTPUT,					}, // PA27 // Debug2							//n

		{ AVR32_PIN_PC09, GPIO_NORMAL_DRIVE_OUTPUT,					}, // PC09 // Ch1 AdcVSelect(VOLTAGE2_MODE)		//n
		{ AVR32_PIN_PD25, GPIO_NORMAL_DRIVE_OUTPUT,					}, // PD25 // Ch2 AdcVSelect(VOLTAGE2_MODE)		//n
		{ AVR32_PIN_PD26, GPIO_NORMAL_DRIVE_OUTPUT,					}, // PD26 // Ch3 AdcVSelect(VOLTAGE2_MODE)		//n			
		{ AVR32_PIN_PA28, GPIO_NORMAL_DRIVE_OUTPUT,					}, // PA28 // Ch4 AdcVSelect(VOLTAGE2_MODE)		//n						
			
		{ AVR32_PIN_PB00, GPIO_NORMAL_INPUT,						}, // PB00 // Clock Input 12Mhz					//n
		{ AVR32_PIN_PB24, GPIO_NORMAL_INPUT,						}, // PB24 // Clock Input 12Mhz					//n
		{ AVR32_PIN_PB30, GPIO_NORMAL_INPUT,						}, // PB30 // Clock Input 12Mhz					//n
		{ AVR32_PIN_PB01, GPIO_NORMAL_DRIVE_OUTPUT,					}, // PB01 // Alarm Led Red						//n
		{ AVR32_PIN_PB02, GPIO_NORMAL_DRIVE_OUTPUT,					}, // PB02 // Avr Run Led						//n
						
		{ AVR32_PIN_PB14, GPIO_NORMAL_DRIVE_OUTPUT,					}, // PB14 // Rs485Rts							//n
		{ AVR32_PIN_PB16, GPIO_NORMAL_INPUT, 						}, // PB16 // Rs485Rxd							//n
		{ AVR32_PIN_PB17, GPIO_NORMAL_DRIVE_OUTPUT,					}, // PB17 // Rs485Txd							//n
		
		{ AVR32_PIN_PC05, GPIO_NORMAL_DRIVE_OUTPUT,					}, // PB04 // EEPROM WP
		{ AVR32_PIN_PD29, GPIO_NORMAL_DRIVE_OUTPUT,					}, // PB05 // EEPROM SCL
		{ AVR32_PIN_PC13, GPIO_NORMAL_DRIVE_OUTPUT,					}, // PB06 // EEPROM SDA
		{ AVR32_PIN_PC11, GPIO_NORMAL_DRIVE_OUTPUT,					}, // PC11 // I_TR_Soft0_Fast1	//220517smy
	};
	
	int iMax;
	
	iMax = sizeof(uiPinConfigration)/sizeof(uint32_t)/2;
	for( iPin=0; iPin<iMax; iPin++)
	{
		if ( uiPinConfigration[iPin][0]>0xffff )
			break;
		gpio_configure_pin( uiPinConfigration[iPin][0], uiPinConfigration[iPin][1] );
	}
	

	return;
}

void KpuFrequenceClockSetUP( void )
{
	sysclk_init();												//pll0를 fcpu로 선택.. 66Mhz.
	delay_init(sysclk_get_cpu_hz());							//@@@ cpu cycle count를 이용한 delay함수 초기화
}

void KpuSdram16M16BitInitialize( void )
{
	sdram_enter_self_refresh();
	sdramc_init( m_iPbcHz );
}

void PcCommUsartSetup( void )
{
	static const gpio_map_t USART_GPIO_MAP =
	{
		{ PC_COMM_USART_RX_PIN,		PC_COMM_USART_RX_FUNCTION },
		{ PC_COMM_USART_TX_PIN,		PC_COMM_USART_TX_FUNCTION },
		{ PC_COMM_USART_RTS_PIN,	PC_COMM_USART_RTS_FUNCTION },
	};

	static const usart_options_t USART_OPTIONS =
	{
		.baudrate     = PC_COMM_USART_BAUDRATE,
		.charlength   = 8,
		.paritytype   = USART_EVEN_PARITY,
		.stopbits     = USART_1_STOPBIT,
		.channelmode  = USART_NORMAL_CHMODE
	};

	PC_COMM_USART->cr = AVR32_USART_CR_RTSEN_MASK;// First INIT
	gpio_enable_module(USART_GPIO_MAP, sizeof(USART_GPIO_MAP) / sizeof(USART_GPIO_MAP[0]));
	PC_COMM_USART->cr = AVR32_USART_CR_RTSEN_MASK;// First INIT
	usart_init_rs485(PC_COMM_USART, &USART_OPTIONS, sysclk_get_cpu_hz());
	PC_COMM_USART->cr = AVR32_USART_CR_RTSEN_MASK;// First INIT
	PC_COMM_USART->ttgr = 1;
	
	PDCA_PcCommSetup();
}

void KpuDacSPIUsartSetup( void )
{
	static const gpio_map_t USART_SPI_GPIO_MAP =
	{
		{ DAC_SPI_SCK_PIN,		DAC_SPI_SCK_FUNCTION  },
		{ DAC_SPI_MISO_PIN,		DAC_SPI_MISO_FUNCTION },
		{ DAC_SPI_MOSI_PIN,		DAC_SPI_MOSI_FUNCTION },
		{ DAC_SPI_NPCS0_PIN,	DAC_SPI_NPCS0_FUNCTION }
	};

	gpio_enable_module(USART_SPI_GPIO_MAP, sizeof(USART_SPI_GPIO_MAP) / sizeof(USART_SPI_GPIO_MAP[0]));		// Assign GPIO to SPI.
	
	spi_master_init( DAC_SPI );
	spi_master_setup_device( DAC_SPI, (void*)&DAC_SPI_DEVICE, SPI_MODE_1, DAC_SPI_BAUDRATE, 0 );
	
	spi_enable( DAC_SPI );
}

void KpuAdcSPIUsartSetup( void )
{
	static const gpio_map_t USART_ADC_SPI_GPIO_MAP =
	{
		{ ADC_SPI_SCK_PIN,		ADC_SPI_SCK_FUNCTION  },
		{ ADC_SPI_MISO_PIN,		ADC_SPI_MISO_FUNCTION },
		{ ADC_SPI_MOSI_PIN,		ADC_SPI_MOSI_FUNCTION },
		{ ADC_SPI_NPCS0_PIN,	ADC_SPI_NPCS0_FUNCTION }
	};

	gpio_enable_module(USART_ADC_SPI_GPIO_MAP, sizeof(USART_ADC_SPI_GPIO_MAP) / sizeof(USART_ADC_SPI_GPIO_MAP[0]));		// Assign GPIO to SPI.
	
	spi_master_init( ADC_SPI );
	spi_master_setup_device( ADC_SPI, (void*)&ADC_SPI_DEVICE, SPI_MODE_0, ADC_SPI_BAUDRATE, 0 );
	spi_enable( ADC_SPI );
}

void PDCA_PcCommSetup( void )
{
	AVR32_HMATRIXB.mcfg[AVR32_HMATRIXB_MASTER_CPU_INSN] = 0x1;

	PDCA_SetupForUsart( PC_COMM_RX_PDCA_CHANNEL_USART, AVR32_PDCA_PID_USART1_RX, m_ucPcCommPDCA_RxBuffer, sizeof(m_ucPcCommPDCA_RxBuffer) );
	PDCA_SetupForUsart( PC_COMM_TX_PDCA_CHANNEL_USART, AVR32_PDCA_PID_USART1_TX, m_ucPcCommPDCA_TxBuffer, 0 );
}

void PDCA_SetupForUsart( int iPdcaChannel, int iPdcaPid, unsigned char* pBuffer, int iBufferSize  )
{
	static pdca_channel_options_t PDCA_OPTIONS =
	{
		.r_addr = NULL,										// next memory address
		.r_size = 0,										// next transfer counter
		.transfer_size = PDCA_TRANSFER_SIZE_BYTE			// select size of the transfer
	};
	PDCA_OPTIONS.addr = (void *)pBuffer;
	PDCA_OPTIONS.pid = iPdcaPid;
	PDCA_OPTIONS.size = iBufferSize;

	pdca_init_channel( iPdcaChannel, &PDCA_OPTIONS );		// init PDCA channel with options.

	pdca_enable_interrupt_transfer_complete(iPdcaChannel);
	pdca_enable(iPdcaChannel);
}

void PDCA_ReLoadForUsart( int iPdcaChannel, unsigned char* pBuffer, unsigned long iBufferSize  )
{
	AVR32_PDCA.channel[iPdcaChannel].mar = (unsigned long)pBuffer;
	AVR32_PDCA.channel[iPdcaChannel].marr = 0;
	AVR32_PDCA.channel[iPdcaChannel].tcrr = 0;
	AVR32_PDCA.channel[iPdcaChannel].mr = PDCA_TRANSFER_SIZE_BYTE<<AVR32_PDCA_SIZE_OFFSET;
	AVR32_PDCA.channel[iPdcaChannel].cr = AVR32_PDCA_ECLR_MASK;
	AVR32_PDCA.channel[iPdcaChannel].tcr = iBufferSize;
	//AVR32_PDCA.channel[iPdcaChannel].ier = AVR32_PDCA_IER_TRC_MASK;;
	AVR32_PDCA.channel[iPdcaChannel].ier = AVR32_PDCA_TRC_MASK;
	AVR32_PDCA.channel[iPdcaChannel].isr;
}

__attribute__((__interrupt__)) static void PDCA_PcCommRxInterrupHandler(void)
{
	int iIsr;
	iIsr = PC_COMM_RX_PDCA.isr;

	if ( iIsr&AVR32_PDCA_TERR_MASK )
	{
		m_uiSystemInformation[_SystemErrorFlag] |= 1<<SystemErrorFlagPcCommunicationPdcaRx;
	}
	else
	{
		m_uiSystemInformation[_SystemErrorFlag] |= 1<<SystemErrorFlagPcdaPcCommunicationRxBufferOver;
	}
	pdca_reload_channel( PC_COMM_RX_PDCA_CHANNEL_USART,	(void *)m_ucPcCommPDCA_RxBuffer, sizeof( m_ucPcCommPDCA_RxBuffer ) );
}

__attribute__((__interrupt__)) static void PDCA_PcCommTxInterrupHandler(void)
{
	int iIsr;
	iIsr = PC_COMM_TX_PDCA.isr;
	if ( iIsr&AVR32_PDCA_TRC_MASK )
	{
		PC_COMM_TX_PDCA.idr = AVR32_PDCA_TRC_MASK;
	}
	if ( iIsr&AVR32_PDCA_TERR_MASK )
	{
		m_uiSystemInformation[_SystemErrorFlag] |= 1<<SystemErrorFlagPcCommunicationPdcaTx;
		pdca_reload_channel( PC_COMM_TX_PDCA_CHANNEL_USART, (void *)m_ucPcCommPDCA_TxBuffer, 0 );
	}
}

__attribute__((__interrupt__)) static void PcCommInterruptHandler(void)
{
	return;
	PC_COMM_USART->cr = AVR32_USART_CR_RSTSTA_MASK;

	//// Print the received character to USART. This is a simple echo.
	while (usart_get_echo_line(PC_COMM_USART) == USART_FAILURE);
}

void LedStatusSet( void )
{

	if ( !m_bLedRunStatus )
		return;
	
	GetFetTempNVolt2();

	/*
	U8 i;
	U8 u8ChannelStaus[_MAX_CHANNEL+1];	
	u8ChannelStaus[_MAX_CHANNEL] = _STATUS_STOP;

	for( i=0; i<_MAX_CHANNEL; i++ )
	{
		if ( !m_bChannelRunning[i] )
			u8ChannelStaus[i] = _STATUS_STOP;
		else
		if ( m_bChannelRunning[i] && m_bStepRunning[i] )
			u8ChannelStaus[_MAX_CHANNEL] = u8ChannelStaus[i] = _STATUS_RUN;
		else
		if ( m_bChannelRunning[i] && !m_bStepRunning[i] )
			u8ChannelStaus[i] = _STATUS_STEP_END;
	}

	// 20221223 DJL Soft/Boost Indigator용 임시
	//switch( u8ChannelStaus[_MAX_CHANNEL] )
	//{
		//case _STATUS_RUN:	_LED_RUN_TOGGLE;	break;
		//default:			_LED_RUN_OFF;		break;
	//}
	switch( u8ChannelStaus[0] )
	{
		case _STATUS_RUN:		_LED_CH1_TOGGLE;	break;
		case _STATUS_STEP_END:	_LED_CH1_OFF;		break;
		default:				_LED_CH1_ON;		break;
	}
	switch( u8ChannelStaus[1] )
	{
		case _STATUS_RUN:		_LED_CH2_TOGGLE;	break;
		case _STATUS_STEP_END:	_LED_CH2_OFF;		break;
		default:				_LED_CH2_ON;		break;
	}
	switch( u8ChannelStaus[2] )
	{
		case _STATUS_RUN:		_LED_CH3_TOGGLE;	break;
		case _STATUS_STEP_END:	_LED_CH3_OFF;		break;
		default:				_LED_CH3_ON;		break;
	}
	switch( u8ChannelStaus[3] )
	{
		case _STATUS_RUN:		_LED_CH4_TOGGLE;	break;
		case _STATUS_STEP_END:	_LED_CH4_OFF;		break;
		default:				_LED_CH4_ON;		break;
	}
	*/
	EtcAlarmSet();
	m_bLedRunStatus = FALSE;
}

void EtcAlarmSet( void )
{
	unsigned char ucCh;
	unsigned char ucFanCheck = FALSE;

	if ( m_bFanOff )
	{
		if ( m_uiFanTime )
		{
			if ( --m_uiFanTime == 0 )
			{
				m_bFanOff = FALSE;
				PDC_HIGH = _PORT_C_FAN_OFF_H;
				m_u16SafeErrorNumbers[0][RTY_FAN] = 0;
			}
		}
	}
	else
	{
		for(ucCh=0; ucCh<_MAX_CHANNEL; ucCh++)
		{
			if(m_bChannelRunning[ucCh] == FALSE) 
				continue;
				
			if(m_pSeqNow[ucCh]->ucState == _STATE_REST) 
				continue;
				
			if(m_uiStepTimeNow[ucCh] < 1000*10 || m_uiStepTimeNow[ucCh] > 0xfffffff0)
				continue;
	
			ucFanCheck = TRUE;
			break;
		}
		
		if (ucFanCheck == TRUE && !gpio_get_pin_value(_FAN_ERROR_INPUT_&0xFFF) )
		{
			if ( ++m_u16SafeErrorNumbers[0][RTY_FAN] >= 2*10 )			// 10초
				m_bFanError = TRUE;
		}
		else
			m_u16SafeErrorNumbers[0][RTY_FAN] = 0;
	}
	//if ( GetAirTemperature()>60.0f )
	#ifdef MODIFY_ALARM_CONDITION
	if ( GetAirTemperature() > m_pProtectionType[0].fAirTemp )
	#else
	if ( GetAirTemperature()>m_fAirTemp )	//2025.04.25 Alarm Test 용
	#endif
	{
		if ( ++m_u16SafeErrorNumbers[0][RTY_AIR_TEMP]>=2*2 )
			m_bAirTemperaturError = TRUE;
	}
	else
		m_u16SafeErrorNumbers[0][RTY_AIR_TEMP] = 0;
	

	//LJK 2024.07.16 채널별 Heat Sink 적용
	//LJK 2025.05.21 상시에서 Alarm Mask로 적용(bgyu 요청사항. LGES_LV9.8A)
	//for(ucCh=0; ucCh<_MAX_CHANNEL; ucCh++)
	//{
	//	//if ( GetHeatSinkTemperaturError(ucCh)>90.0f ) //2025.01.13 80도 -> 100도변경 (Req 유부사장님, 박희규 부장님)
	//	if ( GetHeatSinkTemperaturError(ucCh)> m_fHeatSinkTemp[ucCh] ) //2025.04.25 Alarm Test 용
	//	{
	//		if ( ++m_u16SafeErrorNumbers[ucCh][RTY_HEAT_SINK_TEMP]>=2*2 )
	//			m_bHeatSinkTemperaturError[ucCh] = TRUE;
	//	}
	//	else
	//		m_u16SafeErrorNumbers[ucCh][RTY_HEAT_SINK_TEMP] = 0;
	//}
}

float GetAirTemperature( void )
{
	return (m_fInternalADCAirTempVoltage - 0.50f) / 0.010f;	////10mV/C
	//return 25.0f;//완전임시
}

float GetHeatSinkTemperaturError( unsigned char ucCh )
{
	//FET 0~3 
	//LM35D 센서위치 확인후 배열사용 2024.07.11
	return m_fInternalADCReadVoltage[ucCh] / 0.020f; //LM35DT 20mV/C
	//return 25.0f;//완전임시
}

__attribute__((__interrupt__)) static void CaptainTimerCounterInterrupHandler( void )
{
	static U8 u8First = 1;
	U8 u8R;
	static U8 bLDac = FALSE;
	U16 usR;
	CAPTAIN_TIMER_COUNT_ADDRESS->channel[CAPTAIN_TIMER_COUNT_CHANNEL].sr;
	CAPTAIN_TP_H;
		
	tc_write_rc( ADC_READ_CYCLE_TIMER_COUNT_ADDRESS, ADC_READ_CYCLE_TIMER_COUNT_CHANNEL, 1 );
	tc_start( ADC_READ_CYCLE_TIMER_COUNT_ADDRESS, ADC_READ_CYCLE_TIMER_COUNT_CHANNEL );
	ADC_READ_CYCLE_TIMER_COUNT_ADDRESS->channel[ADC_READ_CYCLE_TIMER_COUNT_CHANNEL].ccr = AVR32_TC_SWTRG_MASK | AVR32_TC_CLKEN_MASK;	// Adc First Start
	
	//LJK 2025.05.29
	if ( m_ucNextCurrentDelay[0] )
	{
		m_ucNextCurrentDelay[0]--;
		if ( m_ucNextCurrentDelay[0]==0 )
		{
			SetDacWriteData( _DAC_CH1_CC, m_s16DacCurrentNext[0] );
			if ( m_pSeqNow[0]->ucState==_STATE_DISCHARGE ) _CH1_DISCHARGE_ENABLE_ON;
			else if ( m_pSeqNow[0]->ucState==_STATE_CHARGE )  _CH1_CHARGE_ENABLE_ON;
			bLDac = TRUE;
		}
	}
	if ( m_ucNextCurrentDelay[1] )
	{
		m_ucNextCurrentDelay[1]--;
		if ( m_ucNextCurrentDelay[1]==0 )
		{
			SetDacWriteData( _DAC_CH2_CC, m_s16DacCurrentNext[1] );
			if ( m_pSeqNow[1]->ucState==_STATE_DISCHARGE ) _CH2_DISCHARGE_ENABLE_ON;
			else if ( m_pSeqNow[1]->ucState==_STATE_CHARGE )  _CH2_CHARGE_ENABLE_ON;
			bLDac = TRUE;
		}
	}
	if ( m_ucNextCurrentDelay[2] )
	{
		m_ucNextCurrentDelay[2]--;
		if ( m_ucNextCurrentDelay[2]==0 )
		{
			SetDacWriteData( _DAC_CH3_CC, m_s16DacCurrentNext[2] );
			if ( m_pSeqNow[2]->ucState==_STATE_DISCHARGE ) _CH3_DISCHARGE_ENABLE_ON;
			else if ( m_pSeqNow[2]->ucState==_STATE_CHARGE )  _CH3_CHARGE_ENABLE_ON;
			bLDac = TRUE;
		}
	}
	if ( m_ucNextCurrentDelay[3] )
	{
		m_ucNextCurrentDelay[3]--;
		if ( m_ucNextCurrentDelay[3]==0 )
		{
			SetDacWriteData( _DAC_CH4_CC, m_s16DacCurrentNext[3] );
			if ( m_pSeqNow[3]->ucState==_STATE_DISCHARGE ) _CH4_DISCHARGE_ENABLE_ON;
			else if ( m_pSeqNow[3]->ucState==_STATE_CHARGE )  _CH4_CHARGE_ENABLE_ON;
			bLDac = TRUE;
		}
	}	

	if ( bLDac )
	{
		bLDac = FALSE;
		_LDAC_ENABLE;
	}
			
	{
		if ( m_bRunSequence[0] == TRUE && (m_ui1msCounter & 0x03) == 0 )
		{
			m_ucNowPulseMode[0] = _PULSE_MODE_NONE;
			m_bStepStarted[0] = FALSE;
			m_bNewPulseLoad[0] = FALSE;
			m_bRunSequence[0] = FALSE;
			m_usReadyToStartSequenceCounter[0] = 0;
			m_bNewStepSetting[0] = m_bReadyToStartSequence[0] = TRUE;
			m_ucStepIndexNow[0] = m_ucStartStepIndex[0];
			m_pSeqNow[0] = & m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[0][m_ucStepIndexNow[0]];
			m_pSeqNext[0] = & m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[0][(m_ucStepIndexNow[0] + 1) % _MAX_SEQUENCE_STEP];
			m_ucPulseNextState[0] = m_pSeqNext[0]->ucState;
			m_ucFirstStep[0] = TRUE;
			m_bChannelRunning[0] = TRUE;
			m_ucPreState[0] = !m_ucNowRestChargeDisCharge[0];
			if ( m_pSeqNow[0]->ucState==_STATE_DISCHARGE ||
				m_pSeqNow[0]->ucMode==_MODE_2POINT_PULSE_DISCHARGE || m_pSeqNow[0]->ucMode==_MODE_10POINT_PULSE_DISCHARGE ||  m_pSeqNow[0]->ucMode==_MODE_DISCHARGE_CONTACT )	// bgyu 20250616
			{
				_CH1_CHARGE_ENABLE_OFF;
				_CH1_DISCHARGE_ENABLE_OFF;
				_CH1_MODE_DISCHARGE;
				SetDacWriteData( _DAC_CH1_CC, 0x8000 );	// bgyu 20250528
				SetDacWriteData( _DAC_CH1_CV, 0x0000 );
				_LDAC_ENABLE;
			}
			else
			if ( m_pSeqNow[0]->ucState==_STATE_CHARGE ||
				m_pSeqNow[0]->ucMode==_MODE_2POINT_PULSE_CHARGE || m_pSeqNow[0]->ucMode==_MODE_10POINT_PULSE_CHARGE ||  m_pSeqNow[0]->ucMode==_MODE_CHARGE_CONTACT )	// bgyu 20250616
			{
				_CH1_CHARGE_ENABLE_OFF;
				_CH1_DISCHARGE_ENABLE_OFF;
				_CH1_MODE_CHARGE;
				SetDacWriteData( _DAC_CH1_CC, 0x7fff );	// bgyu 20250528
				SetDacWriteData( _DAC_CH1_CV, 0x7fff );
				_LDAC_ENABLE;
			}
			NowModeStateSet(0);
			
			// 20221117 djl 추가.
			m_bMcControlAllComplete[0] = FALSE;
			m_bStepRunning[0] = FALSE;
//			_CH1_CHARGE_ENABLE_OFF;				// bgyu 20250528 삭제
		}
		
		if ( m_bContinueNextSequence[0] )
		{
			m_bContinueNextSequence[0]--;
			if ( m_bContinueNextSequence[0]==0 )
			{
				u8R = FormationEndCheck( 0 );
				if ( !u8R )
				{
					m_bStepStarted[0] = FALSE;
					m_bNewPulseLoad[0] = FALSE;
					m_bNewStepSetting[0] = TRUE;
					m_usReadyToStartSequenceCounter[0] = 0;
					m_bReadyToStartSequence[0] = TRUE;
					if ( m_pSeqNow[0]->usLoopCountNow < m_pSeqNow[0]->usLoopCountMax )
					{
						ClearLoopNowCount(0, m_pSeqNow[0]->ucLoopIndex, m_ucStepIndexNow[0]);
						++m_pSeqNow[0]->usLoopCountNow;
						m_ucStepIndexNow[0] = m_pSeqNow[0]->ucLoopIndex%_MAX_SEQUENCE_STEP;
						m_pSeqNow[0] = & m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[0][m_ucStepIndexNow[0]];
					}
					else
					{
						m_ucStepIndexNow[0] = (++m_ucStepIndexNow[0])%_MAX_SEQUENCE_STEP;
						m_pSeqNow[0] = & m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[0][m_ucStepIndexNow[0]];
					}
					m_pSeqNext[0] = & m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[0][(m_ucStepIndexNow[0]+1)%_MAX_SEQUENCE_STEP];
				}
			}
		}
	
		
		if ( m_ucStopSequence[0] )
		{
			m_ucStopSequence[0] = 0;
			if ( m_bChannelRunning[0] )
			{
				StepEnd( 0, _STEP_END_BY_USER );
				FormationEnd( 0 );
				m_bStepRunning[0] = FALSE;	//LJK 2025.03.18 StepEnd, Record 2번남는 버그 수정
			}
		}
		//LJK 2023.03.21
		if( m_ucPauseSequence[0] == _PAUSE_ON )
		{
			m_ucStopSequence[0] = m_ucPauseSequence[0] = _PAUSE_NONE;
			if ( m_bChannelRunning[0] )
			{
				StepEnd( 0, _STEP_END_PAUSE );
				FormationEnd( 0 );
			}
		}
		else
		if( m_ucPauseSequence[0] == _RESUME_ON )
		{
			m_ucPauseSequence[0] = FALSE;
			m_ucResumeStatus[0] = TRUE;
			
			m_bRunSequence[0] = TRUE;
			m_ucStartStepIndex[0] = m_ucStepIndexNow[0];
			m_uiPauseTimeNow[0] = m_uiStepTimeNow[0];
		}//						
	}
	
	{
		if ( m_bRunSequence[1] == TRUE && (m_ui1msCounter & 0x03) == 1 )
		{
			m_ucNowPulseMode[1] = _PULSE_MODE_NONE;
			m_bStepStarted[1] = FALSE;
			m_bNewPulseLoad[1] = FALSE;
			m_bRunSequence[1] = FALSE;
			m_usReadyToStartSequenceCounter[1] = 0;
			m_bNewStepSetting[1] = m_bReadyToStartSequence[1] = TRUE;
			m_ucStepIndexNow[1] = m_ucStartStepIndex[1];
			m_pSeqNow[1] = & m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[1][m_ucStepIndexNow[1]];
			m_pSeqNext[1] = & m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[1][(m_ucStepIndexNow[1] + 1) % _MAX_SEQUENCE_STEP];
			m_ucPulseNextState[1] = m_pSeqNext[1]->ucState;
			m_ucFirstStep[1] = TRUE;
			m_bChannelRunning[1] = TRUE;
			m_ucPreState[1] = !m_ucNowRestChargeDisCharge[1];			
			if ( m_pSeqNow[1]->ucState==_STATE_DISCHARGE ||
				m_pSeqNow[1]->ucMode==_MODE_2POINT_PULSE_DISCHARGE || m_pSeqNow[1]->ucMode==_MODE_10POINT_PULSE_DISCHARGE ||  m_pSeqNow[1]->ucMode==_MODE_DISCHARGE_CONTACT )	// bgyu 20250616
			{
				_CH2_CHARGE_ENABLE_OFF;
				_CH2_DISCHARGE_ENABLE_OFF;
				_CH2_MODE_DISCHARGE;
				SetDacWriteData( _DAC_CH2_CC, 0x8000 );	// bgyu 20250528
				SetDacWriteData( _DAC_CH2_CV, 0x0000 );
				_LDAC_ENABLE;
			}
			else
			if ( m_pSeqNow[1]->ucState==_STATE_CHARGE ||
				m_pSeqNow[1]->ucMode==_MODE_2POINT_PULSE_CHARGE || m_pSeqNow[1]->ucMode==_MODE_10POINT_PULSE_CHARGE ||  m_pSeqNow[1]->ucMode==_MODE_CHARGE_CONTACT )	// bgyu 20250616
			{
				_CH2_CHARGE_ENABLE_OFF;
				_CH2_DISCHARGE_ENABLE_OFF;
				_CH2_MODE_CHARGE;
				SetDacWriteData( _DAC_CH2_CC, 0x7fff );	// bgyu 20250528
				SetDacWriteData( _DAC_CH2_CV, 0x7fff );
				_LDAC_ENABLE;
			}			
			NowModeStateSet(1);
			
			// 20221117 djl 추가.
			m_bMcControlAllComplete[1] = FALSE;
			m_bStepRunning[1] = FALSE;
			//_CH2_CHARGE_ENABLE_OFF;
		}
		
		if ( m_bContinueNextSequence[1] )
		{
			m_bContinueNextSequence[1]--;
			if ( m_bContinueNextSequence[1]==0 )
			{
				u8R = FormationEndCheck( 1 );
				if ( !u8R )
				{
					m_bStepStarted[1] = FALSE;
					m_bNewPulseLoad[1] = FALSE;
					m_bNewStepSetting[1] = TRUE;
					m_usReadyToStartSequenceCounter[1] = 0;
					m_bReadyToStartSequence[1] = TRUE;
					if ( m_pSeqNow[1]->usLoopCountNow < m_pSeqNow[1]->usLoopCountMax )
					{
						ClearLoopNowCount(1, m_pSeqNow[1]->ucLoopIndex, m_ucStepIndexNow[1]);
						++m_pSeqNow[1]->usLoopCountNow;
						m_ucStepIndexNow[1] = m_pSeqNow[1]->ucLoopIndex%_MAX_SEQUENCE_STEP;
						m_pSeqNow[1] = & m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[1][m_ucStepIndexNow[1]];
					}
					else
					{
						m_ucStepIndexNow[1] = (++m_ucStepIndexNow[1])%_MAX_SEQUENCE_STEP;
						m_pSeqNow[1] = & m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[1][m_ucStepIndexNow[1]];
					}
					m_pSeqNext[1] = & m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[1][(m_ucStepIndexNow[1]+1)%_MAX_SEQUENCE_STEP];
				}
			}
		}
		
		if ( m_ucStopSequence[1] )
		{
			m_ucStopSequence[1] = 0;
			if ( m_bChannelRunning[1] )
			{
				StepEnd( 1, _STEP_END_BY_USER );
				FormationEnd( 1 );
				m_bStepRunning[1] = FALSE;	//LJK 2025.03.18 StepEnd, Record 2번남는 버그 수정
			}
		}
		//LJK 2023.03.21
		if( m_ucPauseSequence[1] == _PAUSE_ON )
		{
			m_ucStopSequence[1] = m_ucPauseSequence[1] = _PAUSE_NONE;
			if ( m_bChannelRunning[1] )
			{
				StepEnd( 1, _STEP_END_PAUSE );
				FormationEnd( 1 );
			}
		}
		else
		if( m_ucPauseSequence[1] == _RESUME_ON )
		{
			m_ucPauseSequence[1] = FALSE;
			m_ucResumeStatus[1] = TRUE;
			
			m_bRunSequence[1] = TRUE;
			m_ucStartStepIndex[1] = m_ucStepIndexNow[1];
			m_uiPauseTimeNow[1] = m_uiStepTimeNow[1];
		}//		
	}
	{
		if ( m_bRunSequence[2] == TRUE && (m_ui1msCounter & 0x03) == 2 )
		{
			m_ucNowPulseMode[2] = _PULSE_MODE_NONE;
			m_bStepStarted[2] = FALSE;
			m_bNewPulseLoad[2] = FALSE;
			m_bRunSequence[2] = FALSE;
			m_usReadyToStartSequenceCounter[2] = 0;
			m_bNewStepSetting[2] = m_bReadyToStartSequence[2] = TRUE;
			
			m_ucStepIndexNow[2] = m_ucStartStepIndex[2];
			m_pSeqNow[2] = & m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[2][m_ucStepIndexNow[2]];			
			m_pSeqNext[2] = & m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[2][(m_ucStepIndexNow[2] + 1) % _MAX_SEQUENCE_STEP];
			m_ucPulseNextState[2] = m_pSeqNext[2]->ucState;
			m_ucFirstStep[2] = TRUE;
			m_bChannelRunning[2] = TRUE;
			m_ucPreState[2] = !m_ucNowRestChargeDisCharge[2];
			if ( m_pSeqNow[2]->ucState==_STATE_DISCHARGE ||
				m_pSeqNow[2]->ucMode==_MODE_2POINT_PULSE_DISCHARGE || m_pSeqNow[2]->ucMode==_MODE_10POINT_PULSE_DISCHARGE ||  m_pSeqNow[2]->ucMode==_MODE_DISCHARGE_CONTACT )	// bgyu 20250616
			{
				_CH3_CHARGE_ENABLE_OFF;
				_CH3_DISCHARGE_ENABLE_OFF;
				_CH3_MODE_DISCHARGE;
				SetDacWriteData( _DAC_CH3_CC, 0x8000 );	// bgyu 20250528
				SetDacWriteData( _DAC_CH3_CV, 0x0000 );
				_LDAC_ENABLE;
			}
			else
			if ( m_pSeqNow[2]->ucState==_STATE_CHARGE ||
				m_pSeqNow[2]->ucMode==_MODE_2POINT_PULSE_CHARGE || m_pSeqNow[2]->ucMode==_MODE_10POINT_PULSE_CHARGE ||  m_pSeqNow[2]->ucMode==_MODE_CHARGE_CONTACT )	// bgyu 20250616
			{
				_CH3_CHARGE_ENABLE_OFF;
				_CH3_DISCHARGE_ENABLE_OFF;
				_CH3_MODE_CHARGE;
				SetDacWriteData( _DAC_CH3_CC, 0x7fff );	// bgyu 20250528
				SetDacWriteData( _DAC_CH3_CV, 0x7fff );
				_LDAC_ENABLE;
			}						
			NowModeStateSet(2);
			
			// 20221117 djl 추가.
			m_bMcControlAllComplete[2] = FALSE;
			m_bStepRunning[2] = FALSE;
			//_CH3_CHARGE_ENABLE_OFF;
		}
		
		if ( m_bContinueNextSequence[2] )
		{
			m_bContinueNextSequence[2]--;
			if ( m_bContinueNextSequence[2]==0 )
			{				
				u8R = FormationEndCheck( 2 );
				if ( !u8R )
				{
					m_bStepStarted[2] = FALSE;
					m_bNewPulseLoad[2] = FALSE;
					m_bNewStepSetting[2] = TRUE;
					m_usReadyToStartSequenceCounter[2] = 0;
					m_bReadyToStartSequence[2] = TRUE;
					if ( m_pSeqNow[2]->usLoopCountNow < m_pSeqNow[2]->usLoopCountMax )
					{
						ClearLoopNowCount(2, m_pSeqNow[2]->ucLoopIndex, m_ucStepIndexNow[2]);
						++m_pSeqNow[2]->usLoopCountNow;
						m_ucStepIndexNow[2] = m_pSeqNow[2]->ucLoopIndex%_MAX_SEQUENCE_STEP;
						m_pSeqNow[2] = & m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[2][m_ucStepIndexNow[2]];
					}
					else
					{
						m_ucStepIndexNow[2] = (++m_ucStepIndexNow[2])%_MAX_SEQUENCE_STEP;
						m_pSeqNow[2] = & m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[2][m_ucStepIndexNow[2]];
					}
					m_pSeqNext[2] = & m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[2][(m_ucStepIndexNow[2]+1)%_MAX_SEQUENCE_STEP];
				}
			}
		}
		
		if ( m_ucStopSequence[2] )
		{
			m_ucStopSequence[2] = 0;
			if ( m_bChannelRunning[2] )
			{
				StepEnd( 2, _STEP_END_BY_USER );
				FormationEnd( 2 );
				m_bStepRunning[2] = FALSE;	//LJK 2025.03.18 StepEnd, Record 2번남는 버그 수정
			}
		}
		//LJK 2023.03.21
		if( m_ucPauseSequence[2] == _PAUSE_ON )
		{
			m_ucStopSequence[2] = m_ucPauseSequence[2] = _PAUSE_NONE;
			if ( m_bChannelRunning[2] )
			{
				StepEnd( 2, _STEP_END_PAUSE );
				FormationEnd( 2 );
			}
		}		
		else 		
		if( m_ucPauseSequence[2] == _RESUME_ON )
		{
			m_ucPauseSequence[2] = FALSE;
			m_ucResumeStatus[2] = TRUE;
			
			m_bRunSequence[2] = TRUE;
			m_ucStartStepIndex[2] = m_ucStepIndexNow[2];
			m_uiPauseTimeNow[2] = m_uiStepTimeNow[2];
		}//
	}
	
	{
		if ( m_bRunSequence[3] == TRUE && (m_ui1msCounter & 0x03) == 3 )
		{
			m_ucNowPulseMode[3] = _PULSE_MODE_NONE;
			m_bStepStarted[3] = FALSE;
			m_bNewPulseLoad[3] = FALSE;
			m_bRunSequence[3] = FALSE;
			m_usReadyToStartSequenceCounter[3] = 0;
			m_bNewStepSetting[3] = m_bReadyToStartSequence[3] = TRUE;
			m_ucStepIndexNow[3] = m_ucStartStepIndex[3];
			m_pSeqNow[3] = & m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[3][m_ucStepIndexNow[3]];
			m_pSeqNext[3] = & m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[3][(m_ucStepIndexNow[3] + 1) % _MAX_SEQUENCE_STEP];
			m_ucPulseNextState[3] = m_pSeqNext[3]->ucState;
			m_ucFirstStep[3] = TRUE;
			m_bChannelRunning[3] = TRUE;
			m_ucPreState[3] = !m_ucNowRestChargeDisCharge[3];
			if ( m_pSeqNow[3]->ucState==_STATE_DISCHARGE ||
				m_pSeqNow[3]->ucMode==_MODE_2POINT_PULSE_DISCHARGE || m_pSeqNow[3]->ucMode==_MODE_10POINT_PULSE_DISCHARGE ||  m_pSeqNow[3]->ucMode==_MODE_DISCHARGE_CONTACT )	// bgyu 20250616
			{
				_CH4_CHARGE_ENABLE_OFF;
				_CH4_DISCHARGE_ENABLE_OFF;
				_CH4_MODE_DISCHARGE;
				SetDacWriteData( _DAC_CH4_CC, 0x8000 );	// bgyu 20250528
				SetDacWriteData( _DAC_CH4_CV, 0x0000 );
				_LDAC_ENABLE;
			}
			else
			if ( m_pSeqNow[3]->ucState==_STATE_CHARGE ||
				m_pSeqNow[3]->ucMode==_MODE_2POINT_PULSE_CHARGE || m_pSeqNow[3]->ucMode==_MODE_10POINT_PULSE_CHARGE ||  m_pSeqNow[3]->ucMode==_MODE_CHARGE_CONTACT )	// bgyu 20250616
			{
				_CH4_CHARGE_ENABLE_OFF;
				_CH4_DISCHARGE_ENABLE_OFF;
				_CH4_MODE_CHARGE;
				SetDacWriteData( _DAC_CH4_CC, 0x7fff );	// bgyu 20250528
				SetDacWriteData( _DAC_CH4_CV, 0x7fff );
				_LDAC_ENABLE;
			}			
			NowModeStateSet(3);
						
			// 20221117 djl 추가.
			m_bMcControlAllComplete[3] = FALSE;
			m_bStepRunning[3] = FALSE;
			//_CH4_CHARGE_ENABLE_OFF;
		}
		
		if ( m_bContinueNextSequence[3] )
		{
			m_bContinueNextSequence[3]--;
			if( m_bContinueNextSequence[3]==0 )
			{				
				u8R = FormationEndCheck( 3 );
				if ( !u8R )
				{
					m_bStepStarted[3] = FALSE;
					m_bNewPulseLoad[3] = FALSE;
					m_bNewStepSetting[3] = TRUE;
					m_usReadyToStartSequenceCounter[3] = 0;
					m_bReadyToStartSequence[3] = TRUE;
					if ( m_pSeqNow[3]->usLoopCountNow < m_pSeqNow[3]->usLoopCountMax )
					{
						ClearLoopNowCount(3, m_pSeqNow[3]->ucLoopIndex, m_ucStepIndexNow[3]);
						++m_pSeqNow[3]->usLoopCountNow;
						m_ucStepIndexNow[3] = m_pSeqNow[3]->ucLoopIndex%_MAX_SEQUENCE_STEP;
						m_pSeqNow[3] = & m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[3][m_ucStepIndexNow[3]];
					}
					else
					{
						m_ucStepIndexNow[3] = (++m_ucStepIndexNow[3])%_MAX_SEQUENCE_STEP;
						m_pSeqNow[3] = & m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[3][m_ucStepIndexNow[3]];
					}
					m_pSeqNext[3] = & m_pFORMATION_STEP_SEQUENCE_STRUCTURE->stSequence[3][(m_ucStepIndexNow[3]+1)%_MAX_SEQUENCE_STEP];
				}
			}
		}
		
		if ( m_ucStopSequence[3] )
		{
			m_ucStopSequence[3] = 0;
			if ( m_bChannelRunning[3] )
			{
				StepEnd( 3, _STEP_END_BY_USER );
				FormationEnd( 3 );
				m_bStepRunning[3] = FALSE;	//LJK 2025.03.18 StepEnd, Record 2번남는 버그 수정
			}
		}
		//LJK 2023.03.21
		if( m_ucPauseSequence[3] == _PAUSE_ON )
		{
			m_ucStopSequence[3] = m_ucPauseSequence[3] = _PAUSE_NONE;
			if ( m_bChannelRunning[3] )
			{
				StepEnd( 3, _STEP_END_PAUSE );
				FormationEnd( 3 );
			}
		}
		else
		if( m_ucPauseSequence[3] == _RESUME_ON )
		{
			m_ucPauseSequence[3] = FALSE;
			m_ucResumeStatus[3] = TRUE;
			
			m_bRunSequence[3] = TRUE;
			m_ucStartStepIndex[3] = m_ucStepIndexNow[3];
			m_uiPauseTimeNow[3] = m_uiStepTimeNow[3];
		}//		
	}

	{
		if ( m_bStepStarted[0] )
		{
			m_uiStepTimeNow[0]++;
			
			// 20230120 djl Run 하고 맨 첫번째 데이터는 안남김.
			#ifdef MODIFY_DCIR_10MSEC_MODE
			if((m_ucNowMode[0] == _MODE_10ms_DCIR) && (m_ucNowRestChargeDisCharge[0] == _NOW_STATE_DISCHARGE))
			{
			    if( m_bChannelRunning[0] && (m_uiStepTimeNow[0]>3001) && (m_uiStepTimeNow[0]<3013) && (m_bDCIR_StepStop[0] == FALSE))
    			{
    				PushStepRecordData(0, 0);
    		    }
			}
			else
			#endif
			{
			    if( m_bChannelRunning[0] && m_uiStepTimeNow[0]>1 && (m_uiStepTimeNow[0]-1) % m_uiRecordMilliSecond[0] == 0)
				    PushStepRecordData(0, 0);
		    }
		}

		if ( m_bNewPulseLoad[0] )
		{
			m_bNewPulseLoad[0] = FALSE;
			SetPulseModeChange(0);
			_LDAC_ENABLE;
			m_bStepStarted[0] = TRUE;
			if ( m_ucFirstStep2[0] )
			{
				m_ucFirstStep2[0] = FALSE;
				m_ui1SecRingCountVoltage[0] = m_ui1SecRingCountCurrent[0] = m_ui1SecRingCountVoltage2[0] = 0xffffffff;	// 2 Captain Pulse뒤부터 유효함
				m_uiRecordRingCountVoltage[0] = m_uiRecordRingCountCurrent[0] = m_uiRecordRingCountVoltage2[0] = 0;
			}
		}
		if ( m_bNewStepStartReady[0] )
		{
			m_bNewStepStartReady[0] = FALSE;
			StepStart(0);
			if ( m_ucFirstStep[0] )
			{
				m_ucFirstStep[0] = FALSE;
				m_ucFirstStep2[0] = TRUE;
				m_bNewPulseLoad[0] = TRUE;
			}
		}
		
		McControl(0);

		if ( m_bReadyToStartSequence[0] )
		{
			if( !ReadyToStartForSafety(0) )
			{
				m_usReadyToStartSequenceCounter[0]++;
				goto _GOTO_CAPTAIN0 ;
			}
			m_bReadyToStartSequence[0] = FALSE;
		}
		
		if ( m_bNewStepSetting[0] && m_bMcControlAllComplete[0] )
		{
			m_bMcControlAllComplete[0] = m_bNewStepSetting[0] = FALSE;
			NewStepSetting(0);
			m_ucFirstStep2[0] = TRUE;
		}
		else
		{
			if ( m_bStepRunning[0] )
			{
				SetNextPulse(0);
			}
		}

		CalculateCapacity(0);
		if ( m_bChannelRunning[0] && m_bAlarmParameterSet[0])
		{
			if ( (usR = AlarmCheck( 0 )) )
			{
				StepEnd( 0, usR );
				FormationEnd( 0 );
			}
		}
		
		StepEndCheck(0);
		
		if ( m_bStepStarted[0] )
			CpCrMode(0);
	}
	_GOTO_CAPTAIN0:

	{
		if ( m_bStepStarted[1] )
		{
			m_uiStepTimeNow[1]++;
			
			#ifdef MODIFY_DCIR_10MSEC_MODE
			if((m_ucNowMode[1] == _MODE_10ms_DCIR) && (m_ucNowRestChargeDisCharge[1] == _NOW_STATE_DISCHARGE))
			{
			    if( m_bChannelRunning[1] && (m_uiStepTimeNow[1]>3001) && (m_uiStepTimeNow[1]<3013) && (m_bDCIR_StepStop[1] == FALSE))
    			{
    				PushStepRecordData(1, 0);
    		    }
			}
			else
			#endif
			{
    			if( m_bChannelRunning[1] && m_uiStepTimeNow[1]>1 &&(m_uiStepTimeNow[1]-1) % m_uiRecordMilliSecond[1] == 0)
    				PushStepRecordData(1, 0);
    		}
		}

		if ( m_bNewPulseLoad[1] )
		{
			m_bNewPulseLoad[1] = FALSE;
			SetPulseModeChange(1);
			_LDAC_ENABLE;
			m_bStepStarted[1] = TRUE;
			if ( m_ucFirstStep2[1] )
			{
				m_ucFirstStep2[1] = FALSE;
				m_ui1SecRingCountVoltage[1] = m_ui1SecRingCountCurrent[1] = m_ui1SecRingCountVoltage2[1] = 0xffffffff;	// 2 Captain Pulse뒤부터 유효함
				m_uiRecordRingCountVoltage[1] = m_uiRecordRingCountCurrent[1] = m_uiRecordRingCountVoltage2[1] = 0;
			}
		}
		if ( m_bNewStepStartReady[1] )
		{
			m_bNewStepStartReady[1] = FALSE;
			StepStart(1);
			if ( m_ucFirstStep[1] )
			{
				m_ucFirstStep[1] = FALSE;
				m_ucFirstStep2[1] = TRUE;
				m_bNewPulseLoad[1] = TRUE;
			}
		}
		
		McControl(1);

		if ( m_bReadyToStartSequence[1] )
		{
			if( !ReadyToStartForSafety(1) )
			{
				m_usReadyToStartSequenceCounter[1]++;
				goto _GOTO_CAPTAIN1;

			}
			m_bReadyToStartSequence[1] = FALSE;
		}
		
		if ( m_bNewStepSetting[1] && m_bMcControlAllComplete[1] )
		{
			m_bMcControlAllComplete[1] = m_bNewStepSetting[1] = FALSE;
			NewStepSetting(1);
			m_ucFirstStep2[1] = TRUE;
		}
		else
		{
			if ( m_bStepRunning[1] )
			{
				SetNextPulse(1);
			}
		}

		CalculateCapacity(1);
		if ( m_bChannelRunning[1] && m_bAlarmParameterSet[1])
		{
			if ( (usR = AlarmCheck( 1 )) )
			{
				StepEnd( 1, usR );
				FormationEnd( 1 );
			}
		}
		
		StepEndCheck(1);
		
		if ( m_bStepStarted[1] )
			CpCrMode(1);
	}
	_GOTO_CAPTAIN1:
	{
		if ( m_bStepStarted[2] )
		{
			m_uiStepTimeNow[2]++;
			
			#ifdef MODIFY_DCIR_10MSEC_MODE
			if((m_ucNowMode[2] == _MODE_10ms_DCIR) && (m_ucNowRestChargeDisCharge[2] == _NOW_STATE_DISCHARGE))
			{
			    if( m_bChannelRunning[2] && (m_uiStepTimeNow[2]>3001) && (m_uiStepTimeNow[2]<3013) && (m_bDCIR_StepStop[2] == FALSE))
    			{
    				PushStepRecordData(2, 0);
    		    }
			}
			else
			#endif
			{
    			if( m_bChannelRunning[2] && m_uiStepTimeNow[2]>1 &&(m_uiStepTimeNow[2]-1) % m_uiRecordMilliSecond[2] == 0)
    				PushStepRecordData(2, 0);
    	    }
		}

		if ( m_bNewPulseLoad[2] )
		{
			m_bNewPulseLoad[2] = FALSE;
			SetPulseModeChange(2);
			_LDAC_ENABLE;
			m_bStepStarted[2] = TRUE;
			if ( m_ucFirstStep2[2] )
			{
				m_ucFirstStep2[2] = FALSE;
				m_ui1SecRingCountVoltage[2] = m_ui1SecRingCountCurrent[2] = m_ui1SecRingCountVoltage2[2] = 0xffffffff;	// 2 Captain Pulse뒤부터 유효함
				m_uiRecordRingCountVoltage[2] = m_uiRecordRingCountCurrent[2] = m_uiRecordRingCountVoltage2[2] = 0;
			}
		}
		if ( m_bNewStepStartReady[2] )
		{
			m_bNewStepStartReady[2] = FALSE;
			StepStart(2);
			if ( m_ucFirstStep[2] )
			{
				m_ucFirstStep[2] = FALSE;
				m_ucFirstStep2[2] = TRUE;
				m_bNewPulseLoad[2] = TRUE;
			}
		}
		
		McControl(2);

		if ( m_bReadyToStartSequence[2] )
		{
			if( !ReadyToStartForSafety(2) )
			{
				m_usReadyToStartSequenceCounter[2]++;
				goto _GOTO_CAPTAIN2;
			}
			m_bReadyToStartSequence[2] = FALSE;
		}
		
		if ( m_bNewStepSetting[2] && m_bMcControlAllComplete[2] )
		{
			m_bMcControlAllComplete[2] = m_bNewStepSetting[2] = FALSE;
			NewStepSetting(2);
			m_ucFirstStep2[2] = TRUE;
		}
		else
		{
			if ( m_bStepRunning[2] )
			{
				SetNextPulse(2);
			}
		}

		CalculateCapacity(2);
		if ( m_bChannelRunning[2] && m_bAlarmParameterSet[2])
		{
			if ( (usR = AlarmCheck( 2 )) )
			{
				StepEnd( 2, usR );
				FormationEnd( 2 );
			}
		}
		
		StepEndCheck(2);
		
		if ( m_bStepStarted[2] )
			CpCrMode(2);
	}
	_GOTO_CAPTAIN2:
	
	{
		if ( m_bStepStarted[3] )
		{
			m_uiStepTimeNow[3]++;
			
			#ifdef MODIFY_DCIR_10MSEC_MODE
			if((m_ucNowMode[3] == _MODE_10ms_DCIR) && (m_ucNowRestChargeDisCharge[3] == _NOW_STATE_DISCHARGE))
			{
			    if( m_bChannelRunning[3] && (m_uiStepTimeNow[3]>3001) && (m_uiStepTimeNow[3]<3013) && (m_bDCIR_StepStop[3] == FALSE))
    			{
    				PushStepRecordData(3, 0);
    		    }
			}
			else
			#endif
			{
    			if( m_bChannelRunning[3] && m_uiStepTimeNow[3]>1 &&(m_uiStepTimeNow[3]-1) % m_uiRecordMilliSecond[3] == 0)
    				PushStepRecordData(3, 0);
    		}
		}

		if ( m_bNewPulseLoad[3] )
		{
			m_bNewPulseLoad[3] = FALSE;
			SetPulseModeChange(3);
			_LDAC_ENABLE;
			m_bStepStarted[3] = TRUE;
			if ( m_ucFirstStep2[3] )
			{
				m_ucFirstStep2[3] = FALSE;
				m_ui1SecRingCountVoltage[3] = m_ui1SecRingCountCurrent[3] = m_ui1SecRingCountVoltage2[3] = 0xffffffff;	// 2 Captain Pulse뒤부터 유효함
				m_uiRecordRingCountVoltage[3] = m_uiRecordRingCountCurrent[3] = m_uiRecordRingCountVoltage2[3] = 0;
			}
		}
		if ( m_bNewStepStartReady[3] )
		{
			m_bNewStepStartReady[3] = FALSE;
			StepStart(3);
			if ( m_ucFirstStep[3] )
			{
				m_ucFirstStep[3] = FALSE;
				m_ucFirstStep2[3] = TRUE;
				m_bNewPulseLoad[3] = TRUE;
			}
		}
		
		McControl(3);

		if ( m_bReadyToStartSequence[3] )
		{
			if( !ReadyToStartForSafety(3) )
			{
				m_usReadyToStartSequenceCounter[3]++;
				goto _GOTO_CAPTAIN3;
			}
			m_bReadyToStartSequence[3] = FALSE;
		}
		
		if ( m_bNewStepSetting[3] && m_bMcControlAllComplete[3] )
		{
			m_bMcControlAllComplete[3] = m_bNewStepSetting[3] = FALSE;
			NewStepSetting(3);
			m_ucFirstStep2[3] = TRUE;
		}
		else
		{
			if ( m_bStepRunning[3] )
			{
				SetNextPulse(3);
			}
		}

		CalculateCapacity(3);
		if ( m_bChannelRunning[3] && m_bAlarmParameterSet[3])
		{
			if ( (usR = AlarmCheck( 3 )) )
			{
				StepEnd( 3, usR );
				FormationEnd( 3 );
			}
		}
		
		StepEndCheck(3);
		
		if ( m_bStepStarted[3] )
			CpCrMode(3);
	}
	_GOTO_CAPTAIN3:

	if ( !(m_ui1msCounter++ % 500) )	//LJK 2023.05.19 500ms -> 100ms
		m_bLedRunStatus = TRUE;

	m_uiAdcReadCycleInterruptCount = 0;

	if ( m_iAdcReadComplete!=0x283 && !u8First )
	{
		m_bAdcTimeOver = TRUE;
		
		m_uiSystemInformation[_AdcReadCompleteErrorCode] = m_iAdcReadComplete;
		m_uiSystemInformation[_AdcReadCompleteError]++;
	}
	m_iAdcReadComplete = 0;
	u8First = 0;
	
	PcCommunicationService();
	CAPTAIN_TP_L;
}

void CaptainTimerCounterSetup( volatile avr32_tc_t *pTc )
{
	gpio_enable_module_pin( AVR32_TC0_CLK0_PIN, AVR32_TC0_CLK0_FUNCTION );
	
	static const tc_waveform_opt_t waveform_opt =
	{
		.channel  = CAPTAIN_TIMER_COUNT_CHANNEL,
		.bswtrg   = TC_EVT_EFFECT_NOOP,
		.beevt    = TC_EVT_EFFECT_NOOP,
		.bcpc     = TC_EVT_EFFECT_NOOP,
		.bcpb     = TC_EVT_EFFECT_NOOP,
		.aswtrg   = TC_EVT_EFFECT_NOOP,
		.aeevt    = TC_EVT_EFFECT_NOOP,
		.acpc     = TC_EVT_EFFECT_NOOP,
		.acpa     = TC_EVT_EFFECT_NOOP,
		.wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,
		.enetrg   = false,
		.eevt     = 0,
		.eevtedg  = TC_SEL_NO_EDGE,
		.cpcdis   = false,
		.cpcstop  = false,
		.burst    = false,
		.clki     = false,
		.tcclks   = TC_CLOCK_SOURCE_XC0 //TC_CLOCK_SOURCE_TC3
	};

	static const tc_interrupt_t tc_interrupt =
	{
		.etrgs = 0,
		.ldrbs = 0,
		.ldras = 0,
		.cpcs  = 1,
		.cpbs  = 0,
		.cpas  = 0,
		.lovrs = 0,
		.covfs = 0
	};
	
	tc_init_waveform( pTc, &waveform_opt);

	double dblScaler;
	unsigned short usScaler;
	dblScaler = ((double)BOARD_OSC0_HZ) * 0.001;	// 1ms
	usScaler = (unsigned short)dblScaler;
	m_iCaptainClockCountT1Over = (unsigned short)(dblScaler*0.95);
	tc_write_rc( pTc, CAPTAIN_TIMER_COUNT_CHANNEL, usScaler );
	tc_configure_interrupts( pTc, CAPTAIN_TIMER_COUNT_CHANNEL, &tc_interrupt);
}

void AdcReadCycleTimerCounterSetup( volatile avr32_tc_t *pTc )
{
	static const tc_waveform_opt_t WAVEFORM_OPT =
	{
		.channel  = ADC_READ_CYCLE_TIMER_COUNT_CHANNEL,
		.bswtrg   = TC_EVT_EFFECT_NOOP,
		.beevt    = TC_EVT_EFFECT_NOOP,
		.bcpc     = TC_EVT_EFFECT_NOOP,
		.bcpb     = TC_EVT_EFFECT_NOOP,
		.aswtrg   = TC_EVT_EFFECT_NOOP,
		.aeevt    = TC_EVT_EFFECT_NOOP,
		.acpc     = TC_EVT_EFFECT_NOOP,
		.acpa     = TC_EVT_EFFECT_NOOP,
		.wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,
		.enetrg   = FALSE,
		.eevt     = 0,
		.eevtedg  = TC_SEL_NO_EDGE,
		.cpcdis   = FALSE,
		.cpcstop  = FALSE,
		.burst    = FALSE,
		.clki     = FALSE,
		.tcclks   = TC_CLOCK_SOURCE_TC3
	};

	static const tc_interrupt_t TC_INTERRUPT =
	{
		.etrgs = 0,
		.ldrbs = 0,
		.ldras = 0,
		.cpcs  = 1,
		.cpbs  = 0,
		.cpas  = 0,
		.lovrs = 0,
		.covfs = 0
	};

	tc_init_waveform( pTc, &WAVEFORM_OPT );
	
	double dblScaler;
	dblScaler = ((double)m_iPbcHz / 8.) * 0.0004;	/// Never Don't Touch : 400us
	//dblScaler = ((double)m_iPbcHz / 8.) * 0.00026;	/// Never Don't Touch : 260us	
	m_usScalerAdcRead = (unsigned short)dblScaler;
	tc_write_rc( pTc, ADC_READ_CYCLE_TIMER_COUNT_CHANNEL, 2 );
	tc_configure_interrupts( pTc, ADC_READ_CYCLE_TIMER_COUNT_CHANNEL, &TC_INTERRUPT );
}

void TimerInterruptCounterStart( void )
{
	tc_start( CAPTAIN_TIMER_COUNT_ADDRESS, CAPTAIN_TIMER_COUNT_CHANNEL);
//	tc_start( COMMUNICATION_TIMER_COUNT_ADDRESS, COMMUNICATION_TIMER_COUNT_CHANNEL);
}


__attribute__((__interrupt__)) static void AdcReadCycleInterruptHandler( void )
{
	ADC_TP_H;
	if ( m_uiAdcReadCycleInterruptCount==0 )
	{
		tc_write_rc( ADC_READ_CYCLE_TIMER_COUNT_ADDRESS, ADC_READ_CYCLE_TIMER_COUNT_CHANNEL, m_usScalerAdcRead );	// 500us단위
		tc_start( ADC_READ_CYCLE_TIMER_COUNT_ADDRESS, ADC_READ_CYCLE_TIMER_COUNT_CHANNEL );
	}
	ADC_READ_CYCLE_TIMER_COUNT_ADDRESS->channel[ADC_READ_CYCLE_TIMER_COUNT_CHANNEL].sr;
	
	int iCaptainCount1 = CAPTAIN_TIMER_COUNT_ADDRESS->channel[CAPTAIN_TIMER_COUNT_CHANNEL].cv;	// Current Time
	if ( m_iCaptainClockCountT1Over<=iCaptainCount1 )
	{
		ADC_READ_CYCLE_TIMER_COUNT_ADDRESS->channel[ADC_READ_CYCLE_TIMER_COUNT_CHANNEL].ccr = AVR32_TC_CLKDIS_MASK;
		m_uiSystemInformation[_SystemErrorFlagCounter] += 0x10000;
		m_uiSystemInformation[_SystemErrorFlag] |= 1<<SystemErrorFlagAdcTimeOver;
	}
	
	
	ReadAdcSpi8One( &m_iV[0], &m_iC[0] );	// Ch1~Ch4
	if ( m_uiAdcReadCycleInterruptCount==1 )
	{
		memset(m_iVoltage, 0, sizeof(m_iVoltage));
		memset(m_iCurrent, 0, sizeof(m_iCurrent));
	}
	m_iVoltage[0] += (int)(short)(unsigned short)m_iV[0];
	m_iCurrent[0] += (int)(short)(unsigned short)m_iC[0];
	m_iVoltage[1] += (int)(short)(unsigned short)m_iV[1];
	m_iCurrent[1] += (int)(short)(unsigned short)m_iC[1];
	m_iVoltage[2] += (int)(short)(unsigned short)m_iV[2];
	m_iCurrent[2] += (int)(short)(unsigned short)m_iC[2];
	m_iVoltage[3] += (int)(short)(unsigned short)m_iV[3];
	m_iCurrent[3] += (int)(short)(unsigned short)m_iC[3];
		
	m_iAdcReadComplete |= 1<<m_uiAdcReadCycleInterruptCount;

	m_uiAdcReadCycleInterruptCount++;
	if(m_bAdcTimeOver == TRUE )
	{
		m_bAdcTimeOver = FALSE;
		
		memcpy(m_fVoltage, m_fAdcPreVoltage, sizeof(m_fAdcPreVoltage));
		memcpy(m_fWatt, m_fAdcPreWatt, sizeof(m_fAdcPreWatt));
		memcpy(m_fCurrent, m_fAdcPreCurrent, sizeof(m_fAdcPreCurrent));

		m_iAdcReadComplete = 0x283;
		m_uiAdcReadCycleInterruptCount = 1;
		return;
	}
	
	if ( m_uiAdcReadCycleInterruptCount==1 )
	{
		m_iAdcReadComplete |= 1<<7;
		
		{
			m_iNowVoltageAdc[0] = m_iVoltage[0];
			m_iNowCurrentAdc[0] = m_iCurrent[0];
			m_i1msVoltage[0] = m_iNowVoltageAdc[0];
			m_fNowVoltage[0] = GetHexToVoltageCurrent( VOLTAGE_RANGE_INDEX+0, m_iNowVoltageAdc[0]*1000 );
			
			if ( m_bVoltage2Mode[0] )
			{
				m_i1msVoltage2[0] = m_iNowCurrentAdc[0];
				m_fVoltage2[0] = m_fNowVoltage2[0] = GetHexToVoltageCurrent( VOLTAGE2_RANGE_INDEX+0, m_iNowCurrentAdc[0]*1000 );
			}
			else
			{
				if( m_iNowCurrentAdc[0] & 0x80000000)
				{
					m_ucCalculateMode[0] = _NOW_STATE_DISCHARGE;
					m_fCurrent[0] = m_fNowCurrent[0] = GetHexToVoltageCurrent( CURRENT_RANGE_INDEX+0*2, m_iNowCurrentAdc[0]*1000 );		// Discharge
				}
				else
				{
					m_ucCalculateMode[0] = _NOW_STATE_CHARGE;
					m_fCurrent[0] = m_fNowCurrent[0] = GetHexToVoltageCurrent( CURRENT_RANGE_INDEX+0*2+1, m_iNowCurrentAdc[0]*1000 );		// Charge
				}
				m_i1msCurrent[0] = m_iNowCurrentAdc[0];
			}
			
			if ( m_ucNowState[0] ==_STATE_PULSE )
			{	//LJK 2024.02.15, 2024.04.03 다시살림
				if ( m_u16PulseTime1ms[0] - 1 == m_u16PulseTime1msNow[0] )//+4 && m_u16PulseTime1msNow>4 )
				{
					if ( m_ucCalculateMode[0]==_NOW_STATE_CHARGE )
					{
						m_fChargeVoltage[0] = m_fNowVoltage[0];
						m_fChargeCurrent[0] = m_fNowCurrent[0];
					}
					else
					{
						m_fDisChargeVoltage[0] = m_fNowVoltage[0];
						m_fDisChargeCurrent[0] = m_fNowCurrent[0];
					}
				}
			}
				
			m_fVoltage[0] = m_fNowVoltage[0];
			m_fWatt[0] = m_fVoltage[0]*m_fCurrent[0];
			m_fResister[0] = m_fVoltage[0]/m_fCurrent[0];
		}
		
		{
			m_iNowVoltageAdc[1] = m_iVoltage[1];
			m_iNowCurrentAdc[1] = m_iCurrent[1];
			m_i1msVoltage[1] = m_iNowVoltageAdc[1];
			m_fNowVoltage[1] = GetHexToVoltageCurrent( VOLTAGE_RANGE_INDEX+1, m_iNowVoltageAdc[1]*1000 );
			
			if ( m_bVoltage2Mode[1] )
			{
				m_i1msVoltage2[1] = m_iNowCurrentAdc[1];
				m_fVoltage2[1] = m_fNowVoltage2[1] = GetHexToVoltageCurrent( VOLTAGE2_RANGE_INDEX+1, m_iNowCurrentAdc[1]*1000 );
			}
			else
			{
				if( m_iNowCurrentAdc[1] & 0x80000000)
				{
					m_ucCalculateMode[1] = _NOW_STATE_DISCHARGE;
					m_fCurrent[1] = m_fNowCurrent[1] = GetHexToVoltageCurrent( CURRENT_RANGE_INDEX+1*2, m_iNowCurrentAdc[1]*1000 );		// Discharge
				}
				else
				{
					m_ucCalculateMode[1] = _NOW_STATE_CHARGE;
					m_fCurrent[1] = m_fNowCurrent[1] = GetHexToVoltageCurrent( CURRENT_RANGE_INDEX+1*2+1, m_iNowCurrentAdc[1]*1000 );		// Charge
				}
				m_i1msCurrent[1] = m_iNowCurrentAdc[1];			
			}
			
			if ( m_ucNowState[1] ==_STATE_PULSE )
			{	//LJK 2024.02.15, 2024.04.03 다시살림
				if ( m_u16PulseTime1ms[1] - 1 == m_u16PulseTime1msNow[1] )//+4 && m_u16PulseTime1msNow>4 )				
				{
					if ( m_ucCalculateMode[1]==_NOW_STATE_CHARGE )
					{
						m_fChargeVoltage[1] = m_fNowVoltage[1];
						m_fChargeCurrent[1] = m_fNowCurrent[1];
					}
					else
					{
						m_fDisChargeVoltage[1] = m_fNowVoltage[1];
						m_fDisChargeCurrent[1] = m_fNowCurrent[1];
					}
				}
			}
			
			m_fVoltage[1] = m_fNowVoltage[1];
			m_fWatt[1] = m_fVoltage[1]*m_fCurrent[1];
			m_fResister[1] = m_fVoltage[1]/m_fCurrent[1];
		}
		
		{
			m_iNowVoltageAdc[2] = m_iVoltage[2];
			m_iNowCurrentAdc[2] = m_iCurrent[2];
			m_i1msVoltage[2] = m_iNowVoltageAdc[2];
			m_fNowVoltage[2] = GetHexToVoltageCurrent( VOLTAGE_RANGE_INDEX+2, m_iNowVoltageAdc[2]*1000 );
			
			if ( m_bVoltage2Mode[2] )
			{
				m_i1msVoltage2[2] = m_iNowCurrentAdc[2];
				m_fVoltage2[2] = m_fNowVoltage2[2] = GetHexToVoltageCurrent( VOLTAGE2_RANGE_INDEX+2, m_iNowCurrentAdc[2]*1000 );
			}
			else
			{
				if( m_iNowCurrentAdc[2] & 0x80000000)
				{
					m_ucCalculateMode[2] = _NOW_STATE_DISCHARGE;
					m_fCurrent[2] = m_fNowCurrent[2] = GetHexToVoltageCurrent( CURRENT_RANGE_INDEX+2*2, m_iNowCurrentAdc[2]*1000 );		// Discharge
				}
				else
				{
					m_ucCalculateMode[2] = _NOW_STATE_CHARGE;
					m_fCurrent[2] = m_fNowCurrent[2] = GetHexToVoltageCurrent( CURRENT_RANGE_INDEX+2*2+1, m_iNowCurrentAdc[2]*1000 );		// Charge
				}
				m_i1msCurrent[2] = m_iNowCurrentAdc[2];
			}
			if ( m_ucNowState[2] ==_STATE_PULSE )
			{	//LJK 2024.02.15, 2024.04.03 다시살림
				if ( m_u16PulseTime1ms[2] - 1 == m_u16PulseTime1msNow[2] )//+4 && m_u16PulseTime1msNow>4 )
				{
					if ( m_ucCalculateMode[2]==_NOW_STATE_CHARGE )
					{
						m_fChargeVoltage[2] = m_fNowVoltage[2];
						m_fChargeCurrent[2] = m_fNowCurrent[2];
					}
					else
					{
						m_fDisChargeVoltage[2] = m_fNowVoltage[2];
						m_fDisChargeCurrent[2] = m_fNowCurrent[2];
					}
				}
			}
			
			m_fVoltage[2] = m_fNowVoltage[2];
			m_fWatt[2] = m_fVoltage[2]*m_fCurrent[2];
			m_fResister[2] = m_fVoltage[2]/m_fCurrent[2];
		}
		
		{
			m_iNowVoltageAdc[3] = m_iVoltage[3];
			m_iNowCurrentAdc[3] = m_iCurrent[3];
			m_i1msVoltage[3] = m_iNowVoltageAdc[3];
			m_fNowVoltage[3] = GetHexToVoltageCurrent( VOLTAGE_RANGE_INDEX+3, m_iNowVoltageAdc[3]*1000 );
			
			if ( m_bVoltage2Mode[3] )
			{
				m_i1msVoltage2[3] = m_iNowCurrentAdc[3];
				m_fVoltage2[3] = m_fNowVoltage2[3] = GetHexToVoltageCurrent( VOLTAGE2_RANGE_INDEX+3, m_iNowCurrentAdc[3]*1000 );
			}
			else
			{
				if( m_iNowCurrentAdc[3] & 0x80000000)
				{
					m_ucCalculateMode[3] = _NOW_STATE_DISCHARGE;
					m_fCurrent[3] = m_fNowCurrent[3] = GetHexToVoltageCurrent( CURRENT_RANGE_INDEX+3*2, m_iNowCurrentAdc[3]*1000 );		// Discharge
				}
				else
				{
					m_ucCalculateMode[3] = _NOW_STATE_CHARGE;
					m_fCurrent[3] = m_fNowCurrent[3] = GetHexToVoltageCurrent( CURRENT_RANGE_INDEX+3*2+1, m_iNowCurrentAdc[3]*1000 );		// Charge
				}
				m_i1msCurrent[3] = m_iNowCurrentAdc[3];
			}
			if ( m_ucNowState[3] ==_STATE_PULSE )
			{	//LJK 2024.02.15, 2024.04.03 다시살림
				if ( m_u16PulseTime1ms[3] - 1 == m_u16PulseTime1msNow[3] )//+4 && m_u16PulseTime1msNow>4 )
				{
					if ( m_ucCalculateMode[3]==_NOW_STATE_CHARGE )
					{
						m_fChargeVoltage[3] = m_fNowVoltage[3];
						m_fChargeCurrent[3] = m_fNowCurrent[3];
					}
					else
					{
						m_fDisChargeVoltage[3] = m_fNowVoltage[3];
						m_fDisChargeCurrent[3] = m_fNowCurrent[3];
					}
				}
			}
			
			m_fVoltage[3] = m_fNowVoltage[3];
			m_fWatt[3] = m_fVoltage[3]*m_fCurrent[3];
			m_fResister[3] = m_fVoltage[3]/m_fCurrent[3];
		}

		if ( m_bVoltage2Mode[0] ) GetRingBufferVoltage2(0);
		else            		  GetRingBufferCurrent(0);
			
		GetRingBufferVoltage(0);
		
		if ( m_bVoltage2Mode[1] ) GetRingBufferVoltage2(1);
		else                      GetRingBufferCurrent(1);

		GetRingBufferVoltage(1);
		
		if ( m_bVoltage2Mode[2] ) GetRingBufferVoltage2(2);
		else                      GetRingBufferCurrent(2);
		
		GetRingBufferVoltage(2);

		if ( m_bVoltage2Mode[3] ) GetRingBufferVoltage2(3);
		else                      GetRingBufferCurrent(3);

		GetRingBufferVoltage(3);
	
		// 20221223 DJL ADC Timeout
		if(m_bAdcTimeOver == FALSE)
		{
			memcpy(m_fAdcPreVoltage, m_fVoltage, sizeof(m_fAdcPreVoltage));
			memcpy(m_fAdcPreWatt, m_fWatt, sizeof(m_fAdcPreWatt));
			memcpy(m_fAdcPreCurrent, m_fCurrent, sizeof(m_fAdcPreCurrent));
		}
	}
	if ( m_uiAdcReadCycleInterruptCount>=2 )				// End & Safe
	{
		m_iAdcReadComplete |= 1<<9;
		ADC_READ_CYCLE_TIMER_COUNT_ADDRESS->channel[ADC_READ_CYCLE_TIMER_COUNT_CHANNEL].ccr = AVR32_TC_CLKDIS_MASK;
	}
	
	ADC_TP_L;
}

void SetUpInterruptHandler( void )
{
	INTC_register_interrupt ((__int_handler)&CaptainTimerCounterInterrupHandler,	CAPTAIN_TIMER_COUNT_IRQ, AVR32_INTC_INT3 );
	INTC_register_interrupt ((__int_handler)&AdcReadCycleInterruptHandler,			ADC_READ_CYCLE_TIMER_COUNT_IRQ, AVR32_INTC_INT2 );

	INTC_register_interrupt( (__int_handler)&PcCommInterruptHandler,					PC_COMM_USART_IRQ, AVR32_INTC_INT1);
	INTC_register_interrupt( (__int_handler)&PDCA_PcCommRxInterrupHandler,				AVR32_PDCA_IRQ_0, AVR32_INTC_INT1 );
	INTC_register_interrupt( (__int_handler)&PDCA_PcCommTxInterrupHandler,				AVR32_PDCA_IRQ_1, AVR32_INTC_INT1 );
}

void SetUpInternelADC( void )
{
	adcifa_sequencer_opt_t adcifa_sequence_opt ;
	adcifa_sequencer_conversion_opt_t adcifa_sequence_conversion_opt[2];
	adcifa_opt_t adc_config_t ;
	
	/* Configure the ADC for the application*/
	adc_config_t.frequency                = 1000000;
	adc_config_t.reference_source         = ADCIFA_REF06VDD;
	adc_config_t.sample_and_hold_disable  = false;
	adc_config_t.single_sequencer_mode    = false;
	adc_config_t.free_running_mode_enable = false;
	adc_config_t.sleep_mode_enable        = false;
	adc_config_t.mux_settle_more_time     = false;

	/* Get ADCIFA Factory Configuration */
	adcifa_get_calibration_data(&AVR32_ADCIFA, &adc_config_t);

	/* Calibrate offset first*/
	adcifa_calibrate_offset(&AVR32_ADCIFA, &adc_config_t, sysclk_get_cpu_hz());

	/* Configure ADCIFA core */
	adcifa_configure(&AVR32_ADCIFA, &adc_config_t, sysclk_get_cpu_hz());

	/* ADCIFA sequencer 0 configuration structure*/
	//adcifa_sequence_opt.convnb              = 1;
	//adcifa_sequence_opt.resolution          = ADCIFA_SRES_12B;
	//adcifa_sequence_opt.trigger_selection   = ADCIFA_TRGSEL_SOFT;
	//adcifa_sequence_opt.start_of_conversion = ADCIFA_SOCB_SINGLECONV;
	//adcifa_sequence_opt.sh_mode             = ADCIFA_SH_MODE_STANDARD;
	//adcifa_sequence_opt.half_word_adjustment= ADCIFA_HWLA_NOADJ;
	//adcifa_sequence_opt.software_acknowledge= ADCIFA_SA_NO_EOS_SOFTACK;

	/* ADCIFA conversions for sequencer 0*/
	//adcifa_sequence_conversion_opt.channel_p = AVR32_ADCIFA_INP_ADCIN1;	//DACTemp AVR32_ADCIFA_INP_ADCIN2;	//FET
	//adcifa_sequence_conversion_opt.channel_n = AVR32_ADCIFA_INN_GNDANA;
	//adcifa_sequence_conversion_opt.gain      = ADCIFA_SHG_1;

	/* Configure ADCIFA sequencer 0 */
	//adcifa_configure_sequencer(&AVR32_ADCIFA, 0, &adcifa_sequence_opt, &adcifa_sequence_conversion_opt);
	
	adcifa_sequence_opt.convnb              = 2;
	adcifa_sequence_opt.resolution          = ADCIFA_SRES_12B;
	adcifa_sequence_opt.trigger_selection   = ADCIFA_TRGSEL_SOFT;
	adcifa_sequence_opt.start_of_conversion = ADCIFA_SOCB_ALLSEQ;
	adcifa_sequence_opt.sh_mode             = ADCIFA_SH_MODE_OVERSAMP;
	adcifa_sequence_opt.half_word_adjustment= ADCIFA_HWLA_NOADJ;
	adcifa_sequence_opt.software_acknowledge= ADCIFA_SA_NO_EOS_SOFTACK;

	adcifa_sequence_conversion_opt[0].channel_p = AVR32_ADCIFA_INP_ADCIN0;	//BoardAirTemperature
	adcifa_sequence_conversion_opt[0].channel_n = AVR32_ADCIFA_INN_GNDANA;
	adcifa_sequence_conversion_opt[0].gain      = ADCIFA_SHG_1;
	
	adcifa_sequence_conversion_opt[1].channel_p = AVR32_ADCIFA_INP_ADCIN2;	// FET Temp
	adcifa_sequence_conversion_opt[1].channel_n = AVR32_ADCIFA_INN_GNDANA;
	adcifa_sequence_conversion_opt[1].gain      = ADCIFA_SHG_1;
	
	/* Configure ADCIFA sequencer 0 */
	adcifa_configure_sequencer(&AVR32_ADCIFA, 0, &adcifa_sequence_opt, adcifa_sequence_conversion_opt);
	
	//LJK 2023.05.18  Internel ADC Sum Init
	//memset(m_fInternalADCSum, 0, sizeof(m_fInternalADCSum));
}