/*
 * KnC_Define.h
 *
 * Created: 2021-11-26 AM 00:05:00
 *  Author: 4N Corporation, bgyu
 */ 

#ifndef KnC_DEFINE_H_
#define KnC_DEFINE_H_
#pragma pack(1)

//#define _5V_20A_LGES_	
//#define _5V_20A_3rd_Board
//#define _5V_50A_		
//#define _5V_200A_
#define _5V_350A_
#define _SOFT_AUTO_	//LJK 2025.04.28 Pulse 모드시 Boost, 일반 충방전시 Soft (테스트 필요)

#ifdef _5V_350A_
	#define MAX_SYSTEM_CURRENT			400.0f
	#define MAX_SYSTEM_VOLTAGE			5.0f
    
	#define MINIMUM_VOLTAGE				0.001f
	#define CHARGE_MINIMUM_CURREN		0.001f		//0.040f
	#define DISCHARGE_MINIMUM_CURRENT	-0.001f		//-0.040f
#endif

#ifdef _5V_200A_
	#define MAX_SYSTEM_CURRENT			200.0f
	#define MAX_SYSTEM_VOLTAGE			5.0f

	#define MINIMUM_VOLTAGE				0.001f
	#define CHARGE_MINIMUM_CURREN		0.001f		//0.040f
	#define DISCHARGE_MINIMUM_CURRENT	-0.001f		//-0.040f
#endif


#ifdef _5V_50A_
	#define MAX_SYSTEM_CURRENT			50.0f
	#define MAX_SYSTEM_VOLTAGE			5.0f
	
	#define MINIMUM_VOLTAGE				0.001f
	#define CHARGE_MINIMUM_CURREN		0.001f		//0.040f
	#define DISCHARGE_MINIMUM_CURRENT	-0.001f		//-0.040f
#endif

#ifdef _5V_20A_3rd_Board
	#define MAX_SYSTEM_CURRENT			20.0f
	#define MAX_SYSTEM_VOLTAGE			5.0f

	#define MINIMUM_VOLTAGE				0.001f
	#define CHARGE_MINIMUM_CURREN		0.001f		//0.040f
	#define DISCHARGE_MINIMUM_CURRENT	-0.001f		//-0.040f
#endif

#ifdef _5V_20A_LGES_
	#define MAX_SYSTEM_CURRENT			20.0f
	#define MAX_SYSTEM_VOLTAGE			5.0f

	#define MINIMUM_VOLTAGE				0.001f
	#define CHARGE_MINIMUM_CURREN		0.001f		//0.040f
	#define DISCHARGE_MINIMUM_CURRENT	-0.001f		//-0.040f
#endif

#define FALSE       0x0
#define TRUE        0x1

#define	PC_KPU_COMM_ETX0	'e'
#define	PC_KPU_COMM_ETX1	'N'
#define	PC_KPU_COMM_ETX2	'd'
#define	PC_KPU_COMM_ETX3	't'
#define	PC_KPU_COMM_ETX4	'X'

#define	VOLTAGE_RANGE_INDEX		0
#define	CURRENT_RANGE_INDEX		(VOLTAGE_RANGE_INDEX+_MAX_CHANNEL)
#define	VOLTAGE2_RANGE_INDEX	(CURRENT_RANGE_INDEX+_MAX_CHANNEL*2)

#define _DAC_CH1_CC				0
#define _DAC_CH1_CV				1
#define _DAC_CH2_CC				2
#define _DAC_CH2_CV				3
#define _DAC_CH3_CC				4
#define _DAC_CH3_CV				5
#define _DAC_CH4_CC				6
#define _DAC_CH4_CV				7

#define DEFAULT_HEAT_SINK_TEMP  90.0f	//2025.04.25 방열반 최대온도 90도
#define DEFAULT_AIR_TEMP	    60.0f	//2025.04.25 보드 AirTemp 최대온도 60도
#define DEFAULT_CELL_TEMP	    70.0f	//20250908 default valute 추가


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// define for adding features
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//#define MODIFY_PULSE_CPCR               //20250826 Normal CP/CR, Pulse CP/CR 오류 수정
#define MODIDY_MONITORING_DATA          //20250828 IDLE 상태에서 전류값이 표시되지 않도록 수정
#define MODIDY_PULSE_OVERSHOOT          //20250905 Pulse LI 동작 시 OverShooting 방지
#define SUPPORT_PROTECTION_CONDITION    //20250908 Alarm Protection 추가
#define MODIFY_ALARM_CONDITION          //20250909 Alarm 동작 조건 수정

#define MODIFY_DCIR_STEP_DATA           //20251216 DCIR 버그 수정
#define MODIFY_DCIR_10MSEC_MODE         //20251219 DCIR 10msec Mode는 1msec 단위로 RawData 저장

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Structure
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
union ReturnFlagUnion
{
	unsigned int uiReturn;
	int iReturn;
	float fReturn;
	U16 u16Return[2];
	U8 u8Return[4];
	U16 *pU16;
	U32 *pU32;
};

typedef union
{
	float f;
	int i;
	unsigned int ui;
	long l;
	unsigned long ul;
	char c[4];
	unsigned char uc[4];
} union32;

#pragma pack()
#endif /* KnC_DEFINE_H_ */
