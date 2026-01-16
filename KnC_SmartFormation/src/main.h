/*
 * main.h
 *
 * Created: 2021-11-26 AM 00:05:00
 *  Author: 4N Corporation, bgyu
 */ 


#ifndef MAIN_H_
#define MAIN_H_

int	 main( void );
void DummyDebugFunction( void );
void DummyFunctionForMain( void );
void EEPRom_Write_Control( void );
void EEPRom_Read_Control( void );
void LoadMcuNumber( void );
void Kpu_B_FatalError(			uint32_t r12,				uint32_t r11,	uint32_t r10,	uint32_t r9,
uint32_t excpiton_number,	uint32_t lr,	uint32_t r17,	uint32_t r6,
uint32_t r5,				uint32_t r4,	uint32_t r3,	uint32_t r2,		uint32_t r1,
uint32_t r0,				uint32_t sp,	uint32_t pc,	uint32_t stack0,	uint32_t stack1,
uint32_t stack2 ); //212223
void DacRest( void );
void ChargeDirectTest( void );
void DischargeDirectTest( void );
void DummyOn( void );
void ChargeDischargeDirectTest( void );
void EEPRomDummySet( void );
void memcpy2( char *pDest, char *pSource, unsigned char ucSize );
void ChargeCvTest( void );
void ChargeCvTest2( void );
void CCCV_CP_Test( void );
void Dac2Initialize ( void );
#ifdef SUPPORT_BLACK_OUT
void ReadPauseInfoData ( void );
void SetPauseInfoData ( unsigned char ucCh );
#endif
#endif /* MAIN_H_ */