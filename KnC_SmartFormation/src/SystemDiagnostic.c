/*
 * SystemDiagnostic.c
 *
 * Created: 2021-11-26 AM 00:05:00
 *  Author: 4N Corporation, bgyu
 */ 


#include "KnC_IncludeFile.h"


bool CheckSdramMemory( void )
{
	m_uiSystemInformation[_SdramFail] = 0;

	for( int i=0; i<7; i++ )
	{
		if ( !CheckSdramPatternTest( i ) )
		{
			return FALSE;;
		}
	}
	return TRUE;
}

bool CheckSdramPatternTest( int iStartIndex )
{
	if ( !CheckSdramPatternTest2( iStartIndex, 0 ) )
		return false;
	return CheckSdramPatternTest2( iStartIndex, 1 );
}

bool CheckSdramPatternTest2( int iStartIndex, unsigned char ucOdd )
{
	volatile static U16 u16PatternData[7] =
	{
		0xffff,
		0x0f0f,
		0xa55a,
		0x5aa5,
		0xff00,
		0x00ff,
		0xf0f0,
	};
	volatile U16 *pExternalSram;
	volatile unsigned int i;

	volatile U16 u16Data;
	volatile U16* pPatternData;
	volatile unsigned int iSize;
	volatile unsigned int iMemoryTail;

	iMemoryTail = (unsigned int)m_pSDRAM_TAIL;
	iMemoryTail &= 0xfffffff;
	iMemoryTail++;
	iMemoryTail >>= 1;
	iSize = EXTERNAL_SDRAM_SIZE_BY_WORD;
	pPatternData = &u16PatternData[iStartIndex%7];
	pExternalSram = (U16 *)EXT_SDRAM_BASE_ADDRESS;
	if ( ucOdd )
		pExternalSram++;
	for( i=0; i<EXTERNAL_SDRAM_SIZE_BY_WORD; i+=2 )
	{
		if( m_uiSdramCheckEnalbe == FALSE )	// stop
			return false;

		if ( pPatternData>&u16PatternData[6] )
			pPatternData = u16PatternData;
			
		*pExternalSram = *pPatternData;
		pExternalSram++;
		pPatternData++;
	}
	pPatternData = &u16PatternData[iStartIndex%7];
	pExternalSram = (U16 *)EXT_SDRAM_BASE_ADDRESS;
	if ( ucOdd )
		pExternalSram++;
	for( i=0; i<iMemoryTail; i+=2 )
	{
		if( m_uiSdramCheckEnalbe == FALSE )	// stop
			return false;
				
		if ( pPatternData>&u16PatternData[6] )
			pPatternData = u16PatternData;
			
		u16Data = *pExternalSram;
		if ( *pPatternData!=u16Data )
		{
			++m_uiSystemInformation[_SdramFail];
			m_uiSystemInformation[_SdramErrorAddress] = (U32)pExternalSram;
			m_uiSystemInformation[_SdramErrorData] = u16Data;
			return false;
		}
		pExternalSram++;
		pPatternData++;
	}

	return true;
}
