/*
 * SystemDiagnostic.h
 *
 * Created: 2021-11-26 AM 00:05:00
 *  Author: 4N Corporation, bgyu
 */ 


#ifndef SYSTEMDIAGNOSTIC_H_
#define SYSTEMDIAGNOSTIC_H_

void CheckSystemSafety( void );
bool CheckSdramMemory( void );
bool CheckSdramPatternTest( int iStartIndex );
bool CheckSdramPatternTest2( int iStartIndex, unsigned char ucOdd );
#endif /* SYSTEMDIAGNOSTIC_H_ */