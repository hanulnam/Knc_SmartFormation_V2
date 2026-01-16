/*
 * CommunicationService.h
 *
 * Created: 2021-11-26 AM 00:05:00
 *  Author: 4N Corporation, bgyu
 */ 


#ifndef COMMUNICATIONSERVICE_H_
#define COMMUNICATIONSERVICE_H_

void PcCommunicationService( void );
bool SetMcuGpioData( unsigned int uiAddress, U16 usData );
int  GetMcuGpioData( unsigned int uiAddress );
void InterpretPcCommunication( int iReceiveCount );
void StepSequence_ClearLoopCounterAll( unsigned char ucCh );
#ifdef SUPPORT_BLACK_OUT
void PauseResumeHandler (unsigned char ucChannel, unsigned char param);
#endif

#define DAC_SPI_WAIT_LOOPN	1000

#endif /* COMMUNICATIONSERVICE_H_ */