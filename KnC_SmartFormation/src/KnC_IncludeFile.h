/*
 * KnC_IncludeFile.h
 *
 * Created: 2021-11-26 AM 00:05:00
 *  Author: 4N Corporation, bgyu
 */ 


#ifndef KnC_INCLUDEFILE_H_
#define KnC_INCLUDEFILE_H_


// Include header files for all drivers that have been imported from Atmel Software Framework (ASF).
#include <asf.h>
#include <avr32/io.h>
#include <string.h>
//#include <sys/reent.h>
#include <stdarg.h>			//va_list .. //del
#include <stdio.h>			//vsprintf //del
//#include <flashc.h>
#include <setjmp.h> //exception jump
#include <math.h>	//LJK 2023.05.17 NTC Calc
#include "compiler.h"
#include "preprocessor.h"
#include "gpio.h"
//#include "eic.h"
//#include "pdca.h"
//#include "user_board.h"


// 4N Include Files
#include "KnC_Define.h"
#include "_CommunicationStructure.h"

#include "_Sdarm_AS4C16M16S.h"
#include "KnC_AddressDataOptions.h"




#include "KpuSystemConfig.h"				//CpuCommunication

#include "CommunicationService.h"			//SerialCommunication
#include "SystemControl.h"					//HwControl
#include "SequenceManager.h"				// ??
#include "GlobalData.h"			//GlobalVariable
#include "SystemDiagnostic.h"				//Diagnostic
#include "EEPROM.h"
#include "I2C_SW.h"

#include "main.h"



#endif /* KnC_INCLUDEFILE_H_ */
