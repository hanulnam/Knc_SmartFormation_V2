/*
 * __KoYongBoard.h
 *
 * Created: 2021-11-26 AM 00:05:00
 *  Author: 4N Corporation, bgyu
 */ 


#ifndef _KnC_BOARD_H_
#define _KnC_BOARD_H_

#define FOSC32					AVR32_SCIF_OSC32_FREQUENCY
#define OSC32_STARTUP			AVR32_SCIF_OSCCTRL32_STARTUP_8192_RCOSC

#define FOSC0					12000000		//72MHz                          
#define OSC0_STARTUP			AVR32_SCIF_OSCCTRL0_STARTUP_2048_RCOSC

#define BOARD_OSC0_HZ			12000000		//72MHz
#define BOARD_OSC0_STARTUP_US   2000

#define BOARD_OSC0_IS_XTAL      false

# if BOARD_OSC0_IS_XTAL == false
		#if BOARD_OSC0_HZ < 2000000
			#define OSC0_GAIN_VALUE      AVR32_SCIF_OSCCTRL0_GAIN_G0
		# elif BOARD_OSC0_HZ < 10000000
			#define OSC0_GAIN_VALUE      AVR32_SCIF_OSCCTRL0_GAIN_G1
		#elif BOARD_OSC0_HZ < 160000000
			#define OSC0_GAIN_VALUE      AVR32_SCIF_OSCCTRL0_GAIN_G2
		#else
			#define OSC0_GAIN_VALUE      AVR32_SCIF_OSCCTRL0_GAIN_G3
		#endif
#endif

#define BOARD_OSC32_HZ          32768
#define BOARD_OSC32_STARTUP_US  71000
#define BOARD_OSC32_IS_XTAL     true

#endif /* _KnC_BOARD_H_ */
