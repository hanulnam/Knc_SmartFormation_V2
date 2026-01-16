/*
 * _KnC_ConfigClock.h
 *
 * Created: 2021-11-26 AM 00:05:00
 *  Author: 4N Corporation, bgyu
 */ 


#ifndef _KnC_CONFIGCLOCK_H_
#define _KnC_CONFIGCLOCK_H_

#define CONFIG_SYSCLK_SOURCE			SYSCLK_SRC_PLL0

#define CONFIG_PLL0_SOURCE				PLL_SRC_OSC0
//#define CONFIG_PLL0_MUL				7					//84MHz
#define CONFIG_PLL0_MUL					12					//72MHz
#define CONFIG_PLL0_DIV					2					//84Mhz DIV 1 (7*12/1),  72Mhz DIV 2 (12*12/2)

#define CONFIG_SYSCLK_CPU_DIV			0
#define CONFIG_SYSCLK_PBA_DIV			0
#define CONFIG_SYSCLK_PBB_DIV			0
#define CONFIG_SYSCLK_PBC_DIV			0

#endif /* _KnC_CONFIGCLOCK_H_ */
