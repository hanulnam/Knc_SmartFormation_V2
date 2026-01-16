/*
 * KnC_AddressDataOptions.h
 *
 * Created: 2021-11-26 AM 00:05:00
 *  Author: 4N Corporation, bgyu
 */ 


#ifndef KnC_ADDRESSDATAOPTIONS_H_
#define KnC_ADDRESSDATAOPTIONS_H_

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Address : SDRAM 
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define EXT_SDRAM_BASE								((unsigned int)AVR32_EBI_CS1_0_ADDRESS)
#define EXT_SDRAM_BASE_ADDRESS						((volatile unsigned char*)AVR32_EBI_CS1_0_ADDRESS)

#define	EXTERNAL_SDRAM_SIZE_BY_WORD				(SDRAM_SIZE/2)

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Timer Counter
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define CAPTAIN_TIMER_COUNT_CHANNEL				0
#define CAPTAIN_TIMER_COUNT_ADDRESS				(&AVR32_TC0)
#define CAPTAIN_TIMER_COUNT_IRQ					AVR32_TC0_IRQ0

#define ADC_READ_CYCLE_TIMER_COUNT_CHANNEL		1
#define ADC_READ_CYCLE_TIMER_COUNT_ADDRESS		(&AVR32_TC1)
#define ADC_READ_CYCLE_TIMER_COUNT_IRQ			AVR32_TC1_IRQ1

#define PC_COMM_USART					(&AVR32_USART1)
#define PC_COMM_USART_RX_PIN			AVR32_USART1_RXD_PIN
#define PC_COMM_USART_RX_FUNCTION		AVR32_USART1_RXD_FUNCTION
#define PC_COMM_USART_TX_PIN			AVR32_USART1_TXD_PIN
#define PC_COMM_USART_TX_FUNCTION		AVR32_USART1_TXD_FUNCTION
#define PC_COMM_USART_RTS_PIN			AVR32_USART1_RTS_PIN
#define PC_COMM_USART_RTS_FUNCTION		AVR32_USART1_RTS_FUNCTION

#define PC_COMM_USART_IRQ				AVR32_USART1_IRQ
#define PC_COMM_USART_BAUDRATE			230400 

#define PC_COMM_RX_PDCA_BUFFER_SIZE	512 
#define PC_COMM_TX_PDCA_BUFFER_SIZE	240

#define PC_COMM_RX_PDCA_CHANNEL_USART	0
#define PC_COMM_RX_PDCA				AVR32_PDCA.channel[PC_COMM_RX_PDCA_CHANNEL_USART]
#define PC_COMM_TX_PDCA_CHANNEL_USART	1
#define PC_COMM_TX_PDCA				AVR32_PDCA.channel[PC_COMM_TX_PDCA_CHANNEL_USART]


#define DAC_SPI                 (&AVR32_SPI1)
#define DAC_SPI_NPCS            0
#define DAC_SPI_SCK_PIN         AVR32_SPI1_SCK_PIN
#define DAC_SPI_SCK_FUNCTION    AVR32_SPI1_SCK_FUNCTION
#define DAC_SPI_MISO_PIN        AVR32_SPI1_MISO_PIN
#define DAC_SPI_MISO_FUNCTION   AVR32_SPI1_MISO_FUNCTION
#define DAC_SPI_MOSI_PIN        AVR32_SPI1_MOSI_PIN
#define DAC_SPI_MOSI_FUNCTION   AVR32_SPI1_MOSI_FUNCTION
#define DAC_SPI_NPCS0_PIN       AVR32_SPI1_NPCS_0_PIN
#define DAC_SPI_NPCS0_FUNCTION  AVR32_SPI1_NPCS_0_FUNCTION

#define DAC_SPI_BAUDRATE		36000000

#define ADC_SPI                 (&AVR32_SPI0)
#define ADC_SPI_NPCS            0
#define ADC_SPI_SCK_PIN         AVR32_SPI0_SCK_2_PIN
#define ADC_SPI_SCK_FUNCTION    AVR32_SPI0_SCK_2_FUNCTION
#define ADC_SPI_MISO_PIN        AVR32_SPI0_MISO_2_PIN
#define ADC_SPI_MISO_FUNCTION   AVR32_SPI0_MISO_2_FUNCTION
#define ADC_SPI_MOSI_PIN        AVR32_SPI0_MOSI_2_PIN
#define ADC_SPI_MOSI_FUNCTION   AVR32_SPI0_MOSI_2_FUNCTION
#define ADC_SPI_NPCS0_PIN       AVR32_SPI0_NPCS_0_2_PIN
#define ADC_SPI_NPCS0_FUNCTION  AVR32_SPI0_NPCS_0_2_FUNCTION

#define ADC_SPI_BAUDRATE		24000000

#endif /* KnC_ADDRESSDATAOPTIONS_H_ */
