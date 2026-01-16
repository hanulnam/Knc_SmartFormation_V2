#ifndef EEPROM_H_
#define EEPROM_H_

//OLED EEP
#define I2C_WP					(0xB0000000|AVR32_PIN_PC05)
#define __C_I2C_WP				(1<<5)

#define EEP_PROTECTED			PDC_HIGH = __C_I2C_WP
#define EEP_UNPROTECTED			PDC_LOW = __C_I2C_WP

#define I2C_CMD_WR			0
#define I2C_CMD_RD			1

#define I2C_ACK				0
#define	I2C_NACK			1

//#define EEPROM_24LC256_		
#define EEPROM_24LC512_		

#ifdef	EEPROM_24LC256_
	#define SLAVE_ADDR			0xA0
	#define EEPROM_PAGESIZE		64

	#define EEP_SETUP_CAL_PAGE_ADDR			2
	#define EEP_SETUP_CAL_BASE_ADDR			(EEPROM_PAGESIZE*EEP_SETUP_CAL_PAGE_ADDR)
#endif

//24LC512  LJK 2023.04.10
#ifdef	EEPROM_24LC512_
	#define SLAVE_ADDR			0xA0
	#define EEPROM_PAGESIZE		128

	#define EEP_SETUP_CAL_PAGE_ADDR			1
	#define EEP_SETUP_CAL_BASE_ADDR			(EEPROM_PAGESIZE*EEP_SETUP_CAL_PAGE_ADDR)

	#define EEP_STEP_SEQUENCE_PAGE_ADDR		32
	#define EEP_STEP_SEQUENCE_BASE_ADDR		(EEPROM_PAGESIZE*EEP_STEP_SEQUENCE_PAGE_ADDR)
#endif


void Init_EEP(void);
uint8_t EEP_Write_Byte(uint16_t addr, uint8_t dat);
uint8_t EEP_Read_Byte(uint16_t addr);
uint8_t EEP_Array_Write_Byte(uint16_t addr, uint8_t *Arr, uint8_t size);
uint8_t EEP_Array_Read_Byte(uint16_t addr, uint8_t *Arr, uint8_t size);
uint8_t EEP_Write_Page(uint16_t page_num, uint8_t *Arr);
uint8_t EEP_Read_Page(uint16_t page_num, uint8_t *Arr);
uint8_t EEP_Write_Block(uint16_t addr, uint8_t *Arr, uint8_t size);
uint8_t EEP_Read_Block(uint16_t addr, uint8_t *Arr, uint8_t size);
uint8_t EEP_Cal_Data_Write(uint8_t *Arr, uint16_t size);
uint8_t EEP_Cal_Data_Read(uint8_t *Arr, uint16_t size);
uint8_t EEP_CAL_Data_Check(uint8_t *Arr, uint16_t size);
//int EEP_Cal_Data_Write_ST(uint8_t CH, uint8_t Kind, int Point);
//int EEP_Cal_Data_Read_ST(uint8_t CH, uint8_t Kind, int Point);

#endif /* INCFILE1_H_ */