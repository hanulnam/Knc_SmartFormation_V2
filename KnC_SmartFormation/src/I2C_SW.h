/*
 * I2C_SW.h
 *
 * Created: 2020-03-13 오전 8:51:01
 *  Author: LG
 */ 


#ifndef I2C_SW_H_
#define I2C_SW_H_

#define I2C_SDA					(0xB0000000|AVR32_PIN_PC13)
#define I2C_SCL					(0xB0000000|AVR32_PIN_PD29)
#define __C_I2C_SDA				(1<<13)
#define __D_I2C_SCL				(1<<29)


void I2C_SDA_OUT(void);
void I2C_SDA_IN(void);
void I2C_Start(void);
void I2C_Stop(void);
uint8_t I2C_Check_Ack(void);
void I2C_Send_Ack(void);
void I2C_Send_Nack(void);
void I2C_Write_Byte(uint8_t dat);
uint8_t I2C_Read_Byte(uint8_t ack);

#endif /* I2C_SW_H_ */