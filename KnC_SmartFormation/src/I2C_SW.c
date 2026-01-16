#include "KnC_IncludeFile.h"

 void I2C_SDA_OUT(void)
 {
	gpio_configure_pin(I2C_SDA, GPIO_DIR_OUTPUT | GPIO_DRIVE_MIN);
 }

 void I2C_SDA_IN(void)
 {
	gpio_configure_pin(I2C_SDA, GPIO_DIR_INPUT | GPIO_DRIVE_MIN);
 }

 void I2C_Start(void)
 {
	PDC_HIGH = __C_I2C_SDA;
	delay_us(1);
	PDD_HIGH = __D_I2C_SCL;;
	delay_us(1);
	PDC_LOW = __C_I2C_SDA;
	delay_us(1);
	PDD_LOW = __D_I2C_SCL;
	delay_us(1);
 }

 void I2C_Stop(void)
 {
	PDC_LOW = __C_I2C_SDA;
	delay_us(1);
	PDD_HIGH = __D_I2C_SCL;
	delay_us(1);
	PDC_HIGH = __C_I2C_SDA;
	delay_us(1);
 }

 uint8_t I2C_Check_Ack(void)
 {
	uint16_t Time_out;

	PDD_LOW = __D_I2C_SCL;
	PDC_HIGH = __C_I2C_SDA;
	I2C_SDA_IN();
	delay_us(1);

	PDD_HIGH = __D_I2C_SCL;
	delay_us(1);
	
	Time_out = 0;
	while(gpio_get_pin_value(I2C_SDA & 0xFFF))
	{
		Time_out++;
		if(Time_out>500)
		{
			I2C_Stop();
			return 1;
		}else{
			break;
		}
	};

	PDD_LOW = __D_I2C_SCL;
	delay_us(1);

	PDC_LOW = __C_I2C_SDA;
	I2C_SDA_OUT();

	return 0;
 }

 void I2C_Send_Ack(void)
 {
	PDD_LOW = __D_I2C_SCL;
	PDC_LOW = __C_I2C_SDA;
	delay_us(1);
	PDD_HIGH = __D_I2C_SCL;
	delay_us(1);
	PDD_LOW = __D_I2C_SCL;
 }

 void I2C_Send_Nack(void)
 {
	 PDD_LOW = __D_I2C_SCL;
	 PDC_HIGH = __C_I2C_SDA;
	 delay_us(1);
	 PDD_HIGH = __D_I2C_SCL;
	 delay_us(1);
	 PDD_LOW = __D_I2C_SCL;
 }

 void I2C_Write_Byte(uint8_t dat)
 {
	uint8_t i;

	I2C_SDA_OUT();
	for(i = 0; i < 8; i++)
	{
		PDD_LOW = __D_I2C_SCL;
		delay_us(1);

		if((dat & 0x80) == 0x80)
		{
			PDC_HIGH = __C_I2C_SDA;
		}
		else
		{
			PDC_LOW = __C_I2C_SDA;
		}
		dat <<= 1;
		delay_us(1);

		PDD_HIGH = __D_I2C_SCL;
		delay_us(1);
		delay_us(1);
	}
 }

 uint8_t I2C_Read_Byte(uint8_t ack)
 {
	uint8_t i, dat = 0;

	I2C_SDA_IN();
	PDC_HIGH = __C_I2C_SDA;
	for(i = 0; i < 8; i++)
	{
		PDD_LOW = __D_I2C_SCL;
		delay_us(1);
		PDD_HIGH = __D_I2C_SCL;
		delay_us(1);
		dat += gpio_get_pin_value(I2C_SDA & 0xFFF) << (7 - i);
		delay_us(1);
		PDD_LOW = __D_I2C_SCL;
		delay_us(1);
	}
	PDD_LOW = __D_I2C_SCL;
	I2C_SDA_OUT();

	if(ack == 0){
		I2C_Send_Ack();
	}
	else{
		I2C_Send_Nack();
	}

	return dat;
 }
