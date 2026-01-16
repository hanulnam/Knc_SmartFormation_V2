 #include "KnC_IncludeFile.h"

 void Init_EEP(void)
 {
	 EEP_PROTECTED;
 }

 uint8_t EEP_Write_Byte(uint16_t addr, uint8_t dat)
 {
	 I2C_Start();

	 I2C_Write_Byte(SLAVE_ADDR | I2C_CMD_WR);
	 if(I2C_Check_Ack() == I2C_NACK)
	 {
		 I2C_Stop();
		 return 0;
	 }

	 I2C_Write_Byte((uint8_t)(addr>>8));
	 if(I2C_Check_Ack() == I2C_NACK)
	 {
		 I2C_Stop();
		 return 0;
	 }

	 I2C_Write_Byte((uint8_t)(addr&0x00FF));
	 if(I2C_Check_Ack() == I2C_NACK)
	 {
		 I2C_Stop();
		 return 0;
	 }

	 I2C_Write_Byte(dat);
	 if(I2C_Check_Ack() == I2C_NACK)
	 {
		 I2C_Stop();
		 return 0;
	 }

	 I2C_Stop();
	 delay_ms(5);
	 //delay_us(5);

	 return 1;
 }

 uint8_t EEP_Read_Byte(uint16_t addr)
 {
	 uint8_t dat;

	 I2C_Start();

	 I2C_Write_Byte(SLAVE_ADDR | I2C_CMD_WR);
	 if(I2C_Check_Ack() == I2C_NACK)
	 {
		 I2C_Stop();
		 return 0;
	 }

	 I2C_Write_Byte((uint8_t)(addr>>8));
	 if(I2C_Check_Ack() == I2C_NACK)
	 {
		 I2C_Stop();
		 return 0;
	 }

	 I2C_Write_Byte((uint8_t)(addr&0x00FF));
	 if(I2C_Check_Ack() == I2C_NACK)
	 {
		 I2C_Stop();
		 return 0;
	 }

	 I2C_Start();

	 I2C_Write_Byte(SLAVE_ADDR | I2C_CMD_RD);
	 if(I2C_Check_Ack() == I2C_NACK)
	 {
		 I2C_Stop();
		 return 0;
	 }

	 dat = I2C_Read_Byte(I2C_NACK);

	 I2C_Stop();

	 return dat;
 }

 uint8_t EEP_Array_Write_Byte(uint16_t addr, uint8_t *Arr, uint8_t size)
 {
	 uint8_t i;

	 I2C_Start();

	 I2C_Write_Byte(SLAVE_ADDR | I2C_CMD_WR);
	 if(I2C_Check_Ack() == I2C_NACK)
	 {
		 I2C_Stop();
		 return 0;
	 }

	 I2C_Write_Byte((uint8_t)(addr>>8));
	 if(I2C_Check_Ack() == I2C_NACK)
	 {
		 I2C_Stop();
		 return 0;
	 }

	 I2C_Write_Byte((uint8_t)(addr&0x00FF));
	 if(I2C_Check_Ack() == I2C_NACK)
	 {
		 I2C_Stop();
		 return 0;
	 }

	 if( size > EEPROM_PAGESIZE ) size = EEPROM_PAGESIZE;
	 for(i = 0; i < size; i++)
	 {
		 I2C_Write_Byte(Arr[i]);
		 if(I2C_Check_Ack() == I2C_NACK)
		 {
			 I2C_Stop();
			 return 0;
		 }
	 }

	 I2C_Stop();

	 delay_ms(5);

	 return 1;
 }

 uint8_t EEP_Array_Read_Byte(uint16_t addr, uint8_t *Arr, uint8_t size)
 {
	 uint8_t i;

	 I2C_Start();

	 I2C_Write_Byte(SLAVE_ADDR | I2C_CMD_WR);
	 if(I2C_Check_Ack() == I2C_NACK)
	 {
		 I2C_Stop();
		 return 0;
	 }

	 I2C_Write_Byte((uint8_t)(addr>>8));
	 if(I2C_Check_Ack() == I2C_NACK)
	 {
		 I2C_Stop();
		 return 0;
	 }

	 I2C_Write_Byte((uint8_t)(addr&0x00FF));
	 if(I2C_Check_Ack() == I2C_NACK)
	 {
		 I2C_Stop();
		 return 0;
	 }

	 I2C_Start();

	 I2C_Write_Byte(SLAVE_ADDR | I2C_CMD_RD);
	 if(I2C_Check_Ack() == I2C_NACK)
	 {
		 I2C_Stop();
		 return 0;
	 }

	 if( size > EEPROM_PAGESIZE ) size = EEPROM_PAGESIZE;
	 for(i = 0; i < size - 1; i++)
	 {
		 Arr[i] = I2C_Read_Byte(I2C_ACK);
	 }

	 Arr[i] = I2C_Read_Byte(I2C_NACK);
	 I2C_Stop();

	 return 1;
 }

 uint8_t EEP_Write_Page(uint16_t page_num, uint8_t *Arr)
 {
	uint8_t ret_val;

	ret_val = EEP_Array_Write_Byte(page_num*EEPROM_PAGESIZE, Arr, EEPROM_PAGESIZE);

	return ret_val;
 }

 uint8_t EEP_Read_Page(uint16_t page_num, uint8_t *Arr)
 {
	uint8_t ret_val;

	ret_val = EEP_Array_Read_Byte(page_num*EEPROM_PAGESIZE, Arr, EEPROM_PAGESIZE);

	return ret_val;
 }
 
 uint8_t EEP_Write_Block(uint16_t addr, uint8_t *Arr, uint8_t size)
 {
	 uint8_t i, ret_val;

	 for(i=0; i<size; i++){
		 ret_val = EEP_Write_Byte(addr+i, Arr[i]);

		 if(ret_val==0)	break;
	 }
	 
	 return ret_val;
 }

 uint8_t EEP_Read_Block(uint16_t addr, uint8_t *Arr, uint8_t size)
 {
	 uint8_t i;
	 
	 for(i=0; i<size; i++){
		 Arr[i] = EEP_Read_Byte(addr+i);
	 }
	 
	 return i;
 }
 
 uint8_t EEP_Cal_Data_Write(uint8_t *Arr, uint16_t size)//EEPWriteAll
 {
	uint8_t i, ret_val;
	uint8_t start_page, page_cnt, page_mod;

	start_page = EEP_SETUP_CAL_PAGE_ADDR;
	page_cnt = (size/EEPROM_PAGESIZE);
	page_mod = (size%EEPROM_PAGESIZE);

	EEP_UNPROTECTED;
	for (i = 0; i < page_cnt; i++)
	{
		ret_val = EEP_Write_Page(start_page+i, (uint8_t *)(Arr + i * EEPROM_PAGESIZE));
	}

	ret_val = EEP_Array_Write_Byte((start_page + page_cnt) * EEPROM_PAGESIZE, (uint8_t *)(Arr + page_cnt * EEPROM_PAGESIZE), page_mod);

	EEP_PROTECTED;

	return ret_val;
 }

 uint8_t EEP_Cal_Data_Read(uint8_t *Arr, uint16_t size)
 {
	uint8_t i, ret_val;
	uint8_t start_page, page_cnt, page_mod;

	start_page = EEP_SETUP_CAL_PAGE_ADDR;
	page_cnt = (uint8_t)((size/EEPROM_PAGESIZE));
	page_mod = (size%EEPROM_PAGESIZE);

	for (i = 0; i < page_cnt; i++)
	{
		ret_val = EEP_Read_Page(start_page+i, (uint8_t *)(Arr + i * EEPROM_PAGESIZE));
	}

	if(page_mod != 0){
		ret_val = EEP_Array_Read_Byte((start_page + page_cnt) * EEPROM_PAGESIZE, (uint8_t *)(Arr + page_cnt * EEPROM_PAGESIZE), page_mod);
	}
	
	EEP_PROTECTED;
	
	return ret_val;
 }
 
 uint8_t EEP_CAL_Data_Check(uint8_t *Arr, uint16_t size)
 {
	int index = 0;
	for(index = 0 ; index < size ; index++)
	{
		if(*Arr != 0xFF)
			return TRUE;
	}
	// EEPROM Read Data is Empty(All 0xFF)
	return FALSE;
 }