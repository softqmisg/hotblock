/************************************************************

*
* ds13.cpp
* Author: Mohammad Hadi Dashti
*

************************************************************/

#include "ds1307.h"
#include "i2c.h"
#define CH 7

uint8_t gDaysInMonth[]={31,28,31,30,31,30,31,31,30,31,30,31};
/*
 * georgian leap: must be like 2020
 */
uint8_t gLeap(uint16_t year)
{
		return (year%4==0 && ((year %100!=0) || (year%400==0)))?1:0;
}

unsigned char DS1307Read(unsigned char address,unsigned char *data)
{
	unsigned char res;	//result
	res=HAL_I2C_Mem_Read(&DS1307_I2C, 0xD0, address, 1, data, 1, 100);
	//res=HAL_I2C_Master_Transmit(&DS1307_I2C, 0xD0, &address, 1, 100);
	//res|=(HAL_I2C_Master_Receive(&DS1307_I2C, 0XD0, data, 1, 100));
	if(res)
		return FALSE;
	else
		return TRUE;
}



unsigned char DS1307Write(unsigned char address,unsigned char *data)
{
	unsigned char res;	//result
	res= HAL_I2C_Mem_Write(&DS1307_I2C, 0xD0, address, 1, data, 1, 100);
	if(res)
		return FALSE;
	else
		return TRUE;
}

uint8_t DS1307_Init()
{
	uint8_t res;
	unsigned char temp,H_nible,L_nible;
	////////// sec
	res=DS1307Read(0x00,&temp);

	temp&=(0x7F);
	H_nible=((temp>>4)&0x0F)*10;
	L_nible=temp&0x0F;
	if(H_nible+L_nible>=60)temp=0;
	res|=DS1307Write(0x00,&temp);
	res|=DS1307Write(0x07, 0x00);
	return res;
}
/*
void Init_Time_Date(void)
{
	unsigned char temp,H_nible,L_nible;
	////////// sec
	DS1307Read(0x00,&temp);

	temp&=(~(1<<CH));
	H_nible=((temp>>4)&0x0F)*10;
	L_nible=temp&0x0F;
	if(H_nible+L_nible>=60)temp=0;
	DS1307Write(0x00,&temp);

	///////////// min
	DS1307Read(0x01,&temp);

	H_nible=((temp>>4)&0x0F)*10;
	L_nible=temp&0x0F;
	if(H_nible+L_nible>=60)temp=0;
	//else temp=0x52;
	DS1307Write(0x01,&temp);
	//////////////Hou
	DS1307Read(0x02,&temp);

	H_nible=((temp>>4)&0x0F)*10;
	L_nible=temp&0x0F;
	if(H_nible+L_nible>=24)temp=0;
	//else temp=0x10;
	DS1307Write(0x02,&temp);
	//////////////Day
	DS1307Read(0x03,&temp);

	H_nible=((temp>>4)&0x0F)*10;
	L_nible=temp&0x0F;
	if((H_nible+L_nible>=8) ||(H_nible+L_nible==0) )temp=1;
	DS1307Write(0x03,&temp);
	//////////////Date
	DS1307Read(0x04,&temp);

	H_nible=((temp>>4)&0x0F)*10;
	L_nible=temp&0x0F;
	if((H_nible+L_nible>=32)||(H_nible+L_nible==0))temp=1;
	//else temp=0x11;
	DS1307Write(0x04,&temp);
	
	/////////Mon
	DS1307Read(0x05,&temp);

	H_nible=((temp>>4)&0x0F)*10;
	L_nible=temp&0x0F;
	if((H_nible+L_nible>=13)||(H_nible+L_nible==0))temp=1;
	//else temp=0x12;
	DS1307Write(0x05,&temp);
	
	////////////Year
	DS1307Read(0x06,&temp);

	H_nible=((temp>>4)&0x0F)*10;
	L_nible=temp&0x0F;
	if((H_nible+L_nible>=100)||(H_nible+L_nible==0))temp=1;
	//else temp=0x14;
	DS1307Write(0x06,&temp);
	
	///////////Config
	DS1307Read(0x07,&temp);
	
	temp=0x10;
	DS1307Write(0x07,&temp);
}
*/
void Get_Time_Date(Time_t *time,Date_t *Date)
{
	//Now Read and format time
	unsigned char data,L_nible,H_nible;
	DS1307Read(0x00,&data);
	H_nible=((data>>4)&0x0F)*10;
	L_nible=data&0x0F;
	time->second=H_nible+L_nible;
	
	DS1307Read(0x01,&data);
	H_nible=((data>>4)&0x0F)*10;
	L_nible=data&0x0F;
	time->minute=H_nible+L_nible;

	
	DS1307Read(0x02,&data);
	H_nible=((data>>4)&0x0F)*10;
	L_nible=data&0x0F;
	time->hour=H_nible+L_nible;

	DS1307Read(0x03,&data);
	H_nible=((data>>4)&0x0F)*10;
	L_nible=data&0x0F;
	Date->day=H_nible+L_nible;

	DS1307Read(0x04,&data);
	H_nible=((data>>4)&0x0F)*10;
	L_nible=data&0x0F;
	Date->date=H_nible+L_nible;
		
	DS1307Read(0x05,&data);
	H_nible=((data>>4)&0x0F)*10;
	L_nible=data&0x0F;
	Date->month=H_nible+L_nible;
		
	DS1307Read(0x06,&data);
	H_nible=((data>>4)&0x0F)*10;
	L_nible=data&0x0F;
	Date->year=H_nible+L_nible;

}
	

void Set_Time(Time_t time)
{
	unsigned char data;
	data=(time.second/10<<4)+(time.second%10);
	DS1307Write(0x00,&data);
	
	data=(time.minute/10<<4)+(time.minute%10);
	DS1307Write(0x01,&data);
	
	data=(time.hour/10<<4)+(time.hour%10);
	DS1307Write(0x02,&data);
}

void Set_Date(Date_t date)
{
	unsigned char data;
	DS1307Write(0x03,&date.day);
	
	data=(date.date/10<<4)+(date.date%10);
	DS1307Write(0x04,&data);
	
	data=(date.month/10<<4)+(date.month%10);
	DS1307Write(0x05,&data);

	data=(date.year/10<<4)+(date.year%10);
	DS1307Write(0x06,&data);
}
