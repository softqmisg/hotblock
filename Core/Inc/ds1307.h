#ifndef _DS13_H
#define _DS13_H
#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"
#include "i2c.h"

#define DS1307_I2C	hi2c1
#define FALSE											0
#define TRUE											1

typedef struct
{
	uint8_t year;
	uint8_t month;
	uint8_t date;
	uint8_t day;

}Date_t;
typedef struct{
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
} Time_t;
unsigned char DS1307Read(unsigned char address,unsigned char *data);
unsigned char DS1307Write(unsigned char address,unsigned char *data);
//unsigned char Chek_M_Kabise(unsigned int);
//unsigned char Chek_Sh_Kabise(unsigned int);
//void Shamsi_To_Miladi(unsigned int ,unsigned char ,unsigned char );
//void Miladi_To_Shamsi(unsigned int ,unsigned char ,unsigned char );
uint8_t DS1307_Init();
void Get_Time_Date(Time_t *time,Date_t *date);
void Set_Time(Time_t time);
void Set_Date(Date_t date);
uint8_t gLeap(uint16_t year);
extern uint8_t gDaysInMonth[];
#ifdef __cplusplus
}
#endif
#endif
