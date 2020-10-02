/*
 * onewire.c
 *
 *  Created on: Sep 30, 2020
 *      Author: mehdi
 */

#include "onewire.h"
void ONEWIRE_DELAY(uint16_t time_us)
{
	_ONEWIRE_TIMER.Instance->CNT = 0;
	while(_ONEWIRE_TIMER.Instance->CNT <= time_us);
}
void ONEWIRE_LOW(void)
{
	HAL_GPIO_WritePin(ONEWIRE_PORT, ONEWIRE_PIN, GPIO_PIN_RESET);
}
void ONEWIRE_HIGH(void)
{
	HAL_GPIO_WritePin(ONEWIRE_PORT, ONEWIRE_PIN, GPIO_PIN_SET);
}
void ONEWIRE_INPUT(void)
{
	GPIO_InitTypeDef	gpinit;
	gpinit.Mode = GPIO_MODE_INPUT;
	gpinit.Pull = GPIO_NOPULL;
	gpinit.Speed = GPIO_SPEED_FREQ_HIGH;
	gpinit.Pin = ONEWIRE_PIN;
	HAL_GPIO_Init(ONEWIRE_PORT,&gpinit);
}
void ONEWIRE_OUTPUT(void)
{
	GPIO_InitTypeDef	gpinit;
	gpinit.Mode = GPIO_MODE_OUTPUT_OD;
	gpinit.Pull = GPIO_NOPULL;
	gpinit.Speed = GPIO_SPEED_FREQ_HIGH;
	gpinit.Pin = ONEWIRE_PIN;
	HAL_GPIO_Init(ONEWIRE_PORT,&gpinit);

}

////////////////////////////////////////////////////////////
bool onewire_reset(void)
{
	bool res = false;
	ONEWIRE_OUTPUT();
	ONEWIRE_LOW();
	ONEWIRE_DELAY(800); //480
	ONEWIRE_INPUT();
	ONEWIRE_DELAY(60);
	res = HAL_GPIO_ReadPin(ONEWIRE_PORT, ONEWIRE_PIN);
	ONEWIRE_DELAY(800); //480
	return res;

}
////////////////////////////////////////////////////////////
void onewire_write(unsigned char value)
{
	unsigned char s=0x01;
 	ONEWIRE_INPUT();
	while(s)
	{
		ONEWIRE_OUTPUT();
		ONEWIRE_LOW();
		if(value&s)
		{
				ONEWIRE_DELAY(1);
				ONEWIRE_INPUT();
				ONEWIRE_DELAY(80);
		}
		else
		{
				ONEWIRE_DELAY(80);//60 us<TX “0”<120 us
				ONEWIRE_INPUT();
				ONEWIRE_DELAY(5);//1 us< tREC <...
		}
		s<<=1;

	}
}
////////////////////////////////////////////////////////////
unsigned char onewire_read(void)
{
	uint8_t s=0x01;
	unsigned char value = 0x00;
	while(s)
	{
		ONEWIRE_OUTPUT();
		ONEWIRE_LOW();
		ONEWIRE_DELAY(1);
		ONEWIRE_INPUT();
		if( HAL_GPIO_ReadPin(ONEWIRE_PORT, ONEWIRE_PIN))
			value|=s;

		s<<=1;
		ONEWIRE_DELAY(60);
	}

	ONEWIRE_DELAY(50);
	return value;
}
////////////////////////////////////////////////////////////

