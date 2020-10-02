/*
 * ds18b20.h
 *
 *  Created on: Sep 30, 2020
 *      Author: mehdi
 */

#ifndef INC_DS18B20_H_
#define INC_DS18B20_H_
#include <stdbool.h>
	typedef struct {
	uint8_t Address[8];
	double Temperature;
	bool DataIsValid;

	} Ds18b20Sensor_t;

	#define DS18B20
	#define RES12
	#define convert_T 0x44
	#define read_scratchpad 0xBE
	#define write_scratchpad 0x4E
//	#define copy_scratchpad 0x48
//	#define recall_E2 0xB8
//	#define read_power_supply 0xB4
	#define skip_ROM 0xCC
	//#define read_ROM	0x33
//	#define resolution 12

	void DS18B20_Init(void);
	bool DS18B20_get_temperature(Ds18b20Sensor_t *ds18b20) ;
//	void DS18B20_Read_ROM(void);
//	extern uint8_t ds18b20_rom[8];

#endif /* INC_DS18B20_H_ */
