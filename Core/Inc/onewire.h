/*
 * onewire.h
 *
 *  Created on: Sep 30, 2020
 *      Author: mehdi
 */

#ifndef INC_ONEWIRE_H_
#define INC_ONEWIRE_H_
#include"main.h"
#include <stdbool.h>
#include "tim.h"
#define	_ONEWIRE_TIMER											htim16
#define ONEWIRE_PORT	DS18B20_GPIO_Port
#define ONEWIRE_PIN		DS18B20_Pin

bool onewire_reset(void);

void onewire_write(unsigned char value);
unsigned char onewire_read(void);

#endif /* INC_ONEWIRE_H_ */
