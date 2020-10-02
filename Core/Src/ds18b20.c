/*
 * ds18b20.c
 *
 *  Created on: Sep 30, 2020
 *      Author: mehdi
 */
#include "onewire.h"
#include "ds18b20.h"
#include <stdbool.h>
#include "iwdg.h"
////////////////////////////////////////////////////////////
void DS18B20_Init(void) {
	HAL_TIM_Base_Start(&_ONEWIRE_TIMER);

	ONEWIRE_INPUT();
	ONEWIRE_DELAY(10); //480
	while (onewire_reset()) {
#ifdef IWDG_ENABLE
		HAL_IWDG_Refresh(&hiwdg);
#endif
	} //delay_ms(100);
	onewire_write(skip_ROM);
	onewire_write(write_scratchpad);
	onewire_write(0x00); //TH
	onewire_write(0x00); //TL
	onewire_write(0x7F); //config: 0 R1 R0 1 1 1 1 1 ,R1R0:00=>9bit,01=>10bit,10=>11bit,11=>12bit
	ONEWIRE_DELAY(200);

}
////////////////////////////////////////////////////////////
bool DS18B20_get_temperature(Ds18b20Sensor_t *ds18b20) {

	bool minus = 0;
	uint16_t decimal;
	uint8_t lsb, msb;
	double temp;
	if (!onewire_reset()) {

		//onewire_reset();
		onewire_write(skip_ROM);
		onewire_write(convert_T);

		HAL_Delay(800);

		if (!onewire_reset()) {
			onewire_write(skip_ROM);
			onewire_write(read_scratchpad);
			lsb = onewire_read();
			ONEWIRE_DELAY(10);
			msb = onewire_read();
			decimal = ((uint16_t) msb << 8);
			decimal += (uint16_t) lsb;

			if (msb & 0x80) // negative
					{
				decimal = ~(uint16_t) decimal + 1;
				minus = 1;
			}

			temp = (double) decimal * 0.0625;

			if (minus)
				temp = 0.0 - (double) temp;

			ds18b20->Temperature = (double) temp;
			ds18b20->DataIsValid = true;
			return true; // temp is multiplied by 10
		}
	}
	ds18b20->DataIsValid = false;
	return false;
}
