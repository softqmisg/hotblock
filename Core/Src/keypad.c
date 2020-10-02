/*
 * keypad.c
 *
 *  Created on: Sep 23, 2020
 *      Author: mehdi
 */

/*
 * joystick.c
 *
 *  Created on: Aug 15, 2020
 *      Author: mehdi
 */
#include "main.h"
#include  "keypad.h"
#include "tim.h"
/*
 * define private variable
 */
GPIO_TypeDef *key_ports[] = { KEYUP_GPIO_Port, KEYSET_GPIO_Port, KEYDN_GPIO_Port};
uint16_t key_pins[] = { KEYUP_Pin, KEYSET_Pin, KEYDN_Pin};
////////////////////////////////////////////////////////////////////////////////////////
volatile uint8_t current_state_sw[KEYS_NUM];
volatile uint8_t previous_state_sw[KEYS_NUM];
volatile uint8_t Key_shortpressed[KEYS_NUM];
volatile uint8_t Key_longpressed[KEYS_NUM];
volatile uint8_t Key_longlongpressed[KEYS_NUM];
volatile uint32_t Key_cntshort[KEYS_NUM];
volatile uint32_t Key_cntlong[KEYS_NUM];
volatile uint8_t keypad_s = keypad_initializing;

/*
 * init private variable
 */
void keypad_init(uint8_t key,uint8_t presstype) {

	keypad_s = keypad_initializing;
	////////////////////////////////////////////
	uint8_t mask = 1, index = 0;
	while (index <= 4) {
		if (mask & key) {
			if(presstype & Short_press)
			{
				Key_shortpressed[index] = 0;
				Key_cntshort[index] = 0;
				current_state_sw[index] = previous_state_sw[index] = 1;
			}
			if(presstype&Long_press)
			{
				Key_longpressed[index] = 0;
				Key_cntlong[index] = 0;
			}
			Key_longlongpressed[index] = 0;

		}
		index++;
		mask <<= 1;
	}

	////////////////////////////////////////////////////
	keypad_s = keypad_initialized;
}
/*
 * check and return:
 * @key keyALL=read all pins, KEY_...=pass its states
 * @presstype short_press long_presss no_press(not return)
 */
uint8_t keypad_read(uint8_t key, enum press_t presstype) {
	/*
	 * check keys state
	 */
	uint8_t out;
	if (key == Key_ALL) {
		for (uint8_t k = 0; k < KEYS_NUM; k++) {
			current_state_sw[k] = (uint8_t) HAL_GPIO_ReadPin(key_ports[k],
					key_pins[k]);

			if (current_state_sw[k] && !previous_state_sw[k]) {
				if ((LIMIT_SHORT_L < Key_cntshort[k])
						&& (Key_cntshort[k] < LIMIT_SHORT_H)
						&& !Key_longpressed[k]) {
					Key_shortpressed[k] = 1;
				}
				Key_cntshort[k] = 0;
			} else if (!current_state_sw[k] && previous_state_sw[k]) {
				Key_cntshort[k] = 0;
			} else if (!current_state_sw[k] && !previous_state_sw[k]) {
				Key_cntshort[k] = (uint32_t) Key_cntshort[k] + 1;
				Key_cntlong[k] = (uint32_t) Key_cntlong[k] + 1;
				if ((LIMIT_T1_L < Key_cntlong[k])
						&& (Key_cntlong[k] < LIMIT_T1_H)&& !Key_shortpressed[k]) {
					Key_longpressed[k] = 1;
					Key_cntlong[k] = 0;
				}
			}
			else if(current_state_sw[k] && previous_state_sw[k])
			{
				Key_cntlong[k]=0;
				Key_cntshort[k]=0;
			}
			previous_state_sw[k] = current_state_sw[k];
		}
		out = 0;
	} else {
		uint8_t index = 0;
		while (key) {
			key >>= 1;
			index++;
		}
		if (index) {
			if (presstype == Short_press)
				out = Key_shortpressed[index - 1];
			else if (presstype == Long_press)
				out = Key_longpressed[index - 1];
		}
	}
	return out;

}

uint8_t keypad_state() {
	return keypad_s;
}

