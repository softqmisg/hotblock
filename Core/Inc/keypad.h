/*
 * keypad.h
 *
 *  Created on: Sep 23, 2020
 *      Author: mehdi
 */

#ifndef INC_KEYPAD_H_
#define INC_KEYPAD_H_
#include "tim.h"
#define KEYPAD_TIMER		TIM3
#define KEYS_NUM			3
/*
 * timing
 */
#define KEY_REFRESH_HZ		1000 //Hz
#define MS_COUNTER			1//1000/KEY_REFRESH_HZ

////short press ~0.01s->0.4s
#define LIMIT_SHORT_L	10/MS_COUNTER//10ms
#define LIMIT_SHORT_H	400/MS_COUNTER//400ms
/////about 3sec
#define LIMIT_T1_L 1500/MS_COUNTER///1.5s
#define LIMIT_T1_H 3500/MS_COUNTER//3.5
//////about >7s
#define LIMIT_T2_L 5000/MS_COUNTER//5s
#define LIMIT_T2_H 10000/MS_COUNTER//10s

enum {
	Key_No=0x00,
	Key_UP = 0x01,
	Key_SET = 0x02,
	Key_DOWN = 0x04,
	Key_ALL = Key_UP|Key_SET|Key_DOWN
};
//enum{
//	KNum_UP=0,
//	KNum_SET=1,
//	KNum_DOWN=2,
//
//};
enum press_t {
	no_press=0x00, Short_press = 0x01, Long_press = 0x02,Both_press=Short_press|Long_press
};

enum {
	keypad_initializing = 0, keypad_initialized = 1
};
void keypad_init(uint8_t key,uint8_t presstype);
uint8_t keypad_state();
uint8_t keypad_read(uint8_t key, uint8_t presstype);



#endif /* INC_KEYPAD_H_ */
