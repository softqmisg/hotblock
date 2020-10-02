/*
 * lcd_ch.h
 *
 *  Created on: Sep 17, 2019
 *      Author: mehdi
 */

#ifndef LCD_CH_H_
#define LCD_CH_H_
#include "main.h"
#include "tim.h"
#define LCDDB4_PIN	0
#define _LCD_TIMER htim16
#define BKL_TIMER htim3
#define BLK_CHANNEL	TIM_CHANNEL_1

#define LCD16X2_CLEAR_DISPLAY					0x01
#define LCD16X2_CURSOR_HOME						0x02
#define LCD16X2_CHARACTER_ENTRY_MODE	0x04
#define LCD16X2_DISPLAY_CURSOR_ON_OFF	0x08
#define LCD16X2_DISPLAY_CURSOR_SHIFT 	0x10
#define LCD16X2_FUNCTION_SET					0x20
#define LCD16X2_SET_CGRAM_ADDRESS	 		0x40
#define LCD16X2_SET_DDRAM_ADDRESS	 		0x80

/* Character entry mode instructions */
#define LCD16X2_INCREMENT							0x02	// Initialization setting
#define LCD16X2_DECREMENT							0x00
#define LCD16X2_DISPLAY_SHIFT_ON			0x01
#define LCD16X2_DISPLAY_SHIFT_OFF			0x00	// Initialization setting
/* Display cursor on off instructions */
#define LCD16X2_DISPLAY_ON	 					0x04
#define LCD16X2_DISPLAY_OFF	 					0x00	// Initialization setting
#define LCD16X2_CURSOR_UNDERLINE_ON	 	0x02
#define LCD16X2_CURSOR_UNDERLINE_OFF	0x00	// Initialization setting
#define LCD16X2_CURSOR_BLINK_ON	 			0x01
#define LCD16X2_CURSOR_BLINK_OFF	 		0x00	// Initialization setting
/* Display cursor shift instructions */
#define LCD16X2_DISPLAY_SHIFT					0x08
#define LCD16X2_CURSOR_MOVE						0x00
#define LCD16X2_RIGHT_SHIFT						0x04
#define LCD16X2_LEFT_SHIFT						0x00
/* Function set instructions */
#define LCD16X2_8BIT_INTERFACE				0x10	// Initialization setting
#define LCD16X2_4BIT_INTERFACE				0x00
#define LCD16X2_2LINE_MODE						0x08
#define LCD16X2_1LINE_MODE						0x00	// Initialization setting
#define LCD16X2_5X10DOT_FORMAT				0x04
#define LCD16X2_5X7DOT_FORMAT					0x00	// Initialization setting
/* Busy flag bit location */
#define LCD16X2_BUSY_FLAG							0x80

/** LCD display and cursor attributes --------------------------------------- */
#define LCD16X2_DISPLAY_OFF_CURSOR_OFF_BLINK_OFF	(LCD16X2_DISPLAY_OFF | \
	LCD16X2_CURSOR_UNDERLINE_OFF | LCD16X2_CURSOR_BLINK_OFF)
#define LCD16X2_DISPLAY_ON_CURSOR_OFF_BLINK_OFF		(LCD16X2_DISPLAY_ON | \
	LCD16X2_CURSOR_UNDERLINE_OFF | LCD16X2_CURSOR_BLINK_OFF)
#define LCD16X2_DISPLAY_ON_CURSOR_OFF_BLINK_ON		(LCD16X2_DISPLAY_ON | \
	LCD16X2_CURSOR_UNDERLINE_OFF | LCD16X2_CURSOR_BLINK_ON)
#define LCD16X2_DISPLAY_ON_CURSOR_ON_BLINK_OFF		(LCD16X2_DISPLAY_ON | \
	LCD16X2_CURSOR_UNDERLINE_ON | LCD16X2_CURSOR_BLINK_OFF)
#define LCD16X2_DISPLAY_ON_CURSOR_ON_BLINK_ON		(LCD16X2_DISPLAY_ON | \
	LCD16X2_CURSOR_UNDERLINE_ON | LCD16X2_CURSOR_BLINK_ON)
//////////////////////////////////////////////////////////////////////////////////////
/** Display size ------------------------------------------------------------ */
// Number of visible lines of the display (1 or 2)
#define LCD16X2_LINES					2
// Visible characters per line of the display
#define LCD16X2_DISP_LENGTH		16
// DDRAM address of first char of line 1
#define LCD16X2_START_LINE_1	0x00
// DDRAM address of first char of line 2
#define LCD16X2_START_LINE_2	0x40
///////////////////////////////////////////////////////////////
#define DAT 1
#define CMD 0
///////////////////////////////////////////////////////////////
//#define CUSTOM_TEXT_BIRD	0
//#define CUSTOM_TEXT_GHOGHNOOS	1
//
//#define	CUSTOM1_CHAR_TO	0
//#define	CUSTOM1_CHAR_SE	1
//#define	CUSTOM1_CHAR_EM	2
//#define	CUSTOM1_CHAR_MHV	3
//#define	CUSTOM1_CHAR_RGH	4
//#define	CUSTOM1_CHAR_AS	5
//#define	CUSTOM1_CHAR_D	6
//#define	CUSTOM1_CHAR_K	7
//
//#define	ASCII_CHAR_TO CUSTOM1_CHAR_TO+128
//#define	ASCII_CHAR_SE CUSTOM1_CHAR_SE+128
//#define	ASCII_CHAR_EM CUSTOM1_CHAR_EM+128
//#define	ASCII_CHAR_MHV CUSTOM1_CHAR_MHV+128
//#define	ASCII_CHAR_RGH CUSTOM1_CHAR_RGH+128
//#define	ASCII_CHAR_AS CUSTOM1_CHAR_AS+128
//#define	ASCII_CHAR_D CUSTOM1_CHAR_D+128
//#define	ASCII_CHAR_K CUSTOM1_CHAR_K+128
//
//#define	CUSTOM2_CHAR_BEEP	7
//#define	CUSTOM2_CHAR_dC	6
//#define CUSTOM2_CHAR_THERMO 5
//#define CUSTOM2_CHAR_DROP 4
////#define CUSTOM_CHAR_GH	3
////#define CUSTOM_CHAR_NN	2
////#define CUSTOM_CHAR_V	1
////#define CUSTOM_CHAR_S	0
//
////#define ASCII_CHAR_GH	CUSTOM_CHAR_GH+128
////#define ASCII_CHAR_NN	CUSTOM_CHAR_NN+128
////#define ASCII_CHAR_V	CUSTOM_CHAR_V+128
////#define ASCII_CHAR_S	CUSTOM_CHAR_S+128
//#define ASCII_CHAR_dC	CUSTOM2_CHAR_dC+128
//#define ASCII_CHAR_DROP CUSTOM2_CHAR_DROP+128
//#define ASCII_CHAR_THERMO CUSTOM2_CHAR_THERMO+128
/////////////////////////////////
//#define LINE2_DELAY	30 //10S for remind time
//#define ROLLING_DELAY	30	//20*10ms delay between roll of string
//#define	REPEAT_ROLLING	3
///////////////////////////////////////////////////////////////
//extern uint8_t custom_char1[][8];
//extern uint8_t custom_char2[][8];

void LCD_backlight(uint8_t onoff);
void LCD_GPIO_init(void);
void LCD_init(void);
void LCD_send(unsigned char value, unsigned char mode);
void LCD_4bit_send(unsigned char lcd_data);
void LCD_putstr(char *lcd_string);
void LCD_putchar(char char_data);
void LCD_clear_home(void);
void LCD_gotoxy(unsigned char x_pos, unsigned char y_pos);
void LCD_entry_inc(void);
void LCD_entry_dec(void);
void LCD_entry_inc_shift(void);
void LCD_entry_dec_shift(void);
void LCD_display_on(void);
void LCD_display_off(void);
void LCD_cursor_on(void);
void LCD_cursor_off(void);
void LCD_blink_on(void);
void LCD_blink_off(void);
void LCD_cursor_blink_off(void);
void LCD_cursor_blink_on(void);
void LCD_display_shift_right(void);
void LCD_display_shift_left(void);
void LCD_cursor_shift_left(void);
void LCD_cursor_shift_right(void);
void LCD_create_custom_char(uint8_t location, uint8_t *data_bytes);
void LCD_put_custom_char(uint8_t location);
void LCD_put_custom_text(uint8_t x, uint8_t y, uint8_t text);
void LCD_load_custom_char(uint8_t custom_chars[][8]);

void toggle_EN_pin(void);
void toggle_io(unsigned char lcd_data, unsigned char bit_pos, uint16_t pin_num);

#endif /* LCD_CH_H_ */
