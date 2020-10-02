/*
 * lcd_ch.c
 *
 *  Created on: Sep 17, 2019
 *      Author: mehdi
 */
	#include "lcd_ch.h"
#include "iwdg.h"
//	#include "tim.h"
//   uint8_t custom_char1[][8] = {
//		   {0, 0, 1, 30, 0, 14, 4, 0 },//P
//		   {0, 0, 0, 1, 1, 2, 12, 0 },//R
//		   {0, 4, 1, 30, 0, 0, 0, 0 },//N
//		   {0, 4, 2, 1, 1, 2, 12, 0 },//D
//		   {2, 5, 9, 9, 14, 0, 0, 0 },//H
//			 {0, 1, 1, 2, 18, 20, 8,0 },//TIK
//			 {24, 24, 0, 3, 4, 4, 3,0 },//'C
//			 {0, 4, 14, 14, 14, 31, 4, 0 }//ALARM oFF
//   };
//   uint8_t custom_char2[][8] = {
//		   {0, 2, 3, 18, 18, 18, 12, 0 },//S
//		   {0, 23, 21, 7, 1, 1, 14, 0 },//V
//		   {0, 0, 4, 1, 30, 0, 0, 0 },//N
//		   {3, 0, 7, 5, 31, 0, 0, 0 },//GH
//		   {4, 10, 10, 17, 31, 31, 14,0 },//DROP
//			 {4, 10, 10, 10, 10, 27, 31,14 },//thermo
//			 {24, 24, 0, 3, 4, 4, 3,0 },//'C
//			 {0, 0, 0, 0, 0, 0, 0, 0 }//ALARM oFF
//   };
//////////////////////////////////////////////////////////////////////////////////////////
//void lcd_delay_us(uint16_t delay_us)
//{
//	while(delay_us)
//	{
//		for(uint8_t i=0;i<60;i++)
//			__ASM("nop");
//		delay_us--;
//	}
//}
void lcd_delay_us(uint16_t time_us)
{
	_LCD_TIMER.Instance->CNT = 0;
	while(_LCD_TIMER.Instance->CNT <= time_us);
}
//////////////////////////////////////////////////////////////////////////////////////////
void LCD_GPIO_init(void)
{
	HAL_GPIO_WritePin(LCDRS_GPIO_Port, LCDRS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCDWR_GPIO_Port, LCDWR_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCDEN_GPIO_Port,LCDEN_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCDDB4_GPIO_Port, LCDDB4_Pin|LCDDB5_Pin|LCDDB6_Pin|LCDDB7_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
}
//////////////////////////////////////////////////////////////////////////////////////////
void toggle_EN_pin(void)
{
	HAL_GPIO_WritePin(LCDEN_GPIO_Port, LCDEN_Pin, GPIO_PIN_SET);
	lcd_delay_us(20);	//140
	HAL_GPIO_WritePin(LCDEN_GPIO_Port, LCDEN_Pin, GPIO_PIN_RESET);
	lcd_delay_us(20);	//140
}

//////////////////////////////////////////////////////////////////////////////////////////
void LCD_out(uint8_t nibble)
{
	HAL_GPIO_WritePin(LCDDB4_GPIO_Port, nibble, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCDDB4_GPIO_Port, (~nibble)&0x0F, GPIO_PIN_RESET);
//	if(nibble&0x01)
//		HAL_GPIO_WritePin(LCDDB4_GPIO_Port,LCDDB4_Pin,GPIO_PIN_SET);
//	else
//		HAL_GPIO_WritePin(LCDDB4_GPIO_Port,LCDDB4_Pin,GPIO_PIN_RESET);
//	if(nibble&0x02)
//		HAL_GPIO_WritePin(LCDDB5_GPIO_Port,LCDDB5_Pin,GPIO_PIN_SET);
//	else
//		HAL_GPIO_WritePin(LCDDB5_GPIO_Port,LCDDB5_Pin,GPIO_PIN_RESET);
//	if(nibble&0x04)
//		HAL_GPIO_WritePin(LCDDB6_GPIO_Port,LCDDB6_Pin,GPIO_PIN_SET);
//	else
//		HAL_GPIO_WritePin(LCDDB6_GPIO_Port,LCDDB6_Pin,GPIO_PIN_RESET);
//	if(nibble&0x08)
//		HAL_GPIO_WritePin(LCDDB7_GPIO_Port,LCDDB7_Pin,GPIO_PIN_SET);
//	else
//		HAL_GPIO_WritePin(LCDDB7_GPIO_Port,LCDDB7_Pin,GPIO_PIN_RESET);

	toggle_EN_pin();

}
//////////////////////////////////////////////////////////////////////////////////////////
void LCD_4bit_send(unsigned char lcd_data)
{

	LCD_out(lcd_data >> 4);
//	lcd_delay_us(10);
	LCD_out(lcd_data & 0x0F);
	lcd_delay_us(40);

}
//////////////////////////////////////////////////////////////////////////////////////////
void LCD_send(unsigned char value, unsigned char mode)
{
	switch(mode)
	{
		case DAT:
			HAL_GPIO_WritePin(LCDRS_GPIO_Port, LCDRS_Pin, GPIO_PIN_SET);
		break;
		case CMD:
			HAL_GPIO_WritePin(LCDRS_GPIO_Port, LCDRS_Pin, GPIO_PIN_RESET);
		break;
	}
	LCD_4bit_send(value);
}

//////////////////////////////////////////////////////////////////////////////////////////
void LCD_init(void)
{
	LCD_GPIO_init();
	HAL_TIM_Base_Start(&_LCD_TIMER);
	//three times send 8 bit interface then 4 bit interface command
	LCD_send(0x33,CMD);
	LCD_send(0x32,CMD);
	///////////////////////////////////////////////////////
	LCD_send(LCD16X2_FUNCTION_SET| LCD16X2_4BIT_INTERFACE |LCD16X2_2LINE_MODE |LCD16X2_5X7DOT_FORMAT,CMD);
	HAL_Delay(100);
	LCD_send(LCD16X2_DISPLAY_CURSOR_ON_OFF| LCD16X2_DISPLAY_ON| LCD16X2_CURSOR_UNDERLINE_OFF|LCD16X2_CURSOR_BLINK_OFF,CMD);
	HAL_Delay(100);
	LCD_send(LCD16X2_CLEAR_DISPLAY,CMD);
	HAL_Delay(100);
	LCD_send(LCD16X2_CHARACTER_ENTRY_MODE|LCD16X2_INCREMENT|LCD16X2_DISPLAY_SHIFT_OFF,CMD);

	LCD_backlight(100);
//   for(uint8_t i=0;i<8;i++)
//	   LCD_create_custom_char(i, custom_char1[i]);
}
//////////////////////////////////////////////////////////////////////////////////////////
void LCD_putstr(char *lcd_string)
{
	uint8_t index=0;
	do
	{
		#ifdef IWDG_ENABLE
		HAL_IWDG_Refresh(&hiwdg);
		#endif
//		if((lcd_string[index])<(char)128)
//			LCD_send(lcd_string[index], DAT);
//		else
//		{
//			LCD_put_custom_char(lcd_string[index]-128);
//		}
		LCD_send(lcd_string[index], DAT);
		index++;
	}while(lcd_string[index] != '\0');
}
//////////////////////////////////////////////////////////////////////////////////////////
void LCD_putchar(char char_data)
{
	LCD_send(char_data, DAT);
}
//////////////////////////////////////////////////////////////////////////////////////////
void LCD_clear_home(void)
{
	LCD_send(LCD16X2_CLEAR_DISPLAY, CMD);
	LCD_send(LCD16X2_CURSOR_HOME, CMD);
	HAL_Delay(5);
}
//////////////////////////////////////////////////////////////////////////////////////////
void LCD_gotoxy(unsigned char x_pos, unsigned char y_pos)
{
	if(y_pos == 0)
	{
		LCD_send(LCD16X2_SET_DDRAM_ADDRESS  | (LCD16X2_START_LINE_1 +x_pos), CMD);
	}
	else
	{
		LCD_send(LCD16X2_SET_DDRAM_ADDRESS  | (LCD16X2_START_LINE_2 +x_pos), CMD);
	}
}
//////////////////////////////////////////////////////////////////////////////////////////
void LCD_entry_inc(void)
{
	LCD_send(LCD16X2_CHARACTER_ENTRY_MODE | LCD16X2_INCREMENT |
		LCD16X2_DISPLAY_SHIFT_OFF,CMD);
}
//////////////////////////////////////////////////////////////////////////////////////////
void LCD_entry_dec(void)
{
	LCD_send(LCD16X2_CHARACTER_ENTRY_MODE | LCD16X2_DECREMENT |
		LCD16X2_DISPLAY_SHIFT_OFF,CMD);
}
//////////////////////////////////////////////////////////////////////////////////////////
void LCD_entry_inc_shift(void)
{
	LCD_send(LCD16X2_CHARACTER_ENTRY_MODE | LCD16X2_INCREMENT |
		LCD16X2_DISPLAY_SHIFT_ON,CMD);
}
//////////////////////////////////////////////////////////////////////////////////////////
void LCD_entry_dec_shift(void)
{
	LCD_send(LCD16X2_CHARACTER_ENTRY_MODE | LCD16X2_DECREMENT |
		LCD16X2_DISPLAY_SHIFT_ON,CMD);
}
//////////////////////////////////////////////////////////////////////////////////////////
void LCD_display_on(void)
{
	LCD_send(LCD16X2_DISPLAY_CURSOR_ON_OFF |
			LCD16X2_DISPLAY_ON|LCD16X2_CURSOR_UNDERLINE_OFF|LCD16X2_CURSOR_BLINK_OFF,CMD);
}
//////////////////////////////////////////////////////////////////////////////////////////
void LCD_display_off(void)
{
	LCD_send(LCD16X2_DISPLAY_CURSOR_ON_OFF |
			LCD16X2_DISPLAY_OFF|LCD16X2_CURSOR_UNDERLINE_OFF|LCD16X2_CURSOR_BLINK_OFF,CMD);
}
//////////////////////////////////////////////////////////////////////////////////////////
void LCD_cursor_on(void)
{
	LCD_send(LCD16X2_DISPLAY_CURSOR_ON_OFF |LCD16X2_DISPLAY_ON|
			LCD16X2_CURSOR_UNDERLINE_ON,CMD);
}
//////////////////////////////////////////////////////////////////////////////////////////
void LCD_cursor_off(void)
{
	LCD_send(LCD16X2_DISPLAY_CURSOR_ON_OFF |LCD16X2_DISPLAY_ON|
			LCD16X2_CURSOR_UNDERLINE_OFF,CMD);
}
//////////////////////////////////////////////////////////////////////////////////////////
void LCD_blink_on(void)
{
	LCD_send(LCD16X2_DISPLAY_CURSOR_ON_OFF |LCD16X2_DISPLAY_ON|
			LCD16X2_CURSOR_BLINK_ON,CMD);
}
//////////////////////////////////////////////////////////////////////////////////////////
void LCD_blink_off(void)
{
	LCD_send(LCD16X2_DISPLAY_CURSOR_ON_OFF |LCD16X2_DISPLAY_ON|
			LCD16X2_CURSOR_BLINK_OFF,CMD);
}
//////////////////////////////////////////////////////////////////////////////////////////
void LCD_cursor_blink_on(void)
{
	LCD_send(LCD16X2_DISPLAY_CURSOR_ON_OFF |LCD16X2_DISPLAY_ON|LCD16X2_CURSOR_BLINK_ON|
			LCD16X2_CURSOR_BLINK_ON,CMD);
}
//////////////////////////////////////////////////////////////////////////////////////////
void LCD_cursor_blink_off(void)
{
	LCD_send(LCD16X2_DISPLAY_CURSOR_ON_OFF |LCD16X2_DISPLAY_ON|LCD16X2_CURSOR_BLINK_OFF|
			LCD16X2_CURSOR_BLINK_OFF,CMD);
}
//////////////////////////////////////////////////////////////////////////////////////////
void LCD_display_shift_right(void)
{
	LCD_send(LCD16X2_DISPLAY_CURSOR_SHIFT |
		LCD16X2_DISPLAY_SHIFT | LCD16X2_RIGHT_SHIFT,CMD);
}
//////////////////////////////////////////////////////////////////////////////////////////
void LCD_display_shift_left(void)
{
	LCD_send(LCD16X2_DISPLAY_CURSOR_SHIFT |
		LCD16X2_DISPLAY_SHIFT | LCD16X2_LEFT_SHIFT,CMD);
}
//////////////////////////////////////////////////////////////////////////////////////////
void LCD_cursor_shift_left(void)
{
	LCD_send(LCD16X2_DISPLAY_CURSOR_SHIFT |
			LCD16X2_CURSOR_MOVE | LCD16X2_LEFT_SHIFT,CMD);
}
//////////////////////////////////////////////////////////////////////////////////////////
void LCD_cursor_shift_right(void)
{
	LCD_send(LCD16X2_DISPLAY_CURSOR_SHIFT |
			LCD16X2_CURSOR_MOVE | LCD16X2_RIGHT_SHIFT,CMD);
}
//////////////////////////////////////////////////////////////////////////////////////////
void LCD_create_custom_char(uint8_t location, uint8_t* data_bytes)
{
	int i;

	// We only have 8 locations 0-7 for custom chars
	location &= 0x07;

	// Set CGRAM address
	LCD_send(LCD16X2_SET_CGRAM_ADDRESS | (location << 3),CMD);

	// Write 8 bytes custom char pattern
	for (i = 0; i < 8; i++)
	{
		LCD_send(data_bytes[i],DAT);
	}
}

//////////////////////////////////////////////////////////////////////////////////////////
void LCD_backlight(uint8_t onoff)
{

	  TIM_OC_InitTypeDef sConfigOC = {0};
	  sConfigOC.OCMode = TIM_OCMODE_PWM1;
	  sConfigOC.Pulse =(uint16_t) __HAL_TIM_GET_AUTORELOAD(&BKL_TIMER)/10*(uint16_t)onoff/10 ;
	  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	  if (HAL_TIM_PWM_ConfigChannel(&BKL_TIMER, &sConfigOC, BLK_CHANNEL) != HAL_OK)
	  {
	    Error_Handler();
	  }
		HAL_TIM_PWM_Start_IT(&BKL_TIMER, BLK_CHANNEL);
}
////////////////////////////////////////////////////////////////////////////////////////////
//void LCD_put_custom_char( uint8_t location)
//{
//	LCD_send(location,DAT);
//}
////////////////////////////////////////////////////////////////////////////////////////////
//void LCD_load_custom_char(uint8_t custom_chars[][8])
//{
//	   for(uint8_t i=0;i<8;i++)
//				LCD_create_custom_char(i, custom_chars[i]);
//}
//////////////////////////////////////////////////////////////////////////////////////////
//void LCD_put_custom_text(uint8_t x,uint8_t y,uint8_t text)
//{
//	if(text==CUSTOM_TEXT_BIRD)
//	{
//		LCD_gotoxy(x+4,y);LCD_put_custom_char(CUSTOM_CHAR_P);
//		LCD_gotoxy(x+3,y);LCD_put_custom_char(CUSTOM_CHAR_R);
//		LCD_gotoxy(x+2,y);LCD_put_custom_char(CUSTOM_CHAR_N);
//		LCD_gotoxy(x+1,y);LCD_put_custom_char(CUSTOM_CHAR_D);
//		LCD_gotoxy(x+0,y);LCD_put_custom_char(CUSTOM_CHAR_H);
///*		for(uint8_t i=0;i<5;i++)
//		{
//			LCD_put_custom_char(x+(4-i),y,i);
//		}
//		*/
//	}
//	if(text==CUSTOM_TEXT_GHOGHNOOS)
//	{
//		LCD_gotoxy(x+4,y);LCD_put_custom_char(CUSTOM_CHAR_GH);
//		LCD_gotoxy(x+3,y);LCD_put_custom_char(CUSTOM_CHAR_GH);
//		LCD_gotoxy(x+2,y);LCD_put_custom_char(CUSTOM_CHAR_NN);
//		LCD_gotoxy(x+1,y);LCD_put_custom_char(CUSTOM_CHAR_V);
//		LCD_gotoxy(x+0,y);LCD_put_custom_char(CUSTOM_CHAR_S);
//
//	}
//
//}

