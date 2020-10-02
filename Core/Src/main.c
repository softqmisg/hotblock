/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "iwdg.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "lcd_ch.h"
#include "keypad.h"
#include "ds18b20.h"
#include "pid_controller.h"
#include "eeprom.h"
#include "ds1307.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define SWVERSION	"2.0"
#define HWVERSION	"1.1"
typedef enum {
	MAIN_Menu = 0,
	SETHOUR_Menu=1,
	SETMINUTE_Menu=2,
	SETYEAR_Menu=3,
	SETMONTH_Menu=4,
	SETDAY_Menu=5
//	SWVERSION_Menu,
//	HWVERSION_Menu
} menu_t;
//char select_menu[][14] = { 	"1-SET TIME:  \0",
//							"2-SET DATE:  \0",
//							"3-SW VERSION:\0",
//							"4-HW VERSION:\0" };
uint16_t VirtAddVarTab[NB_OF_VAR] = { EEPROM_START_ADDRESS, EEPROM_START_ADDRESS
		+ 2, EEPROM_START_ADDRESS + 4 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * precision:
 * 10=one digit
 * 100=two digit
 * 1000=three digit
 */
uint16_t FLOAT2DEC(float in, uint16_t precision) {
	if (in < 0.0)
		in = -in;
	return (uint16_t) ((in) * precision) % precision;
}

int16_t FLOAT2INT(float in) {
	return (int16_t) in;
}
static inline void BUZ_short(void) {
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

}
static inline void BUZ_Long(void) {
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
	HAL_Delay(700);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

}
//HAL_TIM_PeriodElapsedCallback
//HAL_TIM_PWM_PulseFinishedCallback
//HAL_TIM_OC_DelayElapsedCallback
/*
 * time1=PID PWM,50hz
 * time3=Backlight PWM ,interrupt for keypad search,1khz
 * time16=delay us
 * time17=1s interrupt
 * timer  interrupt handlers
 */
///////////////////////////////TIMER3////////////////////////////////////////
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == KEYPAD_TIMER
			&& keypad_state() == keypad_initialized) {
		keypad_read(Key_ALL, no_press);
	}
}
////////////////////////////TIMER17///////////////////////////////////////////
volatile int8_t GOMAINMENU_counter = 0;
volatile uint8_t flag_1s = 0, flag_1smain = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM17) {

		flag_1s = 1;
		flag_1smain = 1;
		if (GOMAINMENU_counter > 0)
			GOMAINMENU_counter--;
	}
}
/*
 * pwm_input=0->1000;~0%:0.1%:100%
 */
#define PID_TIMER htim1
#define PID_CHANNEL	TIM_CHANNEL_1

void PID_PWM(uint16_t duty) {

	__HAL_TIM_SET_COMPARE(&PID_TIMER, TIM_CHANNEL_1, duty);
}
/*
 *
 */
void show_Time(uint8_t xpos, uint8_t ypos, Time_t time) {
	char text[18];
	if ((time.second % 2) == 0)
		sprintf(text, "%02d:%02d", time.hour, time.minute);
	else
		sprintf(text, "%02d %02d", time.hour, time.minute);

	LCD_gotoxy(xpos, ypos);
	LCD_putstr(text);

}
void show_Date(uint8_t xpos, uint8_t ypos, Date_t Date) {
	char text[18];
	sprintf(text, "%02d/%02d/%02d", Date.year, Date.month, Date.date);
	LCD_gotoxy(xpos, ypos);
	LCD_putstr(text);

}
void show_temperature(Ds18b20Sensor_t ds18b20, double setpoint) {
	char text[18];
	if (ds18b20.DataIsValid) {
		sprintf(text, "TEMP=%02d.%d(%02d.%d)", FLOAT2INT(ds18b20.Temperature),
				FLOAT2DEC(ds18b20.Temperature, 10), FLOAT2INT(setpoint),
				FLOAT2DEC(setpoint, 10));
	} else {
		sprintf(text, "TEMP=----(%02d.%0d)", FLOAT2INT(setpoint),
				FLOAT2DEC(setpoint, 10));
	}
	LCD_gotoxy(0, 1);
	LCD_putstr(text);
}
void check_updownkey(uint8_t *value, uint8_t max, uint8_t min, uint8_t xipos,
		uint8_t yipos, uint8_t xbpos, uint8_t ybpos) {
	char text[5];
	keypad_init(Key_UP | Key_DOWN | Key_SET, Long_press);

	if (keypad_read(Key_UP, Short_press)) {
		keypad_init(Key_UP, Short_press);
		GOMAINMENU_counter = GOMAINMENU_DELAY;
		(*value)++;
		if (*value > max)
			*value = min;
		sprintf(text, "%02d", *value);
		LCD_gotoxy(xipos, yipos);
		LCD_putstr(text);
		LCD_gotoxy(xbpos, ybpos);
	}
	if (keypad_read(Key_DOWN, Short_press)) {
		keypad_init(Key_DOWN, Short_press);
		GOMAINMENU_counter = GOMAINMENU_DELAY;
		if (*value == min)
			*value = max + 1;
		(*value)--;
		sprintf(text, "%02d", *value);
		LCD_gotoxy(xipos, yipos);
		LCD_putstr(text);
		LCD_gotoxy(xbpos, ybpos);
	}

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_IWDG_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	uint8_t max_day = 0;
	char str_display0[20];
	char str_display1[20];
	uint8_t menu_index = 0;
	double kp = 300.0, kd = 5.0, ki = 0.0;
	double cur_setpoint = 39.0, tmp_setpoint;
	Ds18b20Sensor_t ds18b20;
	PIDControl pid_temperature;
	menu_t menu_choice = MAIN_Menu;
	/////////////////////////////////////////////////////////////////////
	LCD_init();
	LCD_gotoxy(0, 0);
	LCD_putstr("init device...");
	////////////////////////////init Date & time////////////////////////////////////////
	Date_t cur_date = { .year = 20, .month = 10, .date = 2, .day = 5 };
	Time_t cur_time = { .hour = 16, .minute = 51, .second = 40 };
	uint8_t ramdata;
	DS1307Read(0x08, &ramdata);
	if ((ramdata != 1)|| !(HAL_GPIO_ReadPin(KEYUP_GPIO_Port, KEYUP_Pin)
			|| HAL_GPIO_ReadPin(KEYDN_GPIO_Port, KEYDN_Pin))) {
		Set_Time(cur_time);
		Set_Date(cur_date);
		uint8_t userram = 1;
		DS1307Write(0x08, &userram);
		DS1307Read(0x08, &ramdata);
	}
	DS1307_Init();
	///////////////////////////Start EEPROM///////////////
	HAL_FLASH_Unlock();
	EE_Init();
	uint16_t varValue;
	EE_ReadVariable(VirtAddVarTab[0], &varValue);
	if ((uint8_t) varValue == 0
			|| !(HAL_GPIO_ReadPin(KEYUP_GPIO_Port, KEYUP_Pin)
					|| HAL_GPIO_ReadPin(KEYDN_GPIO_Port, KEYDN_Pin))) {
		EE_WriteVariable(VirtAddVarTab[0], 1);
		HAL_Delay(5);
		EE_WriteVariable(VirtAddVarTab[1], 370);
		HAL_Delay(5);
		BUZ_Long();
	}
	EE_ReadVariable(VirtAddVarTab[1], &varValue);
	cur_setpoint = (double) varValue / 10.0;
	tmp_setpoint = cur_setpoint;
	//////////////////////////////Init DS18B20/////////////////////////
	DS18B20_Init();
	//////////////////////////////Init keypad/////////////////////////
	keypad_init(Key_ALL, Both_press);
	//////////////////////////////start Timers/////////////////////////
//	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim17);
	HAL_TIM_PWM_Start(&PID_TIMER, PID_CHANNEL);
	PID_PWM(0);
	/////////////////////////////////////////////////////////////////////
	HAL_Delay(500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	bool backtomainmenu=false;
	BUZ_short();
	LCD_clear_home();
//	HAL_Delay(10);
	PIDInit(&pid_temperature, kp, kd, ki, 1.0, 0.0, 1000.0, AUTOMATIC, DIRECT);
	PIDSetpointSet(&pid_temperature, cur_setpoint);
	while (1) {

#ifdef IWDG_ENABLE
		HAL_IWDG_Refresh(&hiwdg);
#endif
		/////////////////////////////run always/////////////////////////////////////////

		/////////////////////////////switch menu////////////////////////////////////////////
		switch ((uint8_t) menu_choice) {
		/////////////////////////Main Menu Switch//////////////////////////////
		case MAIN_Menu:
			if(backtomainmenu)
			{
				backtomainmenu=false;
				PIDInit(&pid_temperature, kp, kd, ki, 1.0, 0.0, 1000.0,
						AUTOMATIC, DIRECT);
				PIDSetpointSet(&pid_temperature, cur_setpoint);
			}
			if (flag_1smain) {
				flag_1smain = 0;
				if (DS18B20_get_temperature(&ds18b20)) {
					PIDInputSet(&pid_temperature, ds18b20.Temperature);
					PIDCompute(&pid_temperature);
					PID_PWM((uint16_t) PIDOutputGet(&pid_temperature));
					if (ds18b20.Temperature > cur_setpoint + 0.5)
						HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_SET);
					else
						HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_RESET);
				} else {
					PID_PWM(0);
					DS18B20_Init();
					PIDInit(&pid_temperature, kp, kd, ki, 1.0, 0.0, 1000.0,
							AUTOMATIC, DIRECT);
					PIDSetpointSet(&pid_temperature, cur_setpoint);
				}
				show_temperature(ds18b20, tmp_setpoint);
				Get_Time_Date(&cur_time, &cur_date);
				//cur_time.second++;if(cur_time.second>60)cur_time.second=0;
				show_Time(0, 0, cur_time);
				show_Date(7, 0, cur_date);
			}
			if (keypad_read(Key_UP, Short_press)) {
				keypad_init(Key_UP, Short_press);
				tmp_setpoint += 0.1;
				if (tmp_setpoint > MAX_TEMP) {
					tmp_setpoint = MAX_TEMP;
				}
				show_temperature(ds18b20, tmp_setpoint);
				LCD_gotoxy(15, 1);
				LCD_putchar('*');
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
				GOMAINMENU_counter = GOMAINMENU_DELAY;
			}
			if (keypad_read(Key_UP, Long_press)) {
				keypad_init(Key_UP, Long_press);
				tmp_setpoint += 1.0;
				if (tmp_setpoint > MAX_TEMP) {
					tmp_setpoint = MAX_TEMP;
				}
				show_temperature(ds18b20, tmp_setpoint);
				LCD_gotoxy(15, 1);
				LCD_putchar('*');
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
				GOMAINMENU_counter = GOMAINMENU_DELAY;
			}
			if (keypad_read(Key_DOWN, Short_press)) {
				keypad_init(Key_DOWN, Short_press);
				tmp_setpoint -= 0.1;
				if (tmp_setpoint < MIN_TEMP) {
					tmp_setpoint = MIN_TEMP;
				}
				show_temperature(ds18b20, tmp_setpoint);
				LCD_gotoxy(15, 1);
				LCD_putchar('*');
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
				GOMAINMENU_counter = GOMAINMENU_DELAY;
			}
			if (keypad_read(Key_DOWN, Long_press)) {
				keypad_init(Key_DOWN, Long_press);
				tmp_setpoint -= 1.0;
				if (tmp_setpoint < MIN_TEMP) {
					tmp_setpoint = MIN_TEMP;
				}
				show_temperature(ds18b20, tmp_setpoint);
				LCD_gotoxy(15, 1);
				LCD_putchar('*');
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
				GOMAINMENU_counter = GOMAINMENU_DELAY;
			}
			if (keypad_read(Key_SET, Short_press)) {
				keypad_init(Key_SET, Short_press);
				cur_setpoint = tmp_setpoint;
				varValue = (uint16_t) ((double) cur_setpoint * 10.0);
				EE_WriteVariable(VirtAddVarTab[1], varValue);

				PIDSetpointSet(&pid_temperature, cur_setpoint);
				LCD_gotoxy(15, 1);
				LCD_putchar(' ');
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
				BUZ_short();
				PID_PWM(0);
			}
			if (keypad_read(Key_SET, Long_press)) {
				keypad_init(Key_SET, Long_press);
				menu_choice = SETHOUR_Menu;
				GOMAINMENU_counter = GOMAINMENU_DELAY;
				BUZ_short();
				LCD_clear_home();
//				HAL_Delay(10);
				LCD_gotoxy(0, 0);
				LCD_putstr("1-SET TIME:");
				cur_time.second = 0;
				show_Time(5, 1, cur_time);
				LCD_gotoxy(6, 1);
				LCD_cursor_on();

			}
			if (GOMAINMENU_counter == 0) {
				GOMAINMENU_counter = -1;
				tmp_setpoint = cur_setpoint;
				LCD_gotoxy(15, 1);
				LCD_putchar(' ');
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
			}
			break;
			/////////////////////////set hour Menu //////////////////////////////
		case SETHOUR_Menu:
			check_updownkey(&cur_time.hour, 23, 0, 5, 1, 6, 1);
			if (keypad_read(Key_SET, Short_press)) {
				keypad_init(Key_SET, Short_press);
				GOMAINMENU_counter = GOMAINMENU_DELAY;
				menu_choice = SETMINUTE_Menu;
				sprintf(str_display1, "%02d", cur_time.minute);
				LCD_gotoxy(8, 1);
				LCD_putstr(str_display1);
				LCD_gotoxy(9, 1);
			}
			if (GOMAINMENU_counter == 0) {
				GOMAINMENU_counter = -1;
				backtomainmenu=true;
				LCD_clear_home();
//				HAL_Delay(10);
				menu_choice = MAIN_Menu;
				LCD_cursor_off();
			}
			break;
			/////////////////////////set minute Menu //////////////////////////////
		case SETMINUTE_Menu:
			check_updownkey(&cur_time.minute, 59, 0, 8, 1, 9, 1);
			if (keypad_read(Key_SET, Short_press)) {
				keypad_init(Key_SET, Short_press);
				Set_Time(cur_time);
				GOMAINMENU_counter = GOMAINMENU_DELAY;
				menu_choice = SETYEAR_Menu;
				LCD_clear_home();
//				HAL_Delay(10);

				LCD_gotoxy(0, 0);
				LCD_putstr("2-SET DATE:");
				show_Date(4, 1, cur_date);
				LCD_gotoxy(5, 1);
				BUZ_short();
			}
			if (GOMAINMENU_counter == 0) {
				GOMAINMENU_counter = -1;
				backtomainmenu=true;
				LCD_clear_home();
//				HAL_Delay(10);

				menu_choice = MAIN_Menu;
				LCD_cursor_off();
			}
			break;
			/////////////////////////set year Menu //////////////////////////////
		case SETYEAR_Menu:
			check_updownkey(&cur_date.year, 99, 0, 4, 1, 5, 1);
			if (keypad_read(Key_SET, Short_press)) {
				keypad_init(Key_SET, Short_press);
				GOMAINMENU_counter = GOMAINMENU_DELAY;
				menu_choice = SETMONTH_Menu;
				sprintf(str_display1, "%02d", cur_date.month);
				LCD_gotoxy(7, 1);
				LCD_putstr(str_display1);
				LCD_gotoxy(8, 1);
			}
			if (GOMAINMENU_counter == 0) {
				GOMAINMENU_counter = -1;
				backtomainmenu=true;
				LCD_clear_home();
//				HAL_Delay(10);
				menu_choice = MAIN_Menu;
				LCD_cursor_off();
			}
			break;
			/////////////////////////set month Menu //////////////////////////////
		case SETMONTH_Menu:
			check_updownkey(&cur_date.month, 12, 1, 7, 1, 8, 1);
			if (keypad_read(Key_SET, Short_press)) {
				keypad_init(Key_SET, Short_press);
				GOMAINMENU_counter = GOMAINMENU_DELAY;
				menu_choice = SETDAY_Menu;
				sprintf(str_display1, "%02d", cur_date.date);
				LCD_gotoxy(10, 1);
				LCD_putstr(str_display1);
				LCD_gotoxy(11, 1);
				if (cur_date.month == 2)
					max_day = gDaysInMonth[cur_date.month - 1]
							+ gLeap((uint16_t) 2000 + cur_date.year);
				else
					gDaysInMonth[cur_date.month - 1];

			}
			if (GOMAINMENU_counter == 0) {
				GOMAINMENU_counter = -1;
				backtomainmenu=true;
				LCD_clear_home();
//				HAL_Delay(10);
				menu_choice = MAIN_Menu;
				LCD_cursor_off();
			}
			break;
			/////////////////////////set month Menu //////////////////////////////
		case SETDAY_Menu:
			check_updownkey(&cur_date.date, max_day, 1, 10, 1, 11, 1);
			if (keypad_read(Key_SET, Short_press)) {
				keypad_init(Key_SET, Short_press);
				Set_Date(cur_date);
				GOMAINMENU_counter = 0;
				BUZ_short();
				LCD_cursor_off();
			}
			if (GOMAINMENU_counter == 0) {
				GOMAINMENU_counter = -1;
				backtomainmenu=true;
				LCD_clear_home();
//				HAL_Delay(10);

				menu_choice = MAIN_Menu;
				LCD_cursor_off();
			}
			break;
			///////////////////////////////////////////////////////////
		}
		///////////////////////////////////end of switch/////////////////////////////////

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
