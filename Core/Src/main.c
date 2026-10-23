/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum { //State Machine States
	STATE_TEMP_VIEW = 0,
	STATE_SET_ALARM_TEMP,
	STATE_SET_IDLE_TEMP,
	STATE_SET_SAMPLE_RATE,
	STATE_VIEW_THERMOCOUPLES,
	MAX_MENU_STATES
} MenuState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIMEOUT_DURATION_MS 60000 // 1 minute timer
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// 1602 I2C address
#define I2C_ADDR 0x27 // I2C address of the PCF8574
// 1602 dimensions
#define LCD_ROWS 2 // Number of rows on the LCD
#define LCD_COLS 16 // Number of columns on the LCD
// 1602 message bit numbers
#define DC_BIT 0 // Data/Command bit (register select bit)
#define EN_BIT 2 // Enable bit
#define BL_BIT 3 // Backlight bit
#define D4_BIT 4 // Data 4 bit
#define D5_BIT 5 // Data 5 bit
#define D6_BIT 6 // Data 6 bit
#define D7_BIT 7 // Data 7 bit
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

int CW = 0;
int CCW = 0;
int MenuState = 0;
int A = 0;
int B = 0;
int set = 0;
int Gray = 0b00;
int Anext;
int Bnext;

static MenuState_t current_state = STATE_TEMP_VIEW;
static uint32_t last_interaction_tick = 0;

// Initial variables / testing variables
static int current_temp = 150;
static int alarm_temp_max = 650;
static int alarm_temp_old = 650;
static int alarm_temp_new = 500;
static int idle_temp_old = 140;
static int idle_temp_new = 150;
static int sample_rate_old = 60;
static int sample_rate_new = 30;
static int tc1_temp = 150;
static int tc2_temp = 150;
static int tc3_temp = 150;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
//LCD functions
void CharLCD_Write_Nibble (uint8_t nibble, uint8_t dc);
void CharLCD_Send_Cmd(uint8_t cmd);
void CharLCD_Send_Data(uint8_t data);
void CharLCD_Init();
void CharLCD_Write_String(char *str);
void CharLCD_Set_Cursor(uint8_t row, uint8_t column);
void CharLCD_Clear(void);

//Menu functions
void Menu_Init(void);
void Menu_ButtonPress(void);
void Menu_EncoderRotate(int CW, int CCW);
void Menu_EncoderClick(void);
void Menu_Update(void);
static void Render_CurrentScreen(void);
static void Reset_Timeout(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  CharLCD_Init(); // Initialize the LCD and Menu
  Menu_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Menu_Update(); // Do menu things

	  // Menu button pressed
	  if (HAL_GPIO_ReadPin(GPIOC, MENU_BUTTON_Pin) == GPIO_PIN_RESET) {
		  HAL_Delay(50);
		  // Debounce
		  if (HAL_GPIO_ReadPin(GPIOC, MENU_BUTTON_Pin) == GPIO_PIN_RESET) {

			  Menu_ButtonPress();

			  while (HAL_GPIO_ReadPin(GPIOC, MENU_BUTTON_Pin) == GPIO_PIN_RESET) {
				  Menu_Update();
			  }
		  }
	  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : MENU_BUTTON_Pin */
  GPIO_InitStruct.Pin = MENU_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MENU_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EncSw_Pin A_Channel_Pin */
  GPIO_InitStruct.Pin = EncSw_Pin|A_Channel_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : B_Channel_Pin */
  GPIO_InitStruct.Pin = B_Channel_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B_Channel_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if (GPIO_Pin == EncSw_Pin){
		Menu_EncoderClick();
	}

	if (GPIO_Pin == A_Channel_Pin || GPIO_Pin == B_Channel_Pin){
		int ANext = HAL_GPIO_ReadPin(A_Channel_GPIO_Port, A_Channel_Pin);
		int BNext = HAL_GPIO_ReadPin(B_Channel_GPIO_Port, B_Channel_Pin);
		int GrayNext = (ANext << 1) + BNext;

		switch (Gray){
		case 0b00:
			if (GrayNext == 0b01){
				CCW = 1;
			}
			if (GrayNext == 0b10){
				CW = 1;
			}
			break;
		case 0b01:
			if (GrayNext == 0b11){
				CCW = 1;
			}
			if (GrayNext == 0b00){
				CW = 1;
			}
			break;
		case 0b10:
			if (GrayNext == 0b00){
				CCW = 1;
			}
			if (GrayNext == 0b11){
				CW = 1;
			}
			break;
		case 0b11:
			if (GrayNext == 0b10){
				CCW = 1;
			}
			if (GrayNext == 0b01){
				CW = 1;
			}
			break;
		}

		Gray = GrayNext;
		A = Anext;
		B = Bnext;

	}
}

void CharLCD_Write_Nibble(uint8_t nibble, uint8_t dc) {
 uint8_t data = nibble << D4_BIT; // Shift nibble to D4-D7 position
 data |= dc << DC_BIT; // Set DC bit for data/command selection
 data |= 1 << BL_BIT; // Include backlight state in data
 data |= 1 << EN_BIT; // Set enable bit high
 HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDR << 1, &data, 1, 100); // Send data with EN high
 HAL_Delay(1); // Wait for data setup
 data &= ~(1 << EN_BIT); // Clear enable bit (falling edge triggers LCD)
 HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDR << 1, &data, 1, 100); // Send data with EN low
}
/**
 * @brief Send command to LCD
 * @param cmd: 8-bit command to send to LCD controller
 * @retval None
 */
void CharLCD_Send_Cmd(uint8_t cmd) {
 uint8_t upper_nibble = cmd >> 4; // Extract upper 4 bits
 uint8_t lower_nibble = cmd & 0x0F; // Extract lower 4 bits
 CharLCD_Write_Nibble(upper_nibble, 0); // Send upper nibble (DC=0 for command)
 CharLCD_Write_Nibble(lower_nibble, 0); // Send lower nibble (DC=0 for command)
 if (cmd == 0x01 || cmd == 0x02) { // Clear display or return home commands
 HAL_Delay(2); // These commands need extra time
 }
}
/**
 * @brief Send data (character) to LCD
 * @param data: 8-bit character data to display
 * @retval None
 */
void CharLCD_Send_Data(uint8_t data) {
 uint8_t upper_nibble = data >> 4; // Extract upper 4 bits
 uint8_t lower_nibble = data & 0x0F; // Extract lower 4 bits
 CharLCD_Write_Nibble(upper_nibble, 1); // Send upper nibble (DC=1 for data)
 CharLCD_Write_Nibble(lower_nibble, 1); // Send lower nibble (DC=1 for data)
}
/**
 * @brief Initialize LCD in 4-bit mode via I2C
 * @param None
 * @retval None
 */
void CharLCD_Init() {
 HAL_Delay(50); // Wait for LCD power-on reset (>40ms)
 CharLCD_Write_Nibble(0x03, 0); // Function set: 8-bit mode (first attempt)
 HAL_Delay(5); // Wait >4.1ms
 CharLCD_Write_Nibble(0x03, 0); // Function set: 8-bit mode (second attempt)
 HAL_Delay(1); // Wait >100us
 CharLCD_Write_Nibble(0x03, 0); // Function set: 8-bit mode (third attempt)
 HAL_Delay(1); // Wait >100us
 CharLCD_Write_Nibble(0x02, 0); // Function set: switch to 4-bit mode
 CharLCD_Send_Cmd(0x28); // Function set: 4-bit, 2 lines, 5x8 font
 CharLCD_Send_Cmd(0x0C); // Display control: display on/cursor off/blink off
 CharLCD_Send_Cmd(0x06); // Entry mode: increment cursor, no shift
 CharLCD_Send_Cmd(0x01); // Clear display
 HAL_Delay(2); // Wait for clear display command
}
/**
 * @brief Write string to LCD at current cursor position
 * @param str: Pointer to null-terminated string
 * @retval None
 */
void CharLCD_Write_String(char *str) {
 while (*str) { // Loop until null terminator
 CharLCD_Send_Data(*str++); // Send each character and increment pointer
 }
}

void CharLCD_Set_Cursor(uint8_t row, uint8_t column) {
 uint8_t address;
 switch (row) {
 case 0:
 address = 0x00; break; // First line starts at address 0x00
 case 1:
 address = 0x40; break; // Second line starts at address 0x40
 default:
 address = 0x00; // Default to first line for invalid row
 }
 address += column; // Add column offset
 CharLCD_Send_Cmd(0x80 | address); // Set DDRAM address command (0x80 + address)
}
/**
 * @brief Clear LCD display and return cursor to home position
 * @param None
 * @retval None
 */
void CharLCD_Clear(void) {
 CharLCD_Send_Cmd(0x01); // Clear display command
 HAL_Delay(2); // Wait for command execution
}


// State Machine
void Menu_Init(void) {
	current_state = STATE_TEMP_VIEW;
	Reset_Timeout();
	Render_CurrentScreen();
}

// Cycle through Menu States when Menu Button is pressed.
void Menu_ButtonPress(void) {
	Reset_Timeout();
	current_state = (MenuState_t)((current_state + 1) % MAX_MENU_STATES);
	Render_CurrentScreen();
}

// Temporary Rotary encoder code
// Update to match Bryce's work.
void Menu_EncoderRotate(int CW, int CCW) {
	Reset_Timeout();

	switch (current_state) {
		case STATE_SET_ALARM_TEMP:
			if (CW) {
				alarm_temp_new ++;
				CW = 0;
			}
			if (CCW) {
				alarm_temp_new --;
				CCW = 0;
			}
			// Prevent alarm being set above max or below idle temps
			if (alarm_temp_new > alarm_temp_max)
				alarm_temp_new = alarm_temp_max;
			if (alarm_temp_new < (idle_temp_old + 5))
				alarm_temp_new = (idle_temp_old + 5);
			break;

		case STATE_SET_IDLE_TEMP:
			if (CW) {
				idle_temp_new ++;
				CW = 0;
			}
			if (CCW) {
				idle_temp_new --;
				CCW = 0;
			}
			// Prevent idle being set above alarm or below 0 temps.
			if (idle_temp_new > (alarm_temp_old - 5))
				idle_temp_new = (alarm_temp_old - 5);
			if (idle_temp_new < 0)
				idle_temp_new = 0;
			break;

		case STATE_SET_SAMPLE_RATE:
			if (CW) {
				sample_rate_new ++;
				CW = 0;
			}
			if (CCW) {
				sample_rate_new --;
				CCW = 0;
			}
			if (sample_rate_new < 1)
				sample_rate_new = 1;
			break;

		default:
			break;
	}
	Render_CurrentScreen();
}

// When Rotary Encoder is pressed, set new temps.
void Menu_EncoderClick(void) {
	Reset_Timeout();

	switch (current_state) {
		case STATE_SET_ALARM_TEMP:
			alarm_temp_old = alarm_temp_new;
			break;
		case STATE_SET_IDLE_TEMP:
			idle_temp_old = idle_temp_new;
			break;
		case STATE_SET_SAMPLE_RATE:
			sample_rate_old = sample_rate_new;
			break;
		default:
			break;
	}
	Render_CurrentScreen();
}

// If controller has not been interacted with for 1 minute, set screen to Temp View.
void Menu_Update(void) {
	if (current_state != STATE_TEMP_VIEW) {
		if ((HAL_GetTick() - last_interaction_tick) >= TIMEOUT_DURATION_MS) {
			current_state = STATE_TEMP_VIEW;
			Render_CurrentScreen();
		}
	}
}

// Reset 1 minute timer
static void Reset_Timeout(void) {
	last_interaction_tick = HAL_GetTick();
}

// Update the LCD screen with current info
static void Render_CurrentScreen(void) {
	// Reset screen
	char line1[17];
	char line2[17];

	for(int i=0; i<17; i++) {
		line1[i] = '\0';
		line2[i] = '\0';
	}
	// Print screen according to current state.
	switch (current_state) {
		case STATE_TEMP_VIEW:
			snprintf(line1, sizeof(line1), "Current     %3dF", current_temp);
			snprintf(line2, sizeof(line2), "Temp            ");
			break;
		case STATE_SET_ALARM_TEMP:
			snprintf(line1, sizeof(line1), "Alarm  Old: %3dF", alarm_temp_old);
		    snprintf(line2, sizeof(line2), "Temp   New: %3dF", alarm_temp_new);
		    break;

		case STATE_SET_IDLE_TEMP:
			snprintf(line1, sizeof(line1), "Idle   Old: %3dF", idle_temp_old);
			snprintf(line2, sizeof(line2), "Temp   New: %3dF", idle_temp_new);
			break;

		case STATE_SET_SAMPLE_RATE:
			snprintf(line1, sizeof(line1), "Sample Old: %2d/h", sample_rate_old);
			snprintf(line2, sizeof(line2), "Rate   New: %2d/h", sample_rate_new);
			break;

		case STATE_VIEW_THERMOCOUPLES:
			snprintf(line1, sizeof(line1), "TCs : 1   2   3 ");
			snprintf(line2, sizeof(line2), "Temp:%3d %3d %3d", tc1_temp, tc2_temp, tc3_temp);
			break;

		default:
			// If we see this, something went wrong
			snprintf(line1, sizeof(line1), "Unknown Screen");
			break;
	}

	//Write to LCD
	CharLCD_Clear();
	CharLCD_Set_Cursor(0, 0);
	CharLCD_Write_String(line1);
	CharLCD_Set_Cursor(1, 0);
	CharLCD_Write_String(line2);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
