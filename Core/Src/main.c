/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include <stdbool.h>
#include <stdio.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <string.h>
#include "dwt_delay.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BASE_SPEED 50 // prędkośc bazowa PWM (0-99)

static const uint8_t LOW = 0;
static const uint8_t HIGH = 1;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int currentSpeed = BASE_SPEED;
static uint32_t last_update_time = 0;

void setSpeed(int speed1, int speed2)
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed2);
}

void setDirections(GPIO_PinState in1, GPIO_PinState in2, GPIO_PinState in3, GPIO_PinState in4)
{
  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, in1);
  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, in2);
  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, in3);
  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, in4);
}

void stop()
{
  setSpeed(0, 0);
  setDirections(false, false, false, false);
}

void resetDirection()
{
  stop();         // reset silników
  HAL_Delay(150); // odczekanie
}

void turnLeft()
{
  setSpeed(BASE_SPEED + 25, BASE_SPEED + 25);
  // lewy silnik do tyłu
  // prawy silnik do przodu
  setDirections(LOW, HIGH, HIGH, LOW);
}

void turnRight()
{
  setSpeed(BASE_SPEED + 25, BASE_SPEED + 25);
  // lewy silnik do przodu
  // prawy silnik do tyłu
  setDirections(HIGH, LOW, LOW, HIGH);
}

void goForward()
{
  setSpeed(currentSpeed, currentSpeed);
  setDirections(HIGH, LOW, HIGH, LOW);
}

void goBackward()
{
  setSpeed(currentSpeed, currentSpeed);
  setDirections(LOW, HIGH, LOW, HIGH);
}

float read_distance_cm()
{
  uint32_t startTick, endTick, pulseDuration;
  float distance;

  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, LOW);
  DWT_Delay(2);
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, HIGH);
  DWT_Delay(10);
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, LOW);

  // Wait for Echo to go HIGH
  while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == LOW)
    ;
  startTick = DWT->CYCCNT;

  // Wait for Echo to go LOW
  while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == HIGH)
    ;
  endTick = DWT->CYCCNT;

  uint32_t cpu_freq = HAL_RCC_GetHCLKFreq();
  pulseDuration = (float)(endTick - startTick) / (cpu_freq / 1000000.0f);

  // distance in cm
  distance = pulseDuration * 0.0343f / 2.0f;

  return distance;
}

bool shouldGoLeft()
{
  return HAL_GPIO_ReadPin(LEFT_GPIO_Port, LEFT_Pin) == HIGH;
}

bool shouldGoRight()
{
  return HAL_GPIO_ReadPin(RIGHT_GPIO_Port, RIGHT_Pin) == HIGH;
}

bool shouldGoForward()
{
  return HAL_GPIO_ReadPin(FORWARD_GPIO_Port, FORWARD_Pin) == HIGH;
}

bool shouldGoBackward()
{
  return HAL_GPIO_ReadPin(BACKWARD_GPIO_Port, BACKWARD_Pin) == HIGH;
}

void displayWriteMode(bool autoMode)
{
  char str[13];
  snprintf(str, sizeof(str), "Tryb: %s", autoMode ? "auto" : "manual");
  ssd1306_FillRectangle(0, 21, 128, 28, Black);
  ssd1306_SetCursor(0, 21);
  ssd1306_WriteString(str, Font_7x10, White);
  ssd1306_UpdateScreen();
}

typedef enum
{
  STOP = 0,
  RIGHT = 1,
  LEFT = 2,
  FORWARD = 3,
  BACKWARD = 4
} Direction;
/**
 * 1- w prawo
 * 2- w lewo
 * 3- do przodu
 * 4- do tylu
 * 0 - stop
 */
void displayWriteDir(Direction dirNum)
{
  char str[17];
  char dir[10];
  switch (dirNum)
  {
  case RIGHT:
    strcpy(dir, "Prawo");
    break;
  case LEFT:
    strcpy(dir, "Lewo");
    break;
  case FORWARD:
    strcpy(dir, "Wprost");
    break;
  case BACKWARD:
    strcpy(dir, "Tylem");
    break;
  default:
    strcpy(dir, "STOP");
    break;
  }
  snprintf(str, sizeof(str), "Dir:%s", dir);
  ssd1306_FillRectangle(0, 0, 70, 10, Black);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString(str, Font_7x10, White);
  ssd1306_UpdateScreen();
}

void displayWriteDistance(float distance)
{
  char str[20];
  int int_part = (int)distance;
  int decimal_part = (int)((distance - int_part) * 10);
  snprintf(str, sizeof(str), "Dst:%d.%01d cm", int_part, decimal_part);
  ssd1306_FillRectangle(75, 0, 128, 10, Black);
  ssd1306_SetCursor(75, 0);
  ssd1306_WriteString(str, Font_6x8, White);
  ssd1306_UpdateScreen();
}
/**
 * Zmienia stan wbudowanego LED `state` na płytce
 */
void toggleBoardLed(bool state)
{
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, !state);
}

uint8_t leftOnLine, centerOnLine, rightOnLine;

/** Aktualizuje zmienne o wartości czy czujnik wykrył linię */
void readLineSensors()
{
  leftOnLine = HAL_GPIO_ReadPin(LINE_LEFT_GPIO_Port, LINE_LEFT_Pin) == HIGH;
  centerOnLine = HAL_GPIO_ReadPin(LINE_CENTER_GPIO_Port, LINE_CENTER_Pin) == HIGH;
  rightOnLine = HAL_GPIO_ReadPin(LINE_RIGHT_GPIO_Port, LINE_RIGHT_Pin) == HIGH;
}

static Direction lastMove = FORWARD;

void runAutoMode()
{
  readLineSensors();

  if (centerOnLine)
  {
    goForward();
    lastMove = FORWARD;
    displayWriteDir(FORWARD);
  }
  else if (leftOnLine)
  {
    turnLeft();
    lastMove = LEFT;
    displayWriteDir(LEFT);
  }
  else if (rightOnLine)
  {
    turnRight();
    lastMove = RIGHT;
    displayWriteDir(RIGHT);
  }
  else
  {
    switch (lastMove)
    {
    case 1:
      turnRight();
      break;
    case 2:
      turnLeft();
      break;
    case 3:
      goForward();
      break;

    default:
      break;
    }
    displayWriteDir(lastMove);
  }
}

bool handleDistance()
{
  uint32_t current_time = HAL_GetTick();

  bool obstacleDetected = false;
  // aktualizacja wyświetlacza co 0.45s o dane z sensora odległości
  if ((current_time - last_update_time) >= 450)
  {
    last_update_time = current_time;
    float dist = read_distance_cm();
    displayWriteDistance(dist);
    obstacleDetected = (dist <= 50);
  }

  return obstacleDetected;
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  __HAL_AFIO_REMAP_SWJ_NOJTAG();
  __HAL_AFIO_REMAP_I2C1_ENABLE();
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // Sterownik port "EN_A"
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // Sterownik port "EN_B"
  toggleBoardLed(false);
  ssd1306_Init();

  displayWriteMode(false);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  bool prevForward = false;
  bool prevBackward = false;
  bool prevLeft = false;
  bool prevRight = false;

  uint8_t buttonPrevState = 0;
  uint8_t autoMode = 0;

  DWT_Init();

  bool obstacleDetected = false;

  while (1)
  {
    uint8_t button_current = HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin);

    if (buttonPrevState == 0 && button_current == 1)
    {
      autoMode = !autoMode;
      displayWriteMode(autoMode);
    }

    buttonPrevState = button_current;

    bool currentForward = shouldGoForward();

    obstacleDetected = handleDistance();

    if (autoMode)
    {
      currentSpeed = BASE_SPEED;
      runAutoMode();

      char str[16];
      snprintf(str, sizeof(str), "L%d C%d R%d", leftOnLine, centerOnLine, rightOnLine);
      ssd1306_FillRectangle(90, 21, 128, 28, Black);
      ssd1306_SetCursor(80, 21);
      ssd1306_WriteString(str, Font_6x8, White);
      ssd1306_UpdateScreen();

      HAL_Delay(50);
      stop();
      HAL_Delay(100);
      continue;
    }
    currentSpeed = 65;

    bool currentBackward = shouldGoBackward();
    bool currentLeft = shouldGoLeft();
    bool currentRight = shouldGoRight();
    if (
        currentForward || currentBackward || currentLeft || currentRight)
    {
      toggleBoardLed(true);
    }

    if (currentForward)
    {
      goForward();
      if (!prevForward)
      {
        displayWriteDir(1);
      }
    }
    else if (currentBackward)
    {
      goBackward();
      if (!prevBackward)
      {
        displayWriteDir(2);
      }
    }
    else if (currentLeft && !obstacleDetected)
    {
      turnLeft();
      if (!prevLeft)
      {
        displayWriteDir(3);
      }
    }
    else if (currentRight)
    {
      turnRight();
      if (!prevRight)
      {
        displayWriteDir(4);
      }
    }
    else
    {
      toggleBoardLed(false);
      resetDirection();

      if (prevBackward || prevForward || prevLeft || prevRight)
      {
        displayWriteDir(0);
      }
    }

    // if (currentLeft || currentRight || currentForward || currentBackward) {
    //   HAL_Delay(150);
    // }

    prevForward = currentForward;
    prevBackward = currentBackward;
    prevLeft = currentLeft;
    prevRight = currentRight;

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

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 700;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IN1_Pin | IN2_Pin | IN4_Pin | IN3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_Pin IN2_Pin IN4_Pin IN3_Pin */
  GPIO_InitStruct.Pin = IN1_Pin | IN2_Pin | IN4_Pin | IN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LINE_RIGHT_Pin LINE_CENTER_Pin LINE_LEFT_Pin LEFT_Pin
                           RIGHT_Pin FORWARD_Pin BACKWARD_Pin */
  GPIO_InitStruct.Pin = LINE_RIGHT_Pin | LINE_CENTER_Pin | LINE_LEFT_Pin | LEFT_Pin | RIGHT_Pin | FORWARD_Pin | BACKWARD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ECHO_Pin */
  GPIO_InitStruct.Pin = ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
