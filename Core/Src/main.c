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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BASE_SPEED 69 // prędkośc bazowa PWM (0-99)
#define SPEED_INCR 10 // współczynnik zwiększania prędkości
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static bool turningLeft = false;
static bool turningRight = false;
static bool goingForward = false;
static bool goingBackward = false;
static int currentSpeed = BASE_SPEED;

void setSpeed(int speed1, int speed2)
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed2);
}

void setDirections(bool in1, bool in2, bool in3, bool in4)
{
  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, in1);
  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, in2);
  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, in3);
  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, in4);
}

void resetDirection()
{
  // wcześniej inny kierunek
  stop();         // reset silników
  HAL_Delay(150); // odczekanie
  currentSpeed = BASE_SPEED;
}

void toggleTurnLeft()
{
  if (turningRight)
  {
    // przeciwny kierunek zatrzymuje pojazd
    stop();
    return;
  }

  else if (turningLeft)
  {
    if (currentSpeed + SPEED_INCR <= 99)
    {
      currentSpeed += SPEED_INCR;
    }
  }
  else
  {
    resetDirection();
  }

  setSpeed(currentSpeed, currentSpeed);
  turningLeft = true;
  turningRight = false;
  goingForward = false;
  goingBackward = false;

  // lewy silnik do tyłu
  // prawy silnik do przodu
  setDirections(false, true, true, false);
}

void toggleTurnRight()
{
  if (turningLeft)
  {
    // przeciwny kierunek zatrzymuje pojazd
    stop();
    return;
  }

  else if (turningRight)
  {
    if (currentSpeed + SPEED_INCR <= 99)
    {
      currentSpeed += SPEED_INCR;
    }
  }
  else
  {
    resetDirection();
  }

  turningRight = true;
  turningLeft = false;
  goingForward = false;
  goingBackward = false;

  // lewy silnik do przodu
  // prawy silnik do tyłu
  setDirections(true, false, false, true);
}

void toggleGoForward()
{
  if (goingBackward)
  {
    // przeciwny kierunek zatrzymuje pojazd
    stop();
    return;
  }

  else if (goingForward)
  {
    if (currentSpeed + SPEED_INCR <= 99)
    {
      currentSpeed += SPEED_INCR;
    }
  }
  else
  {
    resetDirection();
  }

  goingForward = true;
  goingBackward = false;
  turningLeft = false;
  turningRight = false;

  setDirections(true, false, true, false);
}

void toggleGoBackward()
{
  if (goingForward)
  {
    // przeciwny kierunek zatrzymuje pojazd
    stop();
    return;
  }

  else if (goingBackward)
  {
    if (currentSpeed + SPEED_INCR <= 99)
    {
      currentSpeed += SPEED_INCR;
    }
  }
  else
  {
    resetDirection();
  }

  goingBackward = true;
  goingForward = false;
  turningLeft = false;
  turningRight = false;

  setDirections(false, true, false, true);
}

void stop()
{
  goingForward = false;
  goingBackward = false;
  turningLeft = false;
  turningRight = false;

  setSpeed(0, 0);

  setDirections(false, false, false, false);
}

bool shouldGoLeft()
{
  return HAL_GPIO_ReadPin(LEFT_GPIO_Port, LEFT_Pin);
}

bool shouldGoRight()
{
  return HAL_GPIO_ReadPin(RIGHT_GPIO_Port, RIGHT_Pin);
}

bool shouldGoForward()
{
  return HAL_GPIO_ReadPin(FORWARD_GPIO_Port, FORWARD_Pin);
}

bool shouldGoBackward()
{
  return HAL_GPIO_ReadPin(BACKWARD_GPIO_Port, BACKWARD_Pin);
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
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // Sterownik port "EN_A"
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // Sterownik port "EN_B"

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  bool prevForward = false;
  bool prevBackward = false;
  bool prevLeft = false;
  bool prevRight = false;
  while (1)
  {
    if (
        (HAL_GPIO_ReadPin(FORWARD_GPIO_Port, FORWARD_Pin) == GPIO_PIN_SET) ||
        (HAL_GPIO_ReadPin(BACKWARD_GPIO_Port, BACKWARD_Pin) == GPIO_PIN_SET) ||
        (HAL_GPIO_ReadPin(LEFT_GPIO_Port, LEFT_Pin) == GPIO_PIN_SET) ||
        (HAL_GPIO_ReadPin(RIGHT_GPIO_Port, RIGHT_Pin) == GPIO_PIN_SET))
    {
      // zapalenie leda na płytce

      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    }

    bool currentForward = shouldGoForward();
    bool currentBackward = shouldGoBackward();
    bool currentLeft = shouldGoLeft();
    bool currentRight = shouldGoRight();

    if (currentForward && !prevForward)
    {
      toggleGoForward();
    }
    else if (currentBackward && !prevBackward)
    {
      toggleGoBackward();
    }
    else if (currentLeft && !prevLeft)
    {
      toggleTurnLeft();
    }
    else if (currentRight && !prevRight)
    {
      toggleTurnRight();
    }
    else if (!goingForward && !goingBackward && !turningLeft && !turningRight)
    {
      // zgaszenie leda na płytce

      // zatrzymanie
      stop();
    }

    if (!currentForward && !currentBackward && !currentLeft && !currentRight)
    {
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    }

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
  htim1.Init.Prescaler = 71;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IN1_Pin | IN2_Pin | IN4_Pin | IN3_Pin, GPIO_PIN_RESET);

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
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DIST_Pin */
  GPIO_InitStruct.Pin = DIST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(DIST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LINE_3_Pin LINE_2_Pin LINE_1_Pin LEFT_Pin
                           RIGHT_Pin FORWARD_Pin BACKWARD_Pin */
  GPIO_InitStruct.Pin = LINE_3_Pin | LINE_2_Pin | LINE_1_Pin | LEFT_Pin | RIGHT_Pin | FORWARD_Pin | BACKWARD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
