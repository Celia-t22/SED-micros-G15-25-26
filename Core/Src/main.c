/* USER CODE BEGIN Header */
/**
  **************************
  * @file           : main.c
  * @brief          : Main program body
  **************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "fonts.h"
#include <stdio.h>
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
// --- VARIABLES DE HARDWARE ---
uint32_t val_joyX = 0;
uint32_t val_joyY = 0;
uint32_t val_LDR_Inicio = 0;
uint32_t val_LDR_Fin = 0;
uint32_t val_LDR_Perder = 0;  // <--- NUEVA (Sustituye a estadoLaser)

// --- VARIABLES DEL JUEGO ---
// Definimos los nombres de los estados
typedef enum {
    ESTADO_INTRO,       // Pantalla "EL LABERINTO"
    ESTADO_SELECCION,   // Elegir "NORMAL" o "LOCO"
    ESTADO_CUENTA,      // 3, 2, 1...
    ESTADO_JUGANDO,     // Mover servos y leer sensores
    ESTADO_GANADO,      // Pantalla Win
    ESTADO_PERDIDO      // Pantalla Game Over
} EstadoJuego;

EstadoJuego estadoActual = ESTADO_INTRO; // Empezamos en la intro

// Variables para el menú
int opcionMenu = 0;     // 0 = Normal, 1 = Loco
int modoJuego = 0;      // Guardará lo que hayamos elegido (0=Normal, 1=Loco)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // Arranca Servo X
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // Arranca Servo Y
  HAL_ADC_Start(&hadc1);

    // Encender el Emisor Láser
  //HAL_GPIO_WritePin(LASER_ON_GPIO_Port, LASER_ON_Pin, GPIO_PIN_SET);

    // Iniciar Pantalla
  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_UpdateScreen();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  switch (estadoActual) {

	  // ============================================================
	          // 1. PANTALLA DE TÍTULO (DISEÑO FINAL)
	          // ============================================================
	          case ESTADO_INTRO:
	              ssd1306_Fill(Black);

	              // "EL" centrado arriba
	              ssd1306_SetCursor(50, 5);
	              ssd1306_WriteString("EL", Font_11x18, White);

	              // "LABERINTO" centrado en medio
	              ssd1306_SetCursor(15, 25);
	              ssd1306_WriteString("LABERINTO", Font_11x18, White);

	              // Instrucción abajo
	              ssd1306_SetCursor(30, 50);
	              ssd1306_WriteString("dale a OK", Font_7x10, White);

	              ssd1306_UpdateScreen();

	              // Esperar al Botón A (PD0) para empezar
	              if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0) == GPIO_PIN_RESET) {
	                  estadoActual = ESTADO_SELECCION;
	                  HAL_Delay(500); // Pequeña espera para no pulsar dos veces sin querer
	              }
	              break;


		  // ============================================================
		  // 2. SELECCIÓN DE NIVEL (Normal vs Loco)
		  // ============================================================
		  case ESTADO_SELECCION:
			  ssd1306_Fill(Black);
			  ssd1306_SetCursor(20, 5);
			  ssd1306_WriteString("ELIGE MODO:", Font_7x10, White);

			  ssd1306_SetCursor(30, 25);
			  ssd1306_WriteString("NIVEL NORMAL", Font_7x10, White);

			  ssd1306_SetCursor(30, 45);
			  ssd1306_WriteString("NIVEL LOOOCO", Font_7x10, White);

			  // Dibujar la flecha >
			  if (opcionMenu == 0) {
				  ssd1306_SetCursor(15, 25);
				  ssd1306_WriteString(">", Font_7x10, White);
			  } else {
				  ssd1306_SetCursor(15, 45);
				  ssd1306_WriteString(">", Font_7x10, White);
			  }
			  ssd1306_UpdateScreen();

			  // Botones B (Arriba) y C (Abajo)
			  if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1) == GPIO_PIN_RESET) opcionMenu = 0; // Normal
			  if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == GPIO_PIN_RESET) opcionMenu = 1; // Loco

			  // Botón A (Confirmar)
			  if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0) == GPIO_PIN_RESET) {
				  modoJuego = opcionMenu; // Guardamos la elección (0 o 1)
				  estadoActual = ESTADO_CUENTA;
				  HAL_Delay(300);
			  }
			  break;


		  // ============================================================
		  // 3. CUENTA ATRÁS DRAMÁTICA
		  // ============================================================
		  case ESTADO_CUENTA:
			  ssd1306_Fill(Black);
			  ssd1306_SetCursor(40, 20);
			  ssd1306_WriteString("3...", Font_16x26, White);
			  ssd1306_UpdateScreen();
			  HAL_Delay(800);

			  ssd1306_Fill(Black);
			  ssd1306_SetCursor(40, 20);
			  ssd1306_WriteString("2...", Font_16x26, White);
			  ssd1306_UpdateScreen();
			  HAL_Delay(800);

			  ssd1306_Fill(Black);
			  ssd1306_SetCursor(40, 20);
			  ssd1306_WriteString("1...", Font_16x26, White);
			  ssd1306_UpdateScreen();
			  HAL_Delay(800);

			  ssd1306_Fill(Black);
			  ssd1306_SetCursor(50, 5);
			  ssd1306_WriteString("NO TE", Font_11x18, White);
			  ssd1306_SetCursor(15, 25);
			  ssd1306_WriteString("CAIGAAAAS!!!", Font_11x18, White);
			  ssd1306_UpdateScreen();
			  HAL_Delay(1000);

			  estadoActual = ESTADO_JUGANDO;
			  break;


		  /// ============================================================
		  // 4. JUEGO REAL (LÓGICA < 200)
		  // ============================================================
		  case ESTADO_JUGANDO:
			  // --- 1. LEER LOS 5 CANALES DEL ADC EN ORDEN ---
			  HAL_ADC_Start(&hadc1);
			  // Leemos uno a uno
			  if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) val_joyX = HAL_ADC_GetValue(&hadc1);
			  if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) val_joyY = HAL_ADC_GetValue(&hadc1);
			  if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) val_LDR_Inicio = HAL_ADC_GetValue(&hadc1);
			  if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) val_LDR_Fin = HAL_ADC_GetValue(&hadc1);
			  if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) val_LDR_Perder = HAL_ADC_GetValue(&hadc1);
			  HAL_ADC_Stop(&hadc1);


			  // --- 2. COMPROBAR VICTORIA / DERROTA ---

			  // A) META (Ganar) - Si baja de 200
			  if (val_LDR_Fin < 200) {
				  estadoActual = ESTADO_GANADO;
			  }

			  // B) TRAMPA (Perder)
			  // AQUÍ ESTÁ TU CAMBIO: Si baja de 200 -> PIERDES
			  if (val_LDR_Perder < 200) {
				  estadoActual = ESTADO_PERDIDO;
			  }

			  // Botón D (Reset de emergencia)
			  if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == GPIO_PIN_RESET) {
				   estadoActual = ESTADO_INTRO;
				   HAL_Delay(500);
			  }


			  // --- 3. MOVER SERVOS (NORMAL vs LOCO) ---
			  if (modoJuego == 1) {
				  // MODO LOCO: Invertimos matemáticas (4095 - valor)
				  val_joyX = 4095 - val_joyX;
				  val_joyY = 4095 - val_joyY;
			  }

			  // Fórmulas ajustadas (1150 y 1090)
			  uint32_t pulsoX = 1150 + (val_joyX * 400) / 4095;
			  uint32_t pulsoY = 1090 + (val_joyY * 300) / 4095;

			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulsoX);
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pulsoY);

			  break;

		  // ============================================================
		  // 5. FIN DEL JUEGO
		  // ============================================================
		  case ESTADO_GANADO:
			  ssd1306_Fill(Black);
			  ssd1306_SetCursor(10, 10);
			  ssd1306_WriteString("VICTORIA!!", Font_11x18, White);

			  ssd1306_SetCursor(30, 35);
			  ssd1306_WriteString("Pulsa OK", Font_7x10, White);
			  ssd1306_SetCursor(25, 48);
			  ssd1306_WriteString("para Menu", Font_7x10, White);
			  ssd1306_UpdateScreen();

			  // Botón A para volver
			  if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0) == GPIO_PIN_RESET) {
				  estadoActual = ESTADO_INTRO;
				  HAL_Delay(500);
			  }
			  break;

		  case ESTADO_PERDIDO:
			  ssd1306_Fill(Black);
			  ssd1306_SetCursor(15, 10);
			  ssd1306_WriteString("GAME OVER", Font_11x18, White);

			  ssd1306_SetCursor(15, 35);
			  ssd1306_WriteString("Pulsa OK", Font_7x10, White);
			  ssd1306_SetCursor(25, 48);
			  ssd1306_WriteString("para Menu", Font_7x10, White);
			  ssd1306_UpdateScreen();

			  // Botón A para volver
			  if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0) == GPIO_PIN_RESET) {
				  estadoActual = ESTADO_INTRO;
				  HAL_Delay(500);
			  }
			  break;
	  }
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 5;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 5;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 500;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  //HAL_GPIO_WritePin(LASER_ON_GPIO_Port, LASER_ON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LASER_RX_Pin */
  //GPIO_InitStruct.Pin = LASER_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  //HAL_GPIO_Init(LASER_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LASER_ON_Pin */
  //GPIO_InitStruct.Pin = LASER_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  //HAL_GPIO_Init(LASER_ON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD2 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
