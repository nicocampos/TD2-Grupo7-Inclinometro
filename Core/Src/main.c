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
#include "../planificador/miniplanificador.h"
#include "mpu_6050.h"
#include "ssd1306.h"
#include "fonts.h"
#include "stdio.h"
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define dwt_init() 			{DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; DWT->CYCCNT=0;}
#define dwt_reset() 		{DWT->CYCCNT=0;}
#define dwt_read() 			(DWT->CYCCNT)

#define MODE_KEY			GPIOB, inputMode_Pin
#define HOLD_KEY			GPIOB, inputHold_Pin
#define ZERO_KEY			GPIOB, inputZero_Pin

#define MODE				1
#define ZERO				2
#define HOLD				3

#define CANT_MODOS			3
#define	NO_KEY				0

#define MPU6050_ADDR 		0xD0
#define MAX_LEN_TASK_LIST	(8)
#define TICK_SISTEMA		(10)
#define TIMEOUT_I2C			10
#define PROMEDIO			50
#define POS_GRADOS			10,30	//30,30
#define POS_GRADOSX			10,15
#define POS_GRADOSY			10,30

// ESTADOS - MODOS
#define MODO_MEDIR			0
#define MODO_NIVEL			1
#define MODO_ALARMAS		2

#define ON					1
#define OFF					0

#define THRESHOLD			0.13

//#define POS_SIGNO			5,35
//#define __SET_IWDG


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
#ifdef __SET_IWDG
IWDG_HandleTypeDef hiwdg;
#endif
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
#ifdef __SET_IWDG
static void MX_IWDG_Init(void);
#endif
static void MX_I2C1_Init(void);
void MPU6050_Init(void);
void start_timer(void);
uint32_t stop_timer(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */
uint8_t antirebote (uint8_t lectura_actual);
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
MPU6050_t MPU6050;

float offset_x = 0;

float offset_y = 0;

uint8_t tecla = NO_KEY;

uint8_t f_hold = OFF;

uint8_t f_zero = OFF;

uint8_t fEjeY = OFF;

/*
 * Variables Para pasar datos entre tareas.
 */
uint32_t ticks;
/*
 * Lista de tareas del planificador.
 */
TaskStat lista_tareas[MAX_LEN_TASK_LIST];

/*
 * Prototipos de las funciones que hacen al sistema.
 */
void falla_sistema(void);
void tarea_iwdg(void *p);
void tarea_led_blinking(void *p);
void tarea_display(void *p);
void tarea_orienta(void *p);
void tarea_refresh(void *p);
void tarea_pulsadores(void *p);
void tarea_modos(void *p);

/* USER CODE END 0 */


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t tics_despachador;
	uint32_t wcet_todo = 0;

	/*
	 * Pongo en diez la variable que cuenta la cantidad de ticks que tienen
	 * que pasar entre llamadas del despachador.
	 */
	ticks = TICK_SISTEMA;

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
  /*
   * Inicializo todo el hardware.
   */
  dwt_init();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MPU6050_Init();
  SSD1306_Init();
  /* USER CODE BEGIN 2 */
	#ifdef __SET_IWDG
		MX_IWDG_Init();
	#endif
  /* USER CODE END 2 */
	HAL_GPIO_WritePin(GPIOC, Led_Blink_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, outputLed_Pin, GPIO_PIN_RESET);



	/*
	 * Uso el timer 2 para el monitor del sistema.
	 */
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	/*
	 * Esta línea configura el prescaler del timer
	 * que cuenta tiempo del procesador. Prestar atención.
	 */
	TIM2->PSC = (SystemCoreClock / 1000000) - 1;
	TIM2->CNT = -1;
	TIM2->CR1 |= TIM_CR1_CEN;
	TIM2->CR1 &= ~TIM_CR1_CEN;
  /*
	 * Inicializo el despachador de las tareas. Le tengo que pasar
	 * la lista de tareas, dos punteros a función: uno para inicializar
	 * un timer y otro para que me devuelva la cuenta y lo frene. El
	 * puntero restante es un puntero a función que se llama cuando
	 * hay falla en la medición de tiempos de las funciones.
	 *
	 */
	inicializar_despachador(lista_tareas,
	MAX_LEN_TASK_LIST, start_timer, stop_timer, falla_sistema);

	//agregar_tarea(lista_tareas, tarea_iwdg, NULL, 0, 1, 0, 100000);
	agregar_tarea(lista_tareas, tarea_led_blinking, NULL, 0, 10, 0, 4); // et_wcet = 2
	agregar_tarea(lista_tareas, tarea_orienta, NULL, 0, 1, 0, 100000);   	// et_wcet = 926
    agregar_tarea(lista_tareas, tarea_refresh, NULL, 0, 1, 0, 100000);  	// et_wcet = 3632
    agregar_tarea(lista_tareas, tarea_pulsadores, NULL, 0, 10, 0, 100000);  	// et_wcet =
    agregar_tarea(lista_tareas, tarea_modos, NULL, 0, 10, 0, 100000);  	// et_wcet =
    agregar_tarea(lista_tareas, tarea_display, NULL, 0, 1, 0, 100000); 	// et_wcet = 7876

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		dwt_reset();
		if (!ticks)
		{
			ticks = TICK_SISTEMA;
			despachar_tareas();
		}
		tics_despachador = dwt_read();
		if (tics_despachador > wcet_todo)
			wcet_todo = tics_despachador;
	}
  /* USER CODE END 3 */
}

////////////////////////////
//	 FUNCION ANTIREBOTE	  //
////////////////////////////

// Recibe el Pin donde esta el pulsador, y el tiempo en ticks debido a la funcion xTaskGetTickCount().
// Veo el pulsador hasta detectar pulso por primera vez, si pasados los 50ms sigue presionado, retorno un 1.
// Si no hay tecla presionada o si no pasaron 50ms desde la primera deteccion devuelvo un 0.

uint8_t antirebote (uint8_t teclaActual)
{
	static uint8_t teclaAnt = NO_KEY;
	static uint8_t cont = 0;

	if(cont == 0){
		teclaAnt = teclaActual;
		cont++;
	}
	else{
		cont = 0;
		if(teclaActual == teclaAnt){
			return teclaAnt;
		}
		else{
			return NO_KEY;
		}
	}
	return NO_KEY;
}


// TAREAS

void tarea_pulsadores (void *p)
{

	if(HAL_GPIO_ReadPin(MODE_KEY)){
		tecla = MODE;
	}

	if(HAL_GPIO_ReadPin(ZERO_KEY)){
		tecla = ZERO;
	}

	if(HAL_GPIO_ReadPin(HOLD_KEY)){
		tecla = HOLD;
	}

	if (tecla != NO_KEY)
		tecla = antirebote(tecla);

}

void tarea_modos(void *p)
{
	static uint8_t clean = OFF;
	static uint8_t mode = MODO_MEDIR;

	if(tecla == MODE){
		mode++;
		mode%=CANT_MODOS;
		clean = OFF;
	}

	switch(mode){
	case MODO_MEDIR:
		if(clean == OFF){
			SSD1306_Fill(SSD1306_COLOR_BLACK); /* Clear screen */
			offset_x = 0;
			offset_y = 0;
			clean = ON;
		}
		fEjeY = OFF;
		if(tecla == HOLD){
			f_hold = ~ f_hold; // toggle flag
		}
		if(tecla == ZERO){
			// VER COMO IMPLEMENTAR -- NO LO VOY A PENSAR AHORA -- gg ez
			f_zero = ON;
		}
		break;

	case MODO_NIVEL:
		if(clean == OFF){
			SSD1306_Fill(SSD1306_COLOR_BLACK); /* Clear screen */
			offset_x = 0;
			offset_y = 0;
			clean = ON;
		}
		fEjeY = ON;
		if(tecla == HOLD){
			f_hold = ~ f_hold; // toggle flag
		}
		if(tecla == ZERO){
			// VER COMO IMPLEMENTAR -- NO LO VOY A PENSAR AHORA -- gg ez
			f_zero = ON;
		}
		break;
	case MODO_ALARMAS:
		if(clean == OFF){
			SSD1306_Fill(SSD1306_COLOR_BLACK); /* Clear screen */
			offset_x = 0;
			offset_y = 0;
			clean = ON;
		}
		fEjeY = OFF;
		f_hold = OFF;


		break;
	default:
		mode = MODO_MEDIR;
	}
}

void tarea_display(void *p)
{
	static int cont = 0;				//!< Variable contador para el promedio
	static float prom_x = 0;			//!<
	static float prom_y = 0;			//!<
	static float valorAntX = 0;			//!<
	static float valorAntY = 0;			//!<
	float prom_offset_x = 0;			//!<
	float prom_offset_y = 0;			//!<
	char str_x[5];						//!<
	char str_y[5];						//!<
	char str1[6] = {0};				//!<
	char str2[6] = {0};				//!<

	if(cont >= PROMEDIO){
		cont = 0;
		prom_x /= PROMEDIO;
		prom_y /= PROMEDIO;

		//------------------------------------------------------------------------------------------------
		//--------------------------------------- Chequeo signo -----------------------------------------
		//------------------------------------------------------------------------------------------------
		if(offset_x != 0){
			prom_x = fabs(prom_x);

			if((prom_offset_x) < 0)
				str1[0] = 118; // v
			else
				str1[0] = 94; // ^
		}
		else{
			if(prom_x < 0)
				str1[0] = 118; // v
			else
				str1[0] = 94; // ^
			prom_x = fabs(prom_x);
		}

		if(offset_y != 0){
			prom_y = fabs(prom_y);

			if((prom_offset_y) < 0)
				str2[0] = 118; // v
			else
				str2[0] = 94; // ^
		}
		else{
			if(prom_y < 0)
				str2[0] = 118; // v
			else
				str2[0] = 94; // ^
			prom_x = fabs(prom_x);
		}

		prom_offset_x = prom_x - offset_x;
		prom_offset_y = prom_y - offset_y;

		//------------------------------------------------------------------------------------------------
		//----------------------------------------------- Zero -------------------------------------------
		//------------------------------------------------------------------------------------------------
		if(f_zero == ON){
			offset_x = prom_x;
			offset_y = prom_y;
			f_zero = OFF;
		}

		//------------------------------------------------------------------------------------------------
		//------------------------------------------- THRESHOLD ------------------------------------------
		//------------------------------------------------------------------------------------------------
		if(fabs(prom_offset_x - valorAntX) < THRESHOLD )
			prom_offset_x = valorAntX;
		if(fabs(prom_offset_x - valorAntY) < THRESHOLD )
			prom_offset_y = valorAntY;

		//------------------------------------------------------------------------------------------------
		//-------------------------------------------- Valor Anterior ------------------------------------
		//------------------------------------------------------------------------------------------------
		valorAntX = prom_offset_x;
		valorAntY = prom_offset_y;

		//------------------------------------------------------------------------------------------------
		//---------------------------------------- Redondeo ----------------------------------------------
		//------------------------------------------------------------------------------------------------
		if(fabs(prom_offset_x) >= 10 ){
			sprintf(str_x,"%.2f", roundf(fabs(prom_offset_x) * 100)/100);
			strcat(str1, str_x);
		}
		else{
			sprintf(str_x,"0%.2f", roundf(fabs(prom_offset_x) * 100)/100);
			strcat(str1, str_x);
		}
		if(prom_offset_y >= 10 ){
			sprintf(str_y,"%.2f", roundf(fabs(prom_offset_y) * 100)/100);
			strcat(str2, str_y);
		}
		else{
			sprintf(str_y,"0%.2f", roundf(fabs(prom_offset_y) * 100)/100);
			strcat(str2, str_y);
		}


		//------------------------------------------------------------------------------------------------
		//-------------------------------------- MODO 1 o MODO 2 -----------------------------------------
		//------------------------------------------------------------------------------------------------
		if(fEjeY == OFF){	// Solo eje X
			SSD1306_GotoXY (POS_GRADOS);
			SSD1306_Puts(str1, &Font_16x26, 1);
		}
		else{	// Solo eje Y
			SSD1306_GotoXY(POS_GRADOSX);
			SSD1306_Puts(str1, &Font_11x18, 1);
			SSD1306_GotoXY(POS_GRADOSY);
			SSD1306_Puts(str2, &Font_11x18, 1);
		}

		prom_x = 0;
		prom_y = 0;
	}
	else{
		prom_x += MPU6050.KalmanAngleX;
		prom_y += MPU6050.KalmanAngleY;
		cont++;
	}


}

void tarea_orienta(void *p)
{
	MPU6050_Read_All(&hi2c1, &MPU6050);
}

void tarea_iwdg(void *p)
{
#ifdef __SET_IWDG
	HAL_IWDG_Refresh(&hiwdg);
#endif
}

void tarea_led_blinking(void *p)
{
	HAL_GPIO_TogglePin(GPIOC, Led_Blink_Pin);
}


void tarea_refresh(void *p)
{
	if(f_hold == OFF)
		SSD1306_UpdateScreen();
}

void falla_sistema(void)
{
	__disable_irq();
	while (1)
	{
		for (uint32_t i = 0; i < 100000; i++);
		HAL_GPIO_TogglePin(GPIOC, Led_Blink_Pin);
	}
}

void start_timer(void)
{
	TIM2->CNT = 0;
	TIM2->CR1 |= TIM_CR1_CEN;
}

uint32_t stop_timer(void)
{
	uint32_t ret = TIM2->CNT;
	TIM2->CR1 &= ~TIM_CR1_CEN;
	return ret;
}

/**
  * @brief MPU6050 Initialization Function
  * @param None
  * @retval None
  */
void MPU6050_Init(void)
{
	uint8_t check, Data;

	// Check device WHO I AM
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_RA_WHO_AM_I, 1, &check, 1, TIMEOUT_I2C);

	if(check == MPU6050_ADDRESS_AD0_LOW){	// if the device is present
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, 1,&Data, 1, TIMEOUT_I2C);
		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_RA_SMPLRT_DIV, 1, &Data, 1, TIMEOUT_I2C);
		// Set accelerometer configuration in MPU6050_RA_ACCEL_CONFIG
		// XA_ST=0, YA_ST=0, ZA_ST=0, FS_SEL=0 -> +/- 2g
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_RA_ACCEL_CONFIG, 1,&Data, 1, TIMEOUT_I2C);
		// Set Gyroscope configuration in MPU6050_RA_GYRO_CONFIG
		// XG_ST=0, YG_ST=0, ZG_ST=0, FS_SEL=0 -> +/- 250°/s
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_RA_GYRO_CONFIG, 1,&Data, 1, TIMEOUT_I2C);
	}
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
#ifdef __SET_IWDG
/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 1300;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}
#endif

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Led_Blink_GPIO_Port, Led_Blink_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(outputLed_GPIO_Port, outputLed_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Led_Blink_Pin */
  GPIO_InitStruct.Pin = Led_Blink_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Led_Blink_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : inputHold_Pin inputZero_Pin inputMode_Pin */
  GPIO_InitStruct.Pin = inputHold_Pin|inputZero_Pin|inputMode_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : outputLed_Pin */
  GPIO_InitStruct.Pin = outputLed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(outputLed_GPIO_Port, &GPIO_InitStruct);

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
