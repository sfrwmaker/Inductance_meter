/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hd44780.h"
#include <math.h>
#include <stdlib.h>

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
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
const double	cal_cap_uF	= 0.001066;									// The calibration capacitance (micro farads)
const double	four_pi_sq	= 39.4784176043574344753379639995;			// 4 * PI^2
const uint32_t 	OPEN_TICKS  = 330000;            						// The sum of ticks in the pulse[] when the circuit is open

// The buffer for pulse data
#define			BUFF_SZ		8
volatile static	uint16_t	pulse[BUFF_SZ];
volatile static uint8_t		pulse_index = 0;
volatile static	uint8_t		pulse_ready = 0;							// The whole buffer has been filled
static double				ticks_LC	= 0.0;
static double				ticks_LCC	= 0.0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// Capture the signal from the timer. 48 ticks per microseconds
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim3) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			pulse[pulse_index] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			if (++pulse_index >= 8) {
				pulse_index = 0;
				pulse_ready = 1;
			}
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) { }

// Absolute difference of two positive values
uint16_t abs_diff(uint16_t a, uint16_t b) {
    if (a >= b) {
        return a - b;
    } else {
        return b - a;
    }
}

// Oscillation period, ticks
uint32_t oscTicksSumm(uint8_t reset) {
	if (reset) {
		pulse_ready = pulse_index = 0;									// Clear up the buffer
		while (!pulse_ready) ;											// Wait till the whole buffer has been filled up
	}
	uint32_t summ = 0;
	for (uint8_t i = 0; i < 10; ++i) {
		summ	= 0;
		uint16_t min_v	= 65535;
		uint16_t max_v	= 0;
		for (uint8_t i = 0; i < BUFF_SZ; ++i) {
			uint16_t p = pulse[i];
			if (p > max_v) max_v = p;
			if (p < min_v) min_v = p;
			summ += p;
		}
		if (abs_diff(min_v, max_v) < 100) break;
		lcdClrScr();
		pulse_ready = pulse_index = 0;									// Clear up the buffer
		HAL_Delay(1000);												// Wait till the whole buffer has been filled up
	}
	return summ;
}

// The square of the period, microseconds^2. Argument - period in ticks
double periodSquare(double P) {
    double ticks_per_mks = (double)HAL_RCC_GetHCLKFreq() / 1000000.0;  	// The number of timer ticks per one mirco second
    P /= ticks_per_mks;                         						// Convert ticks to micro seconds
    P *= P;
    return P;
}

// Frequently used coefficient:  (P2^2 - P1^2) / P1^2
double pCoeff(double P1, double P2) {
    P1 *= P1;
    P2 *= P2;
    double r  = (P2 - P1) / P1;
    if (r < 0) r = 0;
    return r;
}

void calibrate(void) {
    while (1) {
    	HAL_GPIO_WritePin(SHORT_GPIO_Port, SHORT_Pin, GPIO_PIN_SET);	// Short the external socket
    	HAL_Delay(100);
        ticks_LC = (double)oscTicksSumm(1) / (double)BUFF_SZ;			// Period of the internal oscillation circuit, ticks
        HAL_GPIO_WritePin(CALIB_GPIO_Port, CALIB_Pin, GPIO_PIN_SET);	// Connect the calibration capacitor
        HAL_Delay(100);
        ticks_LCC = (double)oscTicksSumm(1) / (double)BUFF_SZ;				// Period of the internal oscillation circuit + calibration capacitor, ticks
        HAL_GPIO_WritePin(CALIB_GPIO_Port, CALIB_Pin, GPIO_PIN_RESET);	// Disconnect the calibration capacitor
        HAL_GPIO_WritePin(SHORT_GPIO_Port, SHORT_Pin, GPIO_PIN_RESET);	// Open the external socket
        HAL_Delay(200);
        if (ticks_LCC > ticks_LC) break;
        lcdClrScr();
        HAL_Delay(2000);
    }
    while (!pulse_ready) ;												// Wait till the whole buffer has been filled up
    //volatile double int_cap	= cal_cap_uF / pCoeff(ticks_LC, ticks_LCC);
}

void printValue(double value) {
    char buff[3]	= "xH";
    buff[0] = 'u';
    if (value > 1000.0) {
        value /= 1000;
        buff[0] = 'm';
    }

    uint8_t value_size = 3;
    if (value >= 100.0)
    	value_size = 5;
    else if (value >= 10.0)
    	value_size = 4;

    lcdGoto(LCD_1st_LINE, 0);
    lcdPuts("Induct.");
    lcdGoto(LCD_2nd_LINE, 0);
    for (uint8_t i = 0; i < 6-value_size; ++i)
    	lcdPutc(' ');
    lcdFtos(value, 1);
    lcdPuts(buff);
}

void checkInductance(void) {
    static uint16_t ticks_prev	 = 0;									// previous Period of the oscillation, including external inductance or capacitance, ticks
    static uint32_t calibrate_ms = 0;

    uint32_t ticks_new	= oscTicksSumm(0);								// Period of the internal oscillation circuit plus external thing, ticks
    if (ticks_new >= OPEN_TICKS) {
    	lcdClrScr();
        lcdPuts(" NO");
        lcdGoto(LCD_2nd_LINE, 0);
        lcdPuts("value");
    	HAL_Delay(1000);
        return;
    }

    double ticks 	= (double)ticks_new / (double)BUFF_SZ;
    double pi_coeff	= 1 / (four_pi_sq * cal_cap_uF);					// 1 / (4 * PI^2 * Cc)
    double int_ind	= (periodSquare(ticks_LCC) - periodSquare(ticks_LC)) * pi_coeff;
    double ext_ind	= int_ind * pCoeff(ticks_LC, ticks);

    printValue(ext_ind);
    if (HAL_GetTick() >= calibrate_ms || (ticks_prev && abs_diff(ticks_new, ticks_prev) > (BUFF_SZ >> 1))) {
        calibrate_ms = HAL_GetTick() + 10000;							// Do calibration every 10 seconds
        calibrate();
    }
    ticks_prev = ticks_new;
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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(500);
  lcdInit();

  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  lcdClrScr();
  calibrate();

  while (1)
  {
	  checkInductance();
	  HAL_Delay(1000);
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

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
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
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin 
                          |SHORT_Pin|LCD_E_Pin|CALIB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_D4_Pin LCD_D5_Pin LCD_D6_Pin LCD_D7_Pin 
                           SHORT_Pin LCD_E_Pin CALIB_Pin */
  GPIO_InitStruct.Pin = LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin 
                          |SHORT_Pin|LCD_E_Pin|CALIB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_RS_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_RS_GPIO_Port, &GPIO_InitStruct);

  /**/
  HAL_I2CEx_EnableFastModePlus(SYSCFG_CFGR1_I2C_FMP_PA9);

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
  while(1)
  {
  }
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
