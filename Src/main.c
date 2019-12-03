/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "vl6180x_api.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define leftSensor  0x52
#define rightSensor  0x62

#define LEFT_RESET_PORT GPIOA
#define RIGHT_RESET_PORT GPIOA
#define LEFT_RESET_PIN GPIO_PIN_2
#define RIGHT_RESET_PIN GPIO_PIN_1

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void VL6180x_Reset(VL6180xDev_t dev, GPIO_PinState state);
int  VL6180x_I2CWrite(VL6180xDev_t dev, uint8_t  *buff, uint8_t len);
void CUSTOM_GPIO_Init(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void delay(int ms)
{
	uint32_t v = HAL_GetTick() + ms;
	while(HAL_GetTick() < v){
		__nop();
	}
}
void setMotorPwm(int sign, VL6180x_RangeData_t range, uint32_t channel)
{
	 int basePwm = 1340;
	 if(range.range_mm < 150) {
		 basePwm += sign * (500 - range.range_mm *3);
	 }
	 __HAL_TIM_SET_COMPARE(&htim1,channel,basePwm);
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	VL6180x_RangeData_t rangeLeft;
	VL6180x_RangeData_t rangeRight;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
	CUSTOM_GPIO_Init();
  VL6180x_Reset(leftSensor, GPIO_PIN_RESET);
  VL6180x_Reset(rightSensor, GPIO_PIN_RESET);
  delay(10);
	VL6180x_Reset(rightSensor, GPIO_PIN_SET);
  delay(1);

	VL6180x_WaitDeviceBooted(leftSensor);
	VL6180x_DisableGPIOxOut(leftSensor, 1);
	VL6180x_SetI2CAddress(leftSensor, rightSensor);
	
  VL6180x_InitData(rightSensor);
	

	VL6180x_Reset(leftSensor, GPIO_PIN_SET);
  delay(1);
  VL6180x_WaitDeviceBooted(leftSensor);
	VL6180x_DisableGPIOxOut(leftSensor, 1);
  VL6180x_InitData(leftSensor);


    /* Enable Dmax calculation only if value is displayed (to save computation power) */
  VL6180x_DMaxSetState(leftSensor, true);
  VL6180x_DMaxSetState(rightSensor, true);
	VL6180x_Prepare(leftSensor);
	VL6180x_Prepare(rightSensor);
	
	
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		VL6180x_RangePollMeasurement(rightSensor, &rangeRight);
		VL6180x_RangePollMeasurement(leftSensor, &rangeLeft);
		int minRange = rangeLeft.range_mm < rangeRight.range_mm ? rangeLeft.range_mm : rangeRight.range_mm;
 		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, minRange < 200 ? (1000 - minRange * 5 ): 0);
		setMotorPwm(-1, rangeRight, TIM_CHANNEL_1);
		setMotorPwm(1, rangeLeft, TIM_CHANNEL_3);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C2 init function */
void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x2000090E;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c2);

    /**Configure Analogue filter 
    */
  HAL_I2CEx_AnalogFilter_Config(&hi2c2, I2C_ANALOGFILTER_ENABLED);

}

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim1);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim1);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1340;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim3);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 120;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
 *
 * @brief       Read data buffer from VL6180x device via i2c
 * @param dev   The device to read from
 * @param buff  The data buffer to fill
 * @param len   The length of the transaction in byte
 * @return      0 on success
 * @ingroup  cci_i2c
 */
void CUSTOM_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}
int VL6180x_I2CRead(VL6180xDev_t dev, uint8_t *buff, uint8_t len) {
		HAL_StatusTypeDef res =  HAL_I2C_Master_Receive(&hi2c2, dev, buff, len, 20000);
	return res == HAL_OK ? 0 : -1;
}

void VL6180x_PollDelay(VL6180xDev_t dev) {}
void VL6180x_Reset(VL6180xDev_t device, GPIO_PinState state)
{
	if(device == leftSensor) {
		HAL_GPIO_WritePin(LEFT_RESET_PORT, LEFT_RESET_PIN, state);
	} else {
		HAL_GPIO_WritePin(RIGHT_RESET_PORT, RIGHT_RESET_PIN, state);
	}
}

/**
 * @brief       Write data buffer to VL6180x device via i2c
 * @param dev   The device to write to
 * @param buff  The data buffer
 * @param len   The length of the transaction in byte
 * @return      0 on success
 * @ingroup cci_i2c
 */
int  VL6180x_I2CWrite(VL6180xDev_t dev, uint8_t  *buff, uint8_t len) {
	HAL_StatusTypeDef res =  HAL_I2C_Master_Transmit(&hi2c2, dev, buff,len, 20000);
	return res == HAL_OK ? 0 : -1;
}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
