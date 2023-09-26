/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdbool.h"
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

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void UART_println(const char* s);

int16_t read_hall_sensor_Gs();
uint16_t read_sys_voltage_mV();


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);
void handle_hall_sensor(int16_t hall_sensor_reading_Gs);
void handle_voltage_sensor(uint16_t system_voltage_mV);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef enum {
	FSM_STATE_INITIAL,
	FSM_STATE_READ_VOLTAGE,
	FSM_STATE_READ_SENSOR,
	FSM_STATE_PAUSED,
	NUMBER_OF_FSM_STATES
} fsm_state_t;

bool button_pressed = false;
bool should_read_sensor = false;
bool should_read_voltage = false;

fsm_state_t run_state_initial() {
	if(button_pressed) {
		button_pressed = false;
		return FSM_STATE_PAUSED;
	} else if(should_read_voltage) {
		should_read_voltage = false;
		return FSM_STATE_READ_VOLTAGE;
	} else if(should_read_sensor) {
		should_read_sensor = false;
		return FSM_STATE_READ_SENSOR;
	} else {
		return FSM_STATE_INITIAL;
	}

}

fsm_state_t run_state_read_voltage() {
	static const uint16_t low_threshold_mV = 1800, high_threshold_mV = 2700;

	enum {
		UNDERVOLTAGE = -1,
		OK_VOLTAGE = 0,
		OVERVOLTAGE = 1
	} voltage_state =
		system_voltage_mV > high_threshold_mV ? OVERVOLTAGE
		: system_voltage_mV < low_threshold_mV ? UNDERVOLTAGE
		: OK_VOLTAGE;

	if(voltage_state == OVERVOLTAGE) {
		HAL_GPIO_WritePin(GPIOB, OVERVOLTAGE_LED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, UNDERVOLTAGE_LED_Pin, GPIO_PIN_RESET);
	} else if (voltage_state == UNDERVOLTAGE) {
		HAL_GPIO_WritePin(GPIOB, OVERVOLTAGE_LED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, UNDERVOLTAGE_LED_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIOB, OVERVOLTAGE_LED_Pin|UNDERVOLTAGE_LED_Pin, GPIO_PIN_RESET);
	}

	return FSM_STATE_INITIAL;
}

fsm_state_t run_state_read_sensor() {
	handle_hall_sensor(read_hall_sensor_Gs());
	return FSM_STATE_INITIAL;
}

fsm_state_t run_state_paused() {
	UART_println("Board in waiting state - please press the emergency button");
	HAL_Delay(500);

	if(button_pressed) {
		button_pressed = false;
		// reset timer inputs, since we did busy waiting
		should_read_sensor = should_read_voltage = false;
		return FSM_STATE_INITIAL;
	} else {
		return FSM_STATE_PAUSED;
	}
}

fsm_state_t run_state(fsm_state_t s) {
	switch(s) {
	case FSM_STATE_INITIAL:
		return run_state_initial();
	case FSM_STATE_READ_VOLTAGE:
		return run_state_read_voltage();
	case FSM_STATE_READ_SENSOR:
		return run_state_read_sensor();
	case FSM_STATE_PAUSED:
		return run_state_paused();
	default:
		char buf[128];
		snprintf(buf, 128, "invalid fsm state \"%d\" passed to run_state", s);
		UART_println(buf);
		while(1);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  fsm_state_t fsm_state = FSM_STATE_INITIAL;
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	fsm_state = run_state(fsm_state);
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /*
   * clock freq is 84Mhz
   * prescale it by 1k -> prescaled freq is 84kHz
   * period = 420 cycles -> timer frequency = 84kHz / 4200 = 2kHz/100 = 1/5 kHz = 20Hz
   * => period = 50ms
   *
   * enabling AutoReloadPreload allows the timer to restart after it has been triggered
  */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4200;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OVERVOLTAGE_LED_Pin|UNDERVOLTAGE_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OVERVOLTAGE_LED_Pin UNDERVOLTAGE_LED_Pin */
  GPIO_InitStruct.Pin = OVERVOLTAGE_LED_Pin|UNDERVOLTAGE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// prints a null terminated string to uart2, followed by a CR and LF
void UART_println(const char* s) {
	static const char* CRLF = "\r\n";
	unsigned char buf[256];
	size_t len = strlen(s);
	memcpy(buf, s, len);
	memcpy(buf+len, CRLF, 2);

	HAL_UART_Transmit(&huart2, buf, len+2, 10);
}

static uint16_t ADC_reading_to_mV(uint16_t a) {
    static const uint16_t Vref_mV = 3300, max_ADC_reading = 1 << 12;
    return a * Vref_mV / max_ADC_reading;
}

static int16_t hall_sensor_reading_mV_to_magnetic_field_Gs(uint16_t a) {
	//2500mV is the quiescent output voltage, which is the voltage the sensor outputs when no magnetic field is detected (=> 0GS)
	//1.6 is the output voltage sensitivity, measured in mV/Gs. it correlates the strength of the magnetic field with the voltage output of the sensor
	return ((int16_t)a - 2500) * 10 / 16;
}

int16_t read_hall_sensor_Gs() {
	HAL_ADC_Start(&hadc1);
	uint16_t hall_sensor_reading = HAL_ADC_GetValue(&hadc1);
	uint16_t hall_sensor_voltage_mV = ADC_reading_to_mV(hall_sensor_reading);
	int16_t magnetic_field = hall_sensor_reading_mV_to_magnetic_field_Gs(hall_sensor_voltage_mV);
	return magnetic_field;
}

uint16_t read_sys_voltage_mV() {
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1);
	uint16_t ignored_reading = HAL_ADC_GetValue(&hadc1);
	UNUSED(ignored_reading);

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1);
	uint16_t ADC_reading = HAL_ADC_GetValue(&hadc1);
	uint16_t sys_voltage_mV = ADC_reading_to_mV(ADC_reading);

	return sys_voltage_mV;
}

// timer callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if(htim == &htim2) {
		//executes every 50ms
		static uint32_t timer_period_counter = 0;

		static const uint32_t timer_period_ms = 50;
		static const uint32_t sys_voltage_period_ms = 350;
		static const uint32_t sensor_period_ms = 200;

		static const uint32_t timer_periods_to_wait_before_checking_sys_voltage = sys_voltage_period_ms / timer_period_ms;
		static const uint32_t timer_periods_to_wait_before_checking_sensor = sensor_period_ms / timer_period_ms;

		if(timer_period_counter % timer_periods_to_wait_before_checking_sys_voltage == 0)
			should_read_voltage = true;
		if(timer_period_counter % timer_periods_to_wait_before_checking_sensor == 0)
			should_read_sensor = true;

		timer_period_counter++;
	}
}

void handle_hall_sensor(int16_t hall_sensor_reading_Gs) {
	char buf[128];

	if (hall_sensor_reading_Gs < -1000) {
    	snprintf(buf, 128, "Hall sensor reading was out of the expected range (%d Gs < -1000Gs)", hall_sensor_reading_Gs);
    } else if (hall_sensor_reading_Gs > 1000) {
    	snprintf(buf, 128, "Hall sensor reading was out of the expected range (%d Gs > 1000Gs)", hall_sensor_reading_Gs);
    } else {
    	//the value read by the sensor is within the expected range
		snprintf(buf, 128, "Hall sensor  %d Gs", hall_sensor_reading_Gs);
    }

	UART_println(buf);
}


// callback for button push
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin == B1_Pin) {
		//toggle the pause state of the system
		button_pressed = true;
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  UART_println("Error occurred");
  __disable_irq();
  while(1);
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
  char buf[256];
  snprintf(buf, 256, "Wrong parameters value: file %s on line %d\r\n", file, line);
  UART_println(buf);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
