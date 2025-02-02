/*
 * Nicole and some other people
 * Martin too
 * */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "rtos.h"
#include "canDefinitions.h"

#define STACK_MAX_VOLTAGE 4200
#define STACK_MIN_VOLTAGE 2700

#define STACK_MAX_TEMP 600

CAN_HandleTypeDef hcan;
SPI_HandleTypeDef hspi1;

#define HALF_SEGMENTS 10

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI1_Init(void);

static void processData(); /* processes all cell data */
static void segmentCS(uint8_t board_id); /* selects the mux channel */
static void sendCAN(); /* sends all segment data onto can */
static void sendCANVerbose(); /* sends all segment data onto can */
static void copyData(); /* copies data from the cells */

#define MODE_FAULT      0
#define MODE_NORMAL 	1
#define MODE_CHARGING 	2

kernel rtos_scheduler = {0, -1, {{0, 0, 0}}, 0, 0, {{0, 0, 0}}, 0, 0, {{0, 0, 0}}};

/*
 * SPI data
 */
struct _SPI_Message {
	uint8_t boardID;
	uint16_t highestVoltage;
	uint16_t avgVoltage;
	uint16_t lowestVoltage;
	uint16_t highestTemp;
	uint16_t avgTemp;
	uint16_t lowestTemp;
} SPI_Message[HALF_SEGMENTS];

struct _SPI_Control {
	uint8_t mode;
	uint16_t lowestVoltage;
	uint8_t _RESERVED[10];
} SPI_Control = {0, 3500, {0}};

uint8_t STATE_NORMAL;
uint8_t STATE_CHARGING;
uint8_t STATE_FAULT;

/*
 * TODO: charging logic ?
 * */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

}


void initCharging() {SPI_Control.mode = MODE_NORMAL;}
void initNormal() {SPI_Control.mode = MODE_CHARGING;}
void initFault() {
	SPI_Control.mode = MODE_FAULT;

	/*
	 * do stuff like open shutdown
	 * */
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_CAN_Init();
	MX_SPI1_Init();

	RTOS_init();

	/* TODO: should there be */
	STATE_NORMAL = RTOS_addState(initNormal, 0);
	STATE_CHARGING = RTOS_addState(initCharging, 0);
	STATE_FAULT = RTOS_addState(initFault, 0);

	RTOS_scheduleTask(STATE_NORMAL, copyData, 20);
	RTOS_scheduleTask(STATE_CHARGING, copyData, 20);
	RTOS_scheduleTask(STATE_FAULT, copyData, 20);

	RTOS_scheduleTask(STATE_NORMAL, processData, 20);
	RTOS_scheduleTask(STATE_CHARGING, processData, 20);

	RTOS_scheduleTask(STATE_NORMAL, sendCAN, 100);
	RTOS_scheduleTask(STATE_CHARGING, sendCAN, 100);
	RTOS_scheduleTask(STATE_FAULT, sendCAN, 100);

	SysTick_Config(48000);

	/* TODO: timer setup */

	/* TODO: idk how the shutdown works, i need to check this lol */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	while (1) {
		RTOS_ExecuteTasks();
	}
}




/* Select the segment to be read and written to
 *
 * Takes in: board_id, unsigned integer
 * Modifies state: writes an integer to the top 4 bits of GPIOB
 *
 *  */
void segmentCS(uint8_t board_id) {
	GPIOB->ODR &= ~(0b1111 << 4);
	GPIOB->ODR |= board_id << 4;
}

/*
 * Dumps all segment data onto the CAN bus
 * */
void sendCAN(){
	CAN_TxHeaderTypeDef CAN_header = {
			.StdId = 0,
			.ExtId = 0,
			.IDE = CAN_ID_STD,
			.RTR = CAN_RTR_DATA,
			.DLC = sizeof(BMS_Status),
			.TransmitGlobalTime = DISABLE
	};
	uint32_t TxMailbox = 0;
	for (int i=0; i < HALF_SEGMENTS; i++){
		CAN_header.StdId = (BMS_CANID_DATA_0 + i);
		BMS_Status CANData = {
			(SPI_Message[i].highestVoltage / 4),
			(SPI_Message[i].lowestVoltage / 4),
			(SPI_Message[i].avgVoltage / 4),
			(SPI_Message[i].highestTemp),
			(SPI_Message[i].lowestTemp),
			(SPI_Message[i].avgTemp),
			0
		}; 
		HAL_CAN_AddTxMessage(&hcan, &CAN_header, (uint8_t *)&CANData, &TxMailbox);
	}
}

/*
 * Destroys the vehicles can bus
 */
void sendCANVerbose(){

}

/*
 * processes data received from cells, controls shutdown, etc.
 * */
void processData() {
	uint16_t newLowestVoltage = 65535;

	for(int i = 0; i < HALF_SEGMENTS; i++){
		if (SPI_Message[i].lowestTemp < newLowestVoltage)
			newLowestVoltage = SPI_Message[i].lowestTemp;

		if(SPI_Message[i].highestVoltage > STACK_MAX_VOLTAGE ||
			SPI_Message[i].lowestVoltage < STACK_MIN_VOLTAGE){
			RTOS_switchState(STATE_FAULT);
		}

		if(SPI_Message[i].highestTemp > STACK_MAX_TEMP){
			RTOS_switchState(STATE_FAULT);
		}
	}

	SPI_Control.lowestVoltage = newLowestVoltage;
}

/**
 * copies the data from the cells
 */
void copyData(){
	for (int i = 0; i < HALF_SEGMENTS; i++){
		segmentCS(i); /* select segment to read */

		HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&SPI_Control, (uint8_t*)&SPI_Message[i], sizeof(SPI_Control), 65535);
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

	/** Initializes the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure.
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
	Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			      |RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
	Error_Handler();
	}
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{
	hcan.Instance = CAN;
	hcan.Init.Prescaler = 48;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK)
	{
	Error_Handler();
	}

	CAN_FilterTypeDef canfilterconfig;
	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 10;
	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfilterconfig.FilterIdHigh = 0xE5<<5;
	canfilterconfig.FilterIdLow = 0x0000;
	canfilterconfig.FilterMaskIdHigh = 0xE5<<5;
	canfilterconfig.FilterMaskIdLow = 0x0000;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;

	HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
	Error_Handler();
	}
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5
			  |GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

	/*Configure GPIO pins : PA0 PA1 */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB1 PB4 PB5
			   PB6 PB7 */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5
			  |GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
  while (1){

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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
