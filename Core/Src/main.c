/*
 * Nicole and some other people
 * Martin too
 * */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "rtos.h"
#include "canDefinitions.h"

/*
 * Feature Enable Flags
 */
#define CHARGER_CONTROL_ENABLED 0

#define STACK_MAX_VOLTAGE 4200
#define STACK_MIN_VOLTAGE 2700

#define STACK_MAX_TEMP 600

#define CHARGER_DERATE_STACK_TEMP 500
#define CHARGER_MAX_VOLTAGE 4500
#define CHARGER_MAX_CURRENT 100

#define MAX_FAULT_COUNT 5
#define FAULT_RESET_COUNT 50

CAN_HandleTypeDef hcan;
SPI_HandleTypeDef hspi1;

#define HALF_SEGMENTS 10

#define BMS_STATUS_PIN 1
#define CHARGER_ENABLE_PIN 0

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
//static void MX_CAN_Init(void);
static void MX_SPI1_Init(void);
static void CAN_Init(void);

static void processLoopNormal(); /* check for persistent fault to switch to fault state */
static void processLoopFault();	/* check for absence of faults to switch to normal state */
static uint8_t processData(); /* processes all cell data, returns fault condition*/

static void segmentCS(uint8_t board_id); /* selects the mux channel */
static void sendCANVerbose(); /* sends all segment data onto can */
static void sendCANStatus(); /* sends BMS_Status CAN message */
static void chargerCtrl(); /* send CAN message to control Elcon charger */
static void copyData(); /* copies data from the cells */

/* sends a can message */
static void sendCAN(uint16_t id, uint8_t * data, uint8_t length);

/*
 * The mode that gets sent to the segments
 */
#define MODE_FAULT      0
#define MODE_NORMAL 	1
#define MODE_CHARGING 	2

kernel rtos_scheduler = {0, -1, {{0, 0, 0}}, 0, 0, {{0, 0, 0}}, 0, 0, {{0, 0, 0}}};

/*
 * SPI data
 */
struct _SPI_Message {
	uint16_t highestVoltage;
	uint16_t avgVoltage;
	uint16_t lowestVoltage;
	uint16_t highestTemp;
	uint16_t avgTemp;
	uint16_t lowestTemp;
} SPI_Message[HALF_SEGMENTS];

struct _SPI_Control {
	uint16_t mode;
	uint16_t lowestVoltage;
	uint8_t _RESERVED[sizeof(SPI_Message[0]) - 4];
} SPI_Control = {0, 3500,  {0,1,2,3,4,5,6,7}};

BMS_FaultStatus FaultStatusMsg;

/**
 * handles to the rtos states
 */
uint8_t STATE_NORMAL;
uint8_t STATE_CHARGING;
uint8_t STATE_FAULT;

/*
 * TODO: charging logic ?
 * */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	/* TODO: filter out*/
}


void initCharging() {
	GPIOA->ODR |= (1UL << BMS_STATUS_PIN);
	SPI_Control.mode = MODE_NORMAL;
	FaultStatusMsg.FaultStatus = 0;
}

void initNormal() {
	GPIOA->ODR |= (1UL << BMS_STATUS_PIN);
	SPI_Control.mode = MODE_CHARGING;
	FaultStatusMsg.FaultStatus = 0;
}

void initFault() {
	chargerCtrl();	/* turns off charger */
	SPI_Control.mode = MODE_FAULT;
	GPIOA->ODR &= ~(1UL << BMS_STATUS_PIN);
	FaultStatusMsg.FaultStatus = 1;
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	//MX_CAN_Init();
	CAN_Init();
	MX_SPI1_Init();

	RTOS_init();

	STATE_NORMAL = RTOS_addState(initNormal, 0);
	STATE_CHARGING = RTOS_addState(initCharging, 0);
	STATE_FAULT = RTOS_addState(initFault, 0);

	/* read data in all modes every 20ms*/
	RTOS_scheduleTask(STATE_NORMAL, copyData, 20);
	RTOS_scheduleTask(STATE_CHARGING, copyData, 20);
	RTOS_scheduleTask(STATE_FAULT, copyData, 20);

	/* process data in non fault modes every 20ms*/
	RTOS_scheduleTask(STATE_NORMAL, processLoopNormal, 20);
	RTOS_scheduleTask(STATE_CHARGING, processLoopNormal, 20);
	RTOS_scheduleTask(STATE_FAULT, processLoopFault, 20);

	/* Send can messages in all modes every 100ms */
	RTOS_scheduleTask(STATE_NORMAL, sendCANStatus, 100);
	RTOS_scheduleTask(STATE_CHARGING, sendCANStatus, 100);
	RTOS_scheduleTask(STATE_FAULT, sendCANStatus, 100);
#if CHARGER_CONTROL_ENABLED == 1
	/* Sends the charger status messages in the charging mode every 500ms */
	RTOS_scheduleTask(STATE_CHARGING, chargerCtrl, 500);
#endif
	SysTick_Config(48000);

	/* TODO: timer setup */

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
	GPIOB->ODR |= (board_id + 1) << 4;
}

/*
 * Send a CAN message
 * Takes in: 
 * id, unsigned integer
 * data, unsigned int pointer
 * length, unsigned integer
 * */
void sendCAN(uint16_t id, uint8_t * data, uint8_t length){
	/* all mailboxes full */
	while(!(CAN->TSR & CAN_TSR_TME_Msk));

	/* find first empty mailbox */
	int j = (CAN->TSR & CAN_TSR_CODE_Msk) >> CAN_TSR_CODE_Pos;

	/* set dlc to length */
	CAN->sTxMailBox[j].TDTR = length;

	/* clears data high/low registers */
	CAN->sTxMailBox[j].TDLR = 0;
	CAN->sTxMailBox[j].TDHR = 0;

	/* writes to high/low registers */
	for(int i = 0; i < length && i < 4; i++)
		CAN->sTxMailBox[j].TDLR |= ((data[i] & 0xFF) << i * 8);
	for(int i = 0; i < length - 4; i++)
		CAN->sTxMailBox[j].TDHR |= ((data[i+4] & 0xFF) << i * 8);

	/* writes id and queues message */
	CAN->sTxMailBox[j].TIR = (uint32_t)((id << CAN_TI0R_STID_Pos) | CAN_TI0R_TXRQ);
	}

/*
 * Spits all segment data out onto the can bus, as defined in canDefinitions.h
 */
void sendCANStatus(){
	for (int i = 0; i < HALF_SEGMENTS; i++) {
		//while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);

		BMS_Status data = {
			.maxVolt = SPI_Message[i].highestVoltage / 4,
			.minVolt = SPI_Message[i].lowestVoltage / 4,
			.avgVolt = SPI_Message[i].avgVoltage / 4,
			.maxTemp = SPI_Message[i].highestTemp,
			.minTemp = SPI_Message[i].lowestTemp,
			.avgTemp = SPI_Message[i].avgTemp,
			._RESERVED = 0
		};
		sendCAN(BMS_CANID_DATA_0 + i, (uint8_t*) &data, sizeof(BMS_Status));
	}
	sendCAN(BMS_CANID_FAULTSTATUS, (uint8_t *)&FaultStatusMsg, sizeof(FaultStatusMsg));
}

/*
 * Sends out every cell temp and voltage over CAN
 */
void sendCANVerbose(){
	/*TODO: add verbose CAN headers and implement this function*/
}

/*
 * Controls the Elcon charger when in charging mode
 */
void chargerCtrl() {
	/*TODO: Send Charger_Control CAN message*/
	Charger_Control chargerMsg = {
			.maxChargingCurrent = CHARGER_MAX_CURRENT,
			.maxChargingVoltage = CHARGER_MAX_VOLTAGE,
			.chargerEnable = (RTOS_inState(STATE_CHARGING)),
			._RESERVED = 0
	};
	sendCAN(BMS_CANID_CHARGER_CTRL, (uint8_t*) &chargerMsg, sizeof(Charger_Control));
}

/*
 * Checks for fault conditions and switches to the fault state
 */
void processLoopNormal() {
	static uint8_t faultCounter = 0;
	switch (processData()){
		case 1: faultCounter++;
		case 0: faultCounter--;
	}
	if (faultCounter > MAX_FAULT_COUNT) {
		RTOS_switchState(STATE_FAULT);
	} else if (GPIOA->IDR & 1 << CHARGER_ENABLE_PIN) { 		/* initiate charging */
		RTOS_switchState(STATE_CHARGING);
	} else {
		RTOS_switchState(STATE_NORMAL);
	}
}

/*
 * Checks for absence of fault conditions and switches back to the normal state
 */
void processLoopFault() {
	static uint8_t resetCounter = 0;
	switch (processData()){
		case 1: resetCounter--;
		case 0: resetCounter++;
	}
	if (resetCounter > FAULT_RESET_COUNT) {
		RTOS_switchState(STATE_NORMAL);
	}
}

/*
 * processes data received from cells and returns fault condition
 */
uint8_t processData() {
	uint16_t newLowestVoltage = 65535;
	uint16_t newHighestVoltage = 0;
	uint16_t newLowestTemp = 65535;
	uint16_t newHighestTemp = 0;
	uint16_t sumAvgVolt = 0;
	uint16_t sumAvgTemp = 0;

	for(int i = 0; i < HALF_SEGMENTS; i++){
		sumAvgVolt += SPI_Message[i].avgVoltage;
		sumAvgTemp += SPI_Message[i].avgTemp;

		if (SPI_Message[i].lowestVoltage < newLowestVoltage){
			newLowestVoltage = SPI_Message[i].lowestVoltage;
		}
		if (SPI_Message[i].highestVoltage < newHighestVoltage){
			newHighestVoltage =	SPI_Message[i].highestVoltage;
		}
		if (SPI_Message[i].lowestTemp < newLowestTemp){
			newLowestTemp =	SPI_Message[i].lowestTemp;
		}
		if (SPI_Message[i].highestTemp < newHighestTemp){
			newHighestTemp = SPI_Message[i].highestTemp;
		}
	}

	FaultStatusMsg.packMaxVolt = newHighestVoltage;
	FaultStatusMsg.packMinVolt = newLowestVoltage;
	FaultStatusMsg.packAvgVolt = sumAvgVolt / HALF_SEGMENTS;
	FaultStatusMsg.packMaxTemp = newHighestTemp;
	FaultStatusMsg.packMinTemp = newLowestTemp;
	FaultStatusMsg.packAvgTemp = sumAvgTemp / HALF_SEGMENTS;

	if(newHighestVoltage > STACK_MAX_VOLTAGE ||
		newLowestVoltage < STACK_MIN_VOLTAGE){
		return 1; /* fault detected */
	}
	if(newHighestTemp > STACK_MAX_TEMP){
		return 1;
	}

	SPI_Control.lowestVoltage = newLowestVoltage;
	return 0; /* no fault detected */
}

/**
 * copies the data from the cells
 */
void copyData(){
	for (int i = 0; i < HALF_SEGMENTS; i++){
		segmentCS(i); /* select segment to read */

		for(int i = 0; i < 200; i++);

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

void CAN_Init (){

	GPIOA->MODER |=   (0b10   << GPIO_MODER_MODER11_Pos)  /* CAN TX altfunc */
	             | 	  (0b10   << GPIO_MODER_MODER12_Pos); /* CAN RX altfunc */

	GPIOA->AFR[1] = (4 << GPIO_AFRH_AFSEL11_Pos) | (4 << GPIO_AFRH_AFSEL12_Pos); /* can AFR */

    RCC->APB1ENR |= RCC_APB1ENR_CANEN;
    CAN->MCR |= CAN_MCR_INRQ; /* goes from normal mode into initialization mode */

    while (!(CAN->MSR & CAN_MSR_INAK));

    /* wakes it up */
    CAN->MCR &= ~CAN_MCR_SLEEP;
    while (CAN->MSR & CAN_MSR_SLAK);

    /* set bittiming - just read wikipedia if you don't know what that is */
    /* TODO: why is it still 1/2 of what it should be */
    CAN->BTR |= 23 << CAN_BTR_BRP_Pos | 1 << CAN_BTR_TS1_Pos | 0 << CAN_BTR_TS2_Pos;
    CAN->MCR &= ~CAN_MCR_INRQ; /* clears the initialization request and starts the actual can */

    while (CAN->MSR & CAN_MSR_INAK);

    /* blank filter - tells the can to read every message */
    CAN->FMR |= CAN_FMR_FINIT;
    CAN->FA1R |= CAN_FA1R_FACT0;
    CAN->sFilterRegister[0].FR1 = 0; /* Its like a filter, but doesn't filter anything! */
    CAN->sFilterRegister[0].FR2 = 0;
    CAN->FMR &=~ CAN_FMR_FINIT;
    CAN->IER |= CAN_IER_FMPIE0;
}
/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */


/*
 * Not used
 */
/*
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

} */

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
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
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
