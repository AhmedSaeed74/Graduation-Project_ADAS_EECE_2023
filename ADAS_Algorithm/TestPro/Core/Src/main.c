/* USER CODE BEGIN Header */
/*******************************************************************************
 *                                                                             *
 * [FILE NAME]:   main.c                                                       *
 *                                                                             *
 * [AUTHOR]:      Ahmed Saeed, Amira Shereif, Doaa Mohamed,                    *
 *                Hassan Sayed, Mennatullah Gamal, and Omneiia Atef            *
 *                                                                             *
 * [Version]:     1.0.0                                                        *
 *                                                                             *
 * [DATE]:        16/07/2023                                                   *
 *                                                                             *
 * [DESCRIPTION]: Source file for the Application                              *
 *                                                                             *
 *******************************************************************************/
/* USER CODE END Header */
/*******************************************************************************
 *                                 Includes                                    *
 *******************************************************************************/
#include "main.h"
#include "cmsis_os.h"

/*******************************************************************************
 *                              Private Includes                               *
 *******************************************************************************/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "local_definition.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "stm32f4xx_hal_tim.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/*******************************************************************************
 *                                Definitions                                  *
 *******************************************************************************/
/* USER CODE BEGIN PD */
#define MAX_FORCE      150
#define CAR_LENGTH     5.21
#define LANE_WIDTH     3.5
#define FULL_BRAKE_T   1
#define HALF_BRAKE_T   2
#define WARNING_T      3
#define LEFT_LANE      1
#define MIDDLE_LANE    2
#define RIGHT_LANE     3
#define THRESHOLD_DISTANCE 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/*******************************************************************************
 *                                Private Variables                            *
 *******************************************************************************/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
uint8_t right = 0;
uint8_t left = 0;
uint8_t brake_flag;
uint8_t Park_flag = 0;
uint8_t steering_on = 0;
uint8_t Lane = 2;
uint8_t Lane_nextState = 2;
uint8_t Range_threshold;
double SteerringAngle;
int32_t angle = 0;
double heading;
uint8_t heading_flag = 0;
uint8_t laneKeeping_flag = 0;

CAN_FilterTypeDef sFilterConfig;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint8_t RcvData;
uint32_t TxMailbox;

TaskHandle_t Task1Handle;
TaskHandle_t Task2Handle;
TaskHandle_t Task3Handle;
TaskHandle_t Task4Handle;
TaskHandle_t Task5Handle;


QueueHandle_t xUARTRecieveDataQueue;
QueueHandle_t xMakeAlgorithmQueue;
QueueHandle_t xMakeAlgorithmQueue2;
QueueHandle_t xUpdateABSQueue;

SemaphoreHandle_t myMutexUARTHandle=NULL;

/* USER CODE END PV */

/*******************************************************************************
 *                              Functions Prototypes                           *
 *******************************************************************************/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM1_Init(void);
void StartDefaultTask(void const * argument);
void LaneMarkerSteeringAngle(float,float,float,float,float,float,float,float,float,float,uint32_t);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*******************************************************************************
 *                                 Main Function                               *
 *******************************************************************************/
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

    
	/* ---------------------------MCU Configuration---------------------------*/

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
	MX_USART1_UART_Init();
	MX_CAN1_Init();
	MX_TIM1_Init();
	
	/* USER CODE BEGIN 2 */
	
	/* Configure CAN filter for receiving messages */
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = (0x123 << 5);
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0xFFE0;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
	{
		/* Filter configuration Error */
		Error_Handler();
	}
	if (HAL_CAN_Start(&hcan1) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}

	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
	{
		/* Notification Error */
		Error_Handler();
	}

    /* Configure CAN message header */
	TxHeader.StdId = 0x321;
	TxHeader.ExtId = 0x01;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = DISABLE;
	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* Create a mutex */
	myMutexUARTHandle = xSemaphoreCreateMutex();
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* Create queues */
	xUARTRecieveDataQueue = xQueueCreate(40, sizeof(InputData_t));
	xMakeAlgorithmQueue   = xQueueCreate(20, sizeof(flags_t));
	xMakeAlgorithmQueue2  = xQueueCreate(40, sizeof(InputData_t));
	xUpdateABSQueue       = xQueueCreate(10, sizeof(ControlData_t));
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* Create tasks */
	xTaskCreate(vTaskReceiveUART  , "Receive UART", configMINIMAL_STACK_SIZE*4  , NULL,4, &Task1Handle);
	xTaskCreate(vTaskMakeAlgorithm, "Make Algorithm", configMINIMAL_STACK_SIZE*4, NULL,3, &Task2Handle);
	xTaskCreate(vTaskUpdateABS    , "Update ABS", configMINIMAL_STACK_SIZE*4    , NULL,2, &Task3Handle);
	xTaskCreate(vTaskSendUART     , "Send UART", configMINIMAL_STACK_SIZE*4     , NULL,1, &Task4Handle);
	xTaskCreate(vTaskReceiveCAN   , "Receive CAN", configMINIMAL_STACK_SIZE*4   , NULL,5, &Task5Handle);
	vTaskSuspend(Task5Handle);
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void)
{

	/* USER CODE BEGIN CAN1_Init 0 */

	/* USER CODE END CAN1_Init 0 */

	/* USER CODE BEGIN CAN1_Init 1 */

	/* USER CODE END CAN1_Init 1 */
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 18;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = DISABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CAN1_Init 2 */

	/* USER CODE END CAN1_Init 2 */

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

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

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
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pin : PG14 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void LaneMarkerSteeringAngle(float LMX1, float LMY1, float LMX2, float LMY2, float WX1, float WY1, float WX2, float WY2, float LM_Left, float LM_Right, uint32_t roadID)
{
    /* Calculate the vector from the left lane marker points */
    double LMV[3] = {(LMX2 - LMX1), (LMY2 - LMY1), 0};

    /* Calculate the vector from the waypoints (road) points */
    double WV[3] = {(WX2 - WX1), (WY2 - WY1), 0};

    /* Calculate the cross product between the lane marker and road vectors */
    double crossProduct[3] = {(LMV[1] * WV[2] - LMV[2] * WV[1]), (-(LMV[0] * WV[2] - LMV[2] * WV[0])), (LMV[0] * WV[1] - LMV[1] * WV[0])};

    /* Calculate the norm (magnitude) of the cross product */
    double norm = sqrt((crossProduct[0] * crossProduct[0]) + (crossProduct[1] * crossProduct[1]) + (crossProduct[2] * crossProduct[2]));

    /* Calculate the dot product between the lane marker and road vectors */
    double dotProduct = (LMV[0] * WV[0]) + (LMV[1] * WV[1]) + (LMV[2] * WV[2]);

    /* Calculate the angular error using the arctan function */
    double angularError = atan(norm / dotProduct);

    /* Check the sign of the z-component of the cross product to determine the direction of the angular error */
    if (crossProduct[2] < 0)
    {
        angularError = angularError * (-1);
    }

    /* Scale the angular error by a factor (1250 in this case) */
    angularError = angularError * 1250;

    /* Calculate lane monitoring based on left and right lane markers */
    double laneMonitoring = LM_Left + LM_Right;
    laneMonitoring = LM_Right / laneMonitoring;
    laneMonitoring = (laneMonitoring - 0.5) * (-250);

    /* Check if roadID is not zero and update the angle accordingly */
    if (roadID != 0)
    {
        angle = angularError + laneMonitoring;
    }
    else
    {
		/* Set angle to zero if roadID is zero */
        angle = 0; 
    }
}



void vTaskReceiveUART(void * argument)
{
	/* Define a buffer to store received UART data */
    char BufferUARTReceive[255]; 
	
	/* Create a structure to store received data */
    InputData_t ReceivedData;    

    while(1)
    {
        if( xSemaphoreTake( myMutexUARTHandle, ( TickType_t ) portMAX_DELAY) == pdTRUE )
        {
            /* Receive data from the UART with a timeout of 200ms */
            HAL_UART_Receive(&huart1, (uint8_t*)BufferUARTReceive, 207, 200);

            /* Parse the received data using sscanf */
            sscanf(BufferUARTReceive, "*%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%d#",
                &ReceivedData.desiredSpeed, &ReceivedData.Heading,
                &ReceivedData.rangeOfLRR, &ReceivedData.relativeSpeedOfLRR,
                &ReceivedData.rangeOfSRR, &ReceivedData.relativeSpeedOfSRR,
                &ReceivedData.rangeOfFLR, &ReceivedData.relativeSpeedOfFLR, &ReceivedData.azimuthAngleOfFLR,
                &ReceivedData.rangeOfFRR, &ReceivedData.relativeSpeedOfFRR, &ReceivedData.azimuthAngleOfFRR,
                &ReceivedData.rangeOfBRR, &ReceivedData.relativeSpeedOfBRR, &ReceivedData.azimuthAngleOfBRR,
                &ReceivedData.rangeOfBLR, &ReceivedData.relativeSpeedOfBLR, &ReceivedData.azimuthAngleOfBLR,
                &ReceivedData.currentVelocity,
                &ReceivedData.laneMarkerCoordX_D0, &ReceivedData.laneMarkerCoordY_D0,
                &ReceivedData.laneMarkerCoordX_D1, &ReceivedData.laneMarkerCoordY_D1,
                &ReceivedData.rightWorldCoordX_D0, &ReceivedData.rightWorldCoordY_D0,
                &ReceivedData.rightWorldCoordX_D1, &ReceivedData.rightWorldCoordY_D1,
                &ReceivedData.laneMarkerLeftIntersection_D1, &ReceivedData.laneMarkerRightIntersection_D1,
                &ReceivedData.roadIDunderLaneMarker);

            /* Send the received data to queues for further processing */
            xQueueSend(xUARTRecieveDataQueue, &ReceivedData, portMAX_DELAY);
            xQueueSend(xMakeAlgorithmQueue2, &ReceivedData, portMAX_DELAY);

            /* Release the UART mutex */
            xSemaphoreGive( myMutexUARTHandle ); 
        }

        /* Delay the task for 60ms */
        vTaskDelay(pdMS_TO_TICKS(60));
    }
}



void vTaskMakeAlgorithm(void *argument)
{
	/* Create a structure to store received data */
	InputData_t RecievedData;
	
	/* Create a structure to store flag data */
	flags_t   Flages_Sets;
	
	/* Create a structure to store TTC calculation data */
	TTCCalculation_t TCC_Data;

	while(1)
	{ 
        /* Initialize all flags to 0 */
		Flages_Sets.Warning_Flag=0;
		Flages_Sets.Decreasing_Speed_Flag=0;
		Flages_Sets.Full_Braking_Flag=0;
		Flages_Sets.Turn_Right=0;
		Flages_Sets.Turn_Left=0;
		Flages_Sets.RangeThresholdFlag=0;

        /* Receive data from the UART receive queue with a timeout of portMAX_DELAY */
		xQueueReceive(xUARTRecieveDataQueue, &RecievedData, portMAX_DELAY);

        /* Calculate Time-to-Collision (TTC) for various sensors */
		if((RecievedData.rangeOfLRR>0))
		{
			TCC_Data.TTC_LRR = RecievedData.rangeOfLRR / (abs(RecievedData.relativeSpeedOfLRR));
		}

		else
		{
			TCC_Data.TTC_LRR = 2000;
		}

		if((RecievedData.rangeOfSRR>0))
		{
			TCC_Data.TTC_SRR = RecievedData.rangeOfSRR / (abs(RecievedData.relativeSpeedOfSRR));
		}

		else
		{
			TCC_Data.TTC_SRR = 2000;
		}

		if((RecievedData.rangeOfFLR>0))
		{
			TCC_Data.TTC_FLR = (RecievedData.rangeOfFLR * (cos(RecievedData.azimuthAngleOfFLR * (M_PI / 180.00)))) / (abs(RecievedData.relativeSpeedOfFLR));
		}

		else
		{
			TCC_Data.TTC_FLR = 1000;
		}

		if((RecievedData.rangeOfFRR>0))
		{
			TCC_Data.TTC_FRR = (RecievedData.rangeOfFRR * (cos(RecievedData.azimuthAngleOfFRR * (M_PI / 180.00)))) / (abs(RecievedData.relativeSpeedOfFRR));
		}

		else
		{
			TCC_Data.TTC_FRR = 1000;
		}

		if((RecievedData.rangeOfBRR>0))
		{
			TCC_Data.TTC_BRR = (RecievedData.rangeOfBRR * (cos(RecievedData.azimuthAngleOfBRR * (M_PI / 180.00)))) / (abs(RecievedData.relativeSpeedOfBRR));
		}

		else
		{
			TCC_Data.TTC_BRR = 1000;
		}

		if((RecievedData.rangeOfBLR>0))
		{
			TCC_Data.TTC_BLR = (RecievedData.rangeOfBLR * (cos(RecievedData.azimuthAngleOfBLR * (M_PI / 180.00)))) / (abs(RecievedData.relativeSpeedOfBLR));
		}

		else
		{
			TCC_Data.TTC_BLR = 1000;
		}

		/* Setting Flags based on TTC values */
		if(TCC_Data.TTC_LRR<WARNING_T)
		{
			Flages_Sets.Warning_Flag=1;
		}

		if(TCC_Data.TTC_LRR<HALF_BRAKE_T)
		{
			Flages_Sets.Decreasing_Speed_Flag=1;
		}

		if(TCC_Data.TTC_SRR<FULL_BRAKE_T)
		{
			Flages_Sets.Full_Braking_Flag=1;
		}

		if((TCC_Data.TTC_FRR>TCC_Data.TTC_LRR))
		{
			Flages_Sets.Turn_Right=1;
		}

		if((TCC_Data.TTC_FLR>TCC_Data.TTC_LRR))
		{
			Flages_Sets.Turn_Left=1;
		}

		if((RecievedData.rangeOfLRR<THRESHOLD_DISTANCE) &&  (RecievedData.rangeOfLRR!=0))
		{
			Flages_Sets.RangeThresholdFlag=1;
		}

        /* Send the flag data to the Make Algorithm queue for further processing */
		xQueueSend(xMakeAlgorithmQueue, &Flages_Sets, portMAX_DELAY);

        /* Delay the task for 60ms */
		vTaskDelay(pdMS_TO_TICKS(60));
	}
}

void vTaskUpdateABS(void * argument)
{
	/* Create a structure to store received data */
	InputData_t RecievedData;
	
	/* Create a structure to store control data */
	ControlData_t Resulted_Data;
	
	/* Create a structure to store flag data */
	flags_t   Flages_Sets;

	while(1)
	{
		/* Receive flags data from the Make Algorithm queue with a timeout of portMAX_DELAY */
		xQueueReceive(xMakeAlgorithmQueue, &Flages_Sets, portMAX_DELAY);
		
		/* Receive data from the Make Algorithm queue with a timeout of portMAX_DELAY */
		xQueueReceive(xMakeAlgorithmQueue2, &RecievedData, portMAX_DELAY);

        /* Initialize laneKeeping flag to 0 */
		Resulted_Data.laneKeeping = 0;

        /* Calculate lane marker steering angle */
		LaneMarkerSteeringAngle(RecievedData.laneMarkerCoordX_D0,RecievedData.laneMarkerCoordY_D0,RecievedData.laneMarkerCoordX_D1,
				RecievedData.laneMarkerCoordY_D1,RecievedData.rightWorldCoordX_D0,RecievedData.rightWorldCoordY_D0,
				RecievedData.rightWorldCoordX_D1,RecievedData.rightWorldCoordY_D1,RecievedData.laneMarkerLeftIntersection_D1,
				RecievedData.laneMarkerRightIntersection_D1,RecievedData.roadIDunderLaneMarker);

        /* Check laneKeeping_flag and currentVelocity to set the brakex value */
		if (laneKeeping_flag == 1)
		{
			if (RecievedData.currentVelocity > 13)
			{
				Resulted_Data.brake = 90;
			}

			else
			{
				Resulted_Data.brake = 0;
			}
		}

        /* Check RxData and range sensors for lane keeping and braking */
		if(RxData[0] != 2 && RecievedData.rangeOfLRR == 0 && RecievedData.rangeOfFRR == 0 && RecievedData.rangeOfFLR == 0)
		{
			Resulted_Data.control = 0;
			SteerringAngle = 0;
			Resulted_Data.laneKeeping = 1;

			if (laneKeeping_flag == 0)
			{
				laneKeeping_flag = 1;

				if (RecievedData.currentVelocity > 13)
				{
					Resulted_Data.brake = 90;
				}

				else
				{
					Resulted_Data.brake = 0;
				}
			}
		}

        /* Check lane, RxData, and range sensors for steering angle and control */
		if(Lane == MIDDLE_LANE && RxData[0] == 2 && (RecievedData.rangeOfFRR > 80 || RecievedData.rangeOfFRR == 0 || RecievedData.rangeOfFLR > 80 || RecievedData.rangeOfFLR == 0))
		{
			/* Calculate steering angle based on range sensors
               Set Resulted_Data.control and steering_on accordingly
               Handle heading and Lane_nextState based on conditions */
			if(RecievedData.rangeOfFRR > 80 || RecievedData.rangeOfFRR == 0)
			{
				SteerringAngle = -((atan(CAR_LENGTH/LANE_WIDTH) *180) / M_PI);
				Resulted_Data.control = 0;
				steering_on = 1;

				if (heading_flag == 0)
				{
					heading = RecievedData.Heading;
					heading_flag = 1;
				}

				if (SteerringAngle == -((atan(CAR_LENGTH/LANE_WIDTH) *180) / M_PI) && RecievedData.Heading < (heading + 10) && Park_flag == 0)
				{
					SteerringAngle = -((atan(CAR_LENGTH/LANE_WIDTH) *180) / M_PI);
					Resulted_Data.control = 0;
				}

				else if(SteerringAngle == -((atan(CAR_LENGTH/LANE_WIDTH) *180) / M_PI) && angle < 1 && angle > -1)
				{
					Lane_nextState++;
					SteerringAngle = 0;
					Resulted_Data.laneKeeping = 1;
					heading_flag = 0;
					Park_flag = 0;
				}

				else
				{
					SteerringAngle = 0;
					Resulted_Data.laneKeeping = 1;
					Park_flag = 1;
				}
			}

			else if(RecievedData.rangeOfFLR > 80 || RecievedData.rangeOfFLR == 0)
			{
				SteerringAngle = ((atan(CAR_LENGTH/LANE_WIDTH) *180) / M_PI);
				Resulted_Data.control = 0;
				steering_on = 1;

				if (heading_flag == 0)
				{
					heading = RecievedData.Heading;
					heading_flag = 1;
				}

				if (SteerringAngle == ((atan(CAR_LENGTH/LANE_WIDTH) *180) / M_PI) && RecievedData.Heading > (heading - 10)  && Park_flag == 0)
				{
					SteerringAngle = ((atan(CAR_LENGTH/LANE_WIDTH) *180) / M_PI);
					Resulted_Data.control = 0;
				}

				else if(SteerringAngle == ((atan(CAR_LENGTH/LANE_WIDTH) *180) / M_PI) && angle < 1 && angle > -1)
				{
					Lane_nextState--;
					SteerringAngle = 0;
					Resulted_Data.laneKeeping = 1;
					heading_flag = 0;
					Park_flag = 0;
				}

				else
				{
					SteerringAngle = 0;
					Resulted_Data.laneKeeping = 1;
					Park_flag = 1;
				}
			}
		}

        /* Check flags for braking and turning scenarios */
		else if (((Flages_Sets.Full_Braking_Flag) &&  (Flages_Sets.Decreasing_Speed_Flag)) || ((RecievedData.rangeOfLRR<5) && (RecievedData.rangeOfLRR>0)))
		{
			if((Flages_Sets.Turn_Right) && (RecievedData.relativeSpeedOfBRR>=0) && (brake_flag != 1) && ((Lane == LEFT_LANE) || (Lane == MIDDLE_LANE)) )
			{
				SteerringAngle = -((atan(CAR_LENGTH/LANE_WIDTH) *180) / M_PI);
				Resulted_Data.control = 1;
				steering_on = 1;

				if (heading_flag == 0)
				{
					heading = RecievedData.Heading;
					heading_flag = 1;
				}
			}

			else if((Flages_Sets.Turn_Left) && (RecievedData.relativeSpeedOfBLR>=0) && (brake_flag != 1) && ((Lane == RIGHT_LANE) || (Lane == MIDDLE_LANE)) )
			{
				SteerringAngle = ((atan(CAR_LENGTH/LANE_WIDTH) *180) / M_PI);
				Resulted_Data.control = 1;
				steering_on = 1;

				if (heading_flag == 0)
				{
					heading = RecievedData.Heading;
					heading_flag = 1;
				}
			}

			else
			{
				Resulted_Data.brake =  MAX_FORCE;  // Apply maximum brake force
				Resulted_Data.control=3;               // speed = 0;
				SteerringAngle=0;
				brake_flag = 1;
			}
		}

		else if  ((Flages_Sets.Decreasing_Speed_Flag) && (!(Flages_Sets.Full_Braking_Flag)))
		{
			if((Flages_Sets.Turn_Right) && (RecievedData.relativeSpeedOfBRR>=0) && (brake_flag != 1) && ((Lane == LEFT_LANE) || (Lane == MIDDLE_LANE)) )
			{
				SteerringAngle = -((atan(CAR_LENGTH/LANE_WIDTH) *180) / M_PI);
				Resulted_Data.control = 1;
				steering_on = 1;

				if (heading_flag == 0)
				{
					heading = RecievedData.Heading;
					heading_flag = 1;
				}
			}

			else if( (Flages_Sets.Turn_Left) && (RecievedData.relativeSpeedOfBLR>=0) && (brake_flag != 1) && ((Lane == RIGHT_LANE) || (Lane == MIDDLE_LANE)) )
			{
				SteerringAngle = ((atan(CAR_LENGTH/LANE_WIDTH) *180) / M_PI);
				Resulted_Data.control = 1;
				steering_on = 1;

				if (heading_flag == 0)
				{
					heading = RecievedData.Heading;
					heading_flag = 1;
				}
			}

			else
			{
				Resulted_Data.brake=  0.6 * MAX_FORCE;    // Apply 40% brake force
				Resulted_Data.control=2;                      // speed = Original speed;
				SteerringAngle=0;
				brake_flag = 1;
			}
		}

		else if ( Flages_Sets.Warning_Flag)
		{
			Resulted_Data.brake= 0;     // No brake force applied
			Resulted_Data.laneKeeping = 1;

			if (laneKeeping_flag == 1)
			{
				if (RecievedData.currentVelocity > 13)
				{
					Resulted_Data.brake = 90;
				}

				else
				{
					Resulted_Data.brake = 0;
				}
			}

			if(brake_flag == 1)
			{
				Resulted_Data.control=4;          // speed = Original speed;
			}

			else
			{
				Resulted_Data.control=1;          // speed = Original speed;
			}
		}

		else
		{
			if (Resulted_Data.laneKeeping == 0)
			{
				Resulted_Data.brake= 0;  // No brake force applied
			}

			if (laneKeeping_flag == 1)
			{
				if (RecievedData.currentVelocity > 13)
				{
					Resulted_Data.brake = 90;
				}

				else
				{
					Resulted_Data.brake = 0;
				}
			}

			if(brake_flag == 1)
			{
				Resulted_Data.control=4;          // speed = Original speed;
			}

			else
			{
				Resulted_Data.control=0;          // speed = Original speed;
			}
		}

        /* Check RxData and steering_on for steering angle adjustments */
		if  (RxData[0] != 2 && steering_on == 1)
		{
			if ((Flages_Sets.Decreasing_Speed_Flag == 0))
			{
				if (RecievedData.Heading > heading)
				{
					if (SteerringAngle == -((atan(CAR_LENGTH/LANE_WIDTH) *180) / M_PI) && RecievedData.Heading < (heading + 10))
					{
						SteerringAngle = -((atan(CAR_LENGTH/LANE_WIDTH) *180) / M_PI);
						Resulted_Data.control = 1;
					}

					else if (SteerringAngle == ((atan(CAR_LENGTH/LANE_WIDTH) *180) / M_PI) && RecievedData.Heading < (heading + 10))
					{
						SteerringAngle = ((atan(CAR_LENGTH/LANE_WIDTH) *180) / M_PI);
						Resulted_Data.control = 1;
					}

					else if(SteerringAngle == -((atan(CAR_LENGTH/LANE_WIDTH) *180) / M_PI))
					{
						Lane_nextState++;
						Resulted_Data.control = 0;
						SteerringAngle = 0;
						Resulted_Data.laneKeeping = 1;
						heading_flag = 0;
					}

					else if(SteerringAngle == ((atan(CAR_LENGTH/LANE_WIDTH) *180) / M_PI))
					{
						Lane_nextState--;
						Resulted_Data.control = 0;
						SteerringAngle = 0;
						Resulted_Data.laneKeeping = 1;
						heading_flag = 0;
					}

					else
					{
						SteerringAngle = 0;
						Resulted_Data.control = 0;
						Resulted_Data.laneKeeping = 1;
					}
				}

				else if (RecievedData.Heading < heading)
				{
					if (SteerringAngle == -((atan(CAR_LENGTH/LANE_WIDTH) *180) / M_PI) && RecievedData.Heading > (heading - 10))
					{
						SteerringAngle = -((atan(CAR_LENGTH/LANE_WIDTH) *180) / M_PI);
						Resulted_Data.control = 1;
					}
					else if (SteerringAngle == ((atan(CAR_LENGTH/LANE_WIDTH) *180) / M_PI) && RecievedData.Heading > (heading - 10))
					{
						SteerringAngle = ((atan(CAR_LENGTH/LANE_WIDTH) *180) / M_PI);
						Resulted_Data.control = 1;
					}

					else if(SteerringAngle == -((atan(CAR_LENGTH/LANE_WIDTH) *180) / M_PI))
					{
						Lane_nextState++;
						Resulted_Data.control = 0;
						SteerringAngle = 0;
						Resulted_Data.laneKeeping = 1;
						heading_flag = 0;
					}

					else if(SteerringAngle == ((atan(CAR_LENGTH/LANE_WIDTH) *180) / M_PI))
					{
						Lane_nextState--;
						Resulted_Data.control = 0;
						SteerringAngle = 0;
						Resulted_Data.laneKeeping = 1;
						heading_flag = 0;
					}

					else
					{
						SteerringAngle = 0;
						Resulted_Data.control = 0;
						Resulted_Data.laneKeeping = 1;
					}
				}
			}
		}

        /* Handle brake_flag and control based on conditions */
		if((brake_flag == 1) && (!Flages_Sets.Decreasing_Speed_Flag) && (!Flages_Sets.Full_Braking_Flag)
				&& ((Lane == 1 && (RecievedData.rangeOfLRR == 0 || RecievedData.rangeOfFRR == 0))
						|| (Lane == 2 && (RecievedData.rangeOfLRR == 0 || RecievedData.rangeOfFRR == 0 || RecievedData.rangeOfFLR == 0))
						|| (Lane == 3 && (RecievedData.rangeOfLRR == 0 || RecievedData.rangeOfFLR == 0))))
		{
			brake_flag = 0;
			Resulted_Data.control = 0;
		}

        /* Set Resulted_Data with calculated values */
		Resulted_Data.steeringAngle = SteerringAngle;

		if (SteerringAngle == 0)
		{
			Lane = Lane_nextState;
		}

		Resulted_Data.laneID = Lane;
		Resulted_Data.laneMarkerAngle = angle;

        /* Set Resulted_Data.driver_drowsiness and Resulted_Data.driver_distraction based on RxData */
		if (RxData[0] == 0)
		{
			Resulted_Data.driver_drowsiness = 0;
		}

		else if (RxData[0] == 1)
		{
			Resulted_Data.driver_drowsiness = 1;
		}

		else if (RxData[0] == 2)
		{
			Resulted_Data.driver_drowsiness = 2;
		}

		if (RxData[1] == 0)
		{
			Resulted_Data.driver_distraction = 0;
		}

		else if (RxData[1] == 1)
		{
			Resulted_Data.driver_distraction = 1;
		}

        /* Send the Resulted_Data to the Update ABS queue for further processing */
		xQueueSend(xUpdateABSQueue, &Resulted_Data, portMAX_DELAY);

        /* Delay the task for 60ms */
		vTaskDelay(pdMS_TO_TICKS(60));
	}

}

void vTaskSendUART(void * argument)
{
	ControlData_t Resulted_Data;
	char BufferUARTSend[37];

	while(1)
	{
		xQueueReceive(xUpdateABSQueue, &Resulted_Data, portMAX_DELAY);

		sprintf(BufferUARTSend,"*%d;%03d_%03d&%d$%d@%05d^%d!%d#",Resulted_Data.control,Resulted_Data.brake,Resulted_Data.steeringAngle,Resulted_Data.laneID,Resulted_Data.laneKeeping,Resulted_Data.laneMarkerAngle,Resulted_Data.driver_drowsiness,Resulted_Data.driver_distraction);

		HAL_UART_Transmit(&huart1,(uint8_t *) BufferUARTSend, strlen(BufferUARTSend), 200);

		vTaskDelay(pdMS_TO_TICKS(60));
	}
}

void vTaskReceiveCAN(void * argument)
{
	ControlData_t Resulted_Data;
	char BufferUARTSend[37];

	while(1)
	{
		if(RxData[0] == 2)
		{
			//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_RESET);

			if((Lane == MIDDLE_LANE))
			{
				vTaskDelay(pdMS_TO_TICKS(60));
			}

			else
			{
				Resulted_Data.brake = MAX_FORCE;
				Resulted_Data.control = 3;
				SteerringAngle = 0;
				Resulted_Data.steeringAngle = SteerringAngle;
				Resulted_Data.laneID = Lane;
				Resulted_Data.laneKeeping = 0;
				Resulted_Data.laneMarkerAngle = 0;
				Resulted_Data.driver_drowsiness = 2;
				Resulted_Data.driver_distraction = 2;
				sprintf(BufferUARTSend,"*%d;%03d_%03d&%d$%d@%05d^%d!%d#",Resulted_Data.control,Resulted_Data.brake,Resulted_Data.steeringAngle,Resulted_Data.laneID,Resulted_Data.laneKeeping,Resulted_Data.laneMarkerAngle,Resulted_Data.driver_drowsiness,Resulted_Data.driver_distraction);
				HAL_UART_Transmit(&huart1,(uint8_t *) BufferUARTSend, strlen(BufferUARTSend), 200);
			}
		}

		else if (RxData[0] == 1 || RxData[1] == 1)
		{
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
			vTaskSuspend(NULL);
		}

		else if (RxData[0] == 0 || RxData[1] == 0)
		{
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
			vTaskSuspend(NULL);
		}




	}
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
	xTaskResumeFromISR(Task5Handle);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for(;;)
	{
		//Send_DefTask();
		vTaskDelay(1000);
	}
	/* USER CODE END 5 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM2 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM2) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

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
