/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "control.h"
#include "pca9685.h"
#include "stdio.h"
#include "servo.h"
#include "NRF24L01.h"
#include "mpu9255.h"
//#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
MPU9255_t MPU9255;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

SPI_HandleTypeDef hspi1;

/* Definitions for Communication */
osThreadId_t CommunicationHandle;
const osThreadAttr_t Communication_attributes = {
  .name = "Communication",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Control */
osThreadId_t ControlHandle;
const osThreadAttr_t Control_attributes = {
  .name = "Control",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* USER CODE BEGIN PV */

//NRF Variables
//uint8_t NRF_RxAddress[5] = { 0xEE, 0xDD, 0xCC, 0xBB, 0xAA };
//uint8_t NRF_TxAddress[5] = { 0xEE, 0xDD, 0xCC, 0xBB, 0xAA };

uint8_t NRF_Master_Address[5] = { 0x00, 0xDD, 0xCC, 0xBB, 0xAA }; //gui
uint8_t NRF_Slave_Address[5] = { 0xEE, 0xDD, 0xCC, 0xBB, 0xAA }; //spider

uint8_t NRF_TxData[32] = "TK xinh dep\n";
uint8_t NRF_RxData[32];

uint8_t NRF_Master_Data[32] = "Master transmit\n";
uint8_t NRF_Slave_Data[32] = "Slave transmit\n";

float Servo_val[12] = { 120, 60, 50, 90, 90, 90, 90, 90, 90, 90, 90, 90 };
extern float startAngle[12];
float endAngle[12];
extern float currentAngle[12]; //4/4
uint8_t flag_pca_tx = 0, flag_pca_rx = 0;
uint32_t Time_start, Time_end, Time_measure;

//char TxData[50];
float pitch1;
float yaw1;
float roll1;
extern float deltat, Now;
//uint8_t tim2_it_flag = 0;
//float b[6];
//float c[6];
//float gyroBias[3] = { 0, 0, 0 };
//float accelBias[3] = { 0, 0, 0 };
//float SelfTest[6];
uint8_t MPU_Flag = 0;
volatile uint8_t MPU_Counter = 0;
volatile uint8_t MPU_Rx_Flag = 0;
//float currentAngle[12] = { 48, 180, 0, 95, 180, 0, 95, 180, 0, 48, 180, 0 };
uint16_t time = 500;
uint16_t TK = 0; //move
float angle;
//Leg length
//a1 = 56.84
//a2 = 114
//a3 = 170.74
//a1 + a2 = 284.74
//a1 + a2 + a3 = 341.58
float x = 284.74;
float y = 0;
float z = 0;

uint8_t timer2_counter = 0;
uint8_t it_timer_flag = 0;
uint32_t time_do = 0;
uint32_t Time_end = 0;

//PID
extern float Roll_E0, Roll_E1, Roll_E2;
extern float Pitch_E0, Pitch_E1, Pitch_E2;
extern float Yaw_E0, Yaw_E1, Yaw_E2;
extern float Roll_Pre, Pitch_Pre, Yaw_Pre;
extern float Roll_Ref, Pitch_Ref, Yaw_Ref;
extern float Roll_u, Roll_u1, Pitch_u, Pitch_u1, Yaw_u, Yaw_u1;
extern float Kp, Ki, Kd;
float T_PID, T_PID_pre;
float T_PID_Yaw, T_PID_Yaw_pre;
//Control
uint8_t Mode = 10;
extern Leg_t LegLF;
extern Leg_t LegLR;
extern Leg_t LegRF;
extern Leg_t LegRR;
extern Servo_t *array_Servo[12];
extern Leg_t *array_Leg[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
void StartCommunication(void *argument);
void StartControl(void *argument);

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
//  MPU
  //uint8_t readData;
  	//HAL_I2C_Mem_Read_DMA(&hi2c1, MPU9250_ADDRESS, WHO_AM_I_MPU9250, 1, &readData, 1);
	//while (MPU9255_Init(&hi2c1) == 1); //Turn on LD12
	//osDelay(1000);
//	HAL_Delay(1000);

//  NRF24
	//NRF24_Init();
	//NRF24_TxMode(NRF_TxAddress, 10);
	//Gui
	//NRF24_TxMode(NRF_Master_Address, 10);
	//NRF24_RxMode(NRF_Slave_Address, 10);
	//spider
	//NRF24_RxMode(NRF_Master_Address, 10);
	//NRF24_TxMode(NRF_Slave_Address, 10);

//	PCA
	//PCA9685_Init(&hi2c2);
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
	//HAL_Delay(2000);

//	HAL_Delay(2000);
//	Spider_Sit();
//	HAL_Delay(2000);
//	Spider_Init();
	//Spider_stand();
	//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	//After setup complete, start timer 2
	//HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Communication */
  CommunicationHandle = osThreadNew(StartCommunication, NULL, &Communication_attributes);

  /* creation of Control */
  ControlHandle = osThreadNew(StartControl, NULL, &Control_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		if (HAL_I2C_IsDeviceReady(&hi2c2, 0x80, 1, 100) == HAL_OK) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, SET);

		} else {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, RESET);
			HAL_I2C_DeInit(&hi2c2);
			HAL_Delay(500);
			HAL_I2C_Init(&hi2c2);
			PCA9685_Init(&hi2c2);
			//Spider_Init();
			//HAL_Delay(1000);
		}
//		if(NRF24_Transmit(NRF_Master_Data) == 1){
//			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
//		}
//		else
//			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
//		HAL_Delay(1000);
		//NRF TX mode gui
//		if((HAL_GetTick() - Time_start) > 1000){
//			NRF24_Init();
//			NRF24_TxMode(NRF_Master_Address, 10);
//			//HAL_Delay(10);
//			Time_start = HAL_GetTick();
//			if(NRF24_Transmit(NRF_Master_Data) == 1){
//				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
//			}
//			else
//				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
//			NRF24_RxMode(NRF_Slave_Address, 10);
//		}

		//NRF RX mode spider

		//if (isDataAvailable(1) == 1) {
//			NRF24_Receive(NRF_RxData);
//			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
//			float distance = atof(NRF_RxData[3]);
//			//forward
//			int number_of_steps = (int) distance / 75;
//			if (((int) distance % 75) > 38) {
//				number_of_steps++;
//			}
//			if (NRF_RxData[1] == '3') {
//				for (int i = 0; i < number_of_steps; i++) {
//					Spider_Step_Forward(137, 75, 120);
//				}
//			} else if (NRF_RxData[1] == '4') {
//				for (int i = 0; i < number_of_steps; i++) {
//					Spider_Step_Backward_Arg(137, 75, 120);
//				}
//			}
		//}
//		else
//			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
//			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
//
//		//NRF TX mode spider
//		if((HAL_GetTick() - Time_start) > 100){
//			NRF24_Init();
//			NRF24_TxMode(NRF_Slave_Address, 10);
//			Time_start = HAL_GetTick();
//			if(NRF24_Transmit(NRF_Slave_Data) == 1){
//				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
//			}
//			else
//				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
//			NRF24_RxMode(NRF_Master_Address, 10);
//		}

		//NRF RX mode spider

//

		//test gui
//		if(it_timer_flag){
//			Time_start = HAL_GetTick();
//		if((HAL_GetTick() - Time_start) > 1000){
//			NRF24_Init();
//			NRF24_TxMode(NRF_Slave_Address, 10);
//			//readAll(&hi2c1, &MPU9255);
//			//Time_start = HAL_GetTick();
////			pitch1 = MPU9255.pitch;
////			yaw1 = MPU9255.yaw;
////			roll1= MPU9255.roll;
////			MPU_Flag = 1;
//
//			//HAL_Delay(10);
//			//snprintf(NRF_Slave_Data, sizeof(NRF_Slave_Data), "%3.2f,%3.2f,%3.2f\r\n", yaw1, pitch1, roll1);
//
//			if(NRF24_Transmit(NRF_Slave_Data) == 1){
//				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
//			}
//			else
//				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
//			NRF24_RxMode(NRF_Master_Address, 10);
////				Time_end = HAL_GetTick();
////				Time_measure = Time_end - Time_start;
//		}
		if (MPU_Flag) {
			MPU_Flag = 0;

			PID(roll1, pitch1, 20);
//			MPU_Counter++;
//			if(MPU_Counter > 100){
//				Inverse_Calc(&LegLF, 120, LegLF.y, -100 + Pitch_u - Roll_u);
//				ControlLeg(&LegLF);
//				Inverse_Calc(&LegLR, 120, LegLR.y, -100 + Pitch_u + Roll_u);
//				ControlLeg(&LegLR);
//				Inverse_Calc(&LegRR, 120, LegRR.y, -100 - Pitch_u + Roll_u);
//				ControlLeg(&LegRR);
//				Inverse_Calc(&LegRF, 120, LegRF.y, -100 - Pitch_u - Roll_u);
//				ControlLeg(&LegRF);
//				MPU_Counter = 0;
		}

		//end test gui
//
//		if (timer2_counter == 100) {
//			//timer2_counter = 0;
////			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
//			NRF24_Init();
//			NRF24_TxMode(NRF_Slave_Address, 10);
////			readAll(&hi2c1, &MPU9255);
////
////			pitch1 = MPU9255.pitch;
////			yaw1 = MPU9255.yaw;
////			roll1= MPU9255.roll;
////
//			snprintf(NRF_Slave_Data, sizeof(NRF_Slave_Data),
//					"%3.2f,%3.2f,%3.2f\r\n", yaw1, pitch1, roll1);
//			if (NRF24_Transmit(NRF_Slave_Data) == 1) {
//				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
//			}
////			else
////				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
//			NRF24_Init();
//			NRF24_RxMode(NRF_Master_Address, 10);
//			timer2_counter = 0;
//		}
//		HAL_UART_Transmit_DMA(&huart2, NRF_TxData, sizeof(NRF_TxData));
//		HAL_Delay(1000);
//		if(tim2_it_flag){
//
//				 //start_time = HAL_GetTick();
//				readAll(&hi2c1, &MPU9255);
//
//			  	pitch1 = MPU9255.pitch;
//			  	yaw1 = MPU9255.yaw;
//			  	roll1= MPU9255.roll;
//		 	  	//HAL_Delay(10);
//		        snprintf(TxData, sizeof(TxData), "%3.2f,%3.2f,%3.2f\r\n", yaw1, pitch1, roll1);
//		        //HAL_UART_Transmit_DMA(&huart2, (uint8_t*)buffer, sizeof(buffer));
//		//        	  if (NRF24_Transmit((uint8_t*)TxData) == 1)
//		//        	  {
//		//
//		//        		  if(RF24_isAckPayloadAvailable(pipe_num)){
//		//        			  NRF24_read_payload(&ackData, sizeof(ackData));
//		//        			  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
//		//        		  }
//		//        	  }
//		//        	  HAL_Delay(10);
//		//        end_time = HAL_GetTick();
//		//        execution_time = end_time - start_time;
//				}
		//else
		//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);

//		for(int servo_id = 0; servo_id < 12; servo_id++){
//			ControlServo(array_Servo[servo_id],startAngle[servo_id]);
//			HAL_Delay(50);
//		}
//		for(int servo_id = 0; servo_id < 12; servo_id++){
//			//ControlServo(array_Servo[servo_id],currentAngle[servo_id]);
//			ControlServo(array_Servo[servo_id],(array_Servo[servo_id])->Current_Angle);
//
//			HAL_Delay(50);
//		}
//		for(int servo_id = 0; servo_id < 12; servo_id++){
//			//ControlServo(array_Servo[servo_id],currentAngle[servo_id]);
//			ControlServo(array_Servo[servo_id],(array_Servo[servo_id])->Target_Angle);
//			HAL_Delay(50);
//		}

//		Spider_Init();
//		HAL_Delay(1000);
//		Spider_Sit();
		//HAL_Delay(1000);
		//Time_start = HAL_GetTick();
//		Time_start = HAL_GetTick();
//		Inverse_Calc(&LegRF, 137, 0, -120);
//		HAL_Delay(20);
//		Time_end = HAL_GetTick();
//		Time_measure = Time_end - Time_start;
//		Spider_Step_Forward();
		//Time_end = HAL_GetTick();
		//Time_measure = Time_start - Time_end;

		Spider_Step_Forward_Arg(137, 75, -120);
		Spider_Step_Backward_Arg(137, 75, -120);
//		InverseDirect(&LegLF, LegLF.target_x, LegLF.target_y, LegLF.target_z);
//		InverseDirect(&LegLR, LegLR.target_x, LegLR.target_y, LegLR.target_z);
//		InverseDirect(&LegRF, LegRF.target_x, LegRF.target_y, LegRF.target_z);
//		InverseDirect(&LegRR, LegRR.target_x, LegRR.target_y, LegRR.target_z);
//		for(int servo_id = 0; servo_id < 12; servo_id++){
//				ControlServoDirect(array_Servo[servo_id],array_Servo[servo_id]->Target_Angle);
//				HAL_Delay(50);
//		}
		//Spider_Turn_Left();
		//ControlServo(array_Servo[2],angle);
//		for(angle = 0; angle < 140; angle += 20){
//			ControlServo(array_Servo[2],angle);
//			ControlServo(array_Servo[5],angle);
//			ControlServo(array_Servo[8],angle);
//			ControlServo(array_Servo[11],angle);
//			ControlServo(array_Servo[3],angle/4.0 + 60);
//			HAL_Delay(500);
//		}
//		HAL_Delay(500);
//		for(angle = 140; angle > 0; angle -= 20){
//			ControlServo(array_Servo[2],angle);
//			ControlServo(array_Servo[5],angle);
//			ControlServo(array_Servo[8],angle);
//			ControlServo(array_Servo[11],angle);
//			ControlServo(array_Servo[3],angle/4.0 + 60);
//			HAL_Delay(500);
//		}
		//HAL_Delay(500);

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
//	if(hi2c == &hi2c2)
//		flag_pca_tx = 1;
	if(hi2c == &hi2c1){
//		MPU_Rx_Flag = 1;
//		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
	}
	// TX Done .. Do Something!
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	// RX Done .. Do Something!
//	if(hi2c == &hi2c2)
//		flag_pca_rx = 1;
	if(hi2c == &hi2c1){
//		MPU_Rx_Flag = 1;
//		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
	}
}
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c){
	if(hi2c == &hi2c1){
//		MPU_Rx_Flag = 1;
//		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
	}
}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
	if(hi2c == &hi2c1){
		MPU_Rx_Flag = 1;
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
	}
}
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	if (htim->Instance == TIM2) {
//		//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
//		timer2_counter++;
//		readAll(&hi2c1, &MPU9255);
//		//Time_start = HAL_GetTick();
//		pitch1 = MPU9255.pitch;
//		yaw1 = MPU9255.yaw;
//		roll1 = MPU9255.roll;
//		MPU_Flag = 1;
//		//Transmit angle to GUI
//		if (timer2_counter == 20) {
//			NRF24_Init();
//			NRF24_TxMode(NRF_Slave_Address, 10);
//
//			snprintf(NRF_Slave_Data, sizeof(NRF_Slave_Data),
//					"%3.2f,%3.2f,%3.2f\r\n", yaw1, pitch1, roll1);
//			if (NRF24_Transmit(NRF_Slave_Data) == 1) {
//				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
//			}
////			else
////				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
//			NRF24_Init();
//			NRF24_RxMode(NRF_Master_Address, 10);
//			timer2_counter = 0;
//		}
//		//it_timer_flag = 1;
//
//	}
//}
void Str_to_float(uint16_t u, uint8_t *y) {

	//y[5] = '\r';
	y[2] = u % 10 + 0x30;
	u = u / 10;
	y[1] = u % 10 + 0x30;
	u = u / 10;
	y[0] = u % 10 + 0x30;
}
void Get_PID(uint8_t array[]){
	uint8_t kp_pos=0, ki_pos=0, kd_pos=0;
	for(int i=0; i<32; i++){
		if(array[i] == '.'){
			if(kp_pos == 0)
				kp_pos = i;
			else if(ki_pos == 0)
				ki_pos = i;
			else if(kd_pos == 0)
				kd_pos = i;
		}
//		else if(array[i] == '\r')
//			end = i;
	}
	float a = 1;
	Kp = array[kp_pos-1] - 48;
	Ki = array[ki_pos-1] - 48;
	Kd = array[kd_pos-1] - 48;
	for(int i = 0; i<4; i++){
		a /= 10;
		Kp += (array[kp_pos+i+1] - 48)*a;
		Ki += (array[ki_pos+i+1] - 48)*a;
		Kd += (array[kd_pos+i+1] - 48)*a;
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartCommunication */
/**
 * @brief  Function implementing the Communication thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCommunication */
void StartCommunication(void *argument)
{
  /* USER CODE BEGIN 5 */
	//osThreadSuspend(ControlHandle);
//	while (MPU9255_Init(&hi2c1) == 1);
//	MPU_Flag = 1;
	//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
	NRF24_Init();
	//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
	//osDelay(3000);
	NRF24_RxMode(NRF_Master_Address, 10);
	//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
	//osThreadResume(ControlHandle);
	//ControlHandle = osThreadNew(StartControl, NULL, &Control_attributes);

	/* Infinite loop */
	for (;;) {
		if (isDataAvailable(1) == 1) {
			NRF24_Receive(NRF_RxData);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
			//Data is valid
			if(NRF_RxData[0] == '0'){
				switch (NRF_RxData[1]) {
				//Emer
				case '0':
					Mode = 0;
					Kp = 0;
					Ki = 0;
					Kd = 0;
					Roll_E0 = 0;
					Roll_E1 = 0;
					Roll_E2 = 0;
					Pitch_E0 = 0;
					Pitch_E1 = 0;
					Pitch_E2 = 0;
					Yaw_E0 = 0;
					Yaw_E1 = 0;
					Yaw_E2 = 0;
					Roll_Pre = 0;
					Pitch_Pre = 0;
					Yaw_Pre = 0;
					Roll_u1 = 0;
					Pitch_u1 = 0;
					Yaw_u1 = 0;
					break;
				//Single leg
				case '1':
					Mode = 1;
					break;
				//Single servo
				case '2':
					Mode = 2;
					break;
				//Forward
				case '3':
					Mode = 3;
					break;
				//Backward
				case '4':
					Mode = 4;
					break;
				//PID 05/01.200/00.010/00.123/-20/-05/-135
				case '5':{
					float temp = 0.001;
					Kp = 0;
					Ki = 0;
					Kd = 0;
					Roll_Ref = 0;
					Pitch_Ref = 0;
					Yaw_Ref = 0;

					for(int i=7; i>1; i--){
						if(i == 4){
							continue;
						}
						Kp += (NRF_RxData[i] - 0x30)*temp;
						Ki += (NRF_RxData[i+6] - 0x30)*temp;
						Kd += (NRF_RxData[i+12] - 0x30)*temp;
						if(i <= 3){
							Roll_Ref += (NRF_RxData[i+19] - 0x30)*temp;
							Pitch_Ref += (NRF_RxData[i+22] - 0x30)*temp;
							Yaw_Ref += (NRF_RxData[i+26] - 0x30)*temp;
						}
						temp*=10;
					}
					Yaw_Ref += (NRF_RxData[27] - 0x30)*temp;

					if(NRF_RxData[20] == '-'){
						Roll_Ref = -Roll_Ref;
					}
					if(NRF_RxData[23] == '-'){
						Pitch_Ref = -Pitch_Ref;
					}
					if(NRF_RxData[26] == '-'){
						Yaw_Ref = -Yaw_Ref;
					}
					Roll_E0 = 0;
					Roll_E1 = 0;
					Roll_E2 = 0;
					Pitch_E0 = 0;
					Pitch_E1 = 0;
					Pitch_E2 = 0;
					Yaw_E0 = 0;
					Yaw_E1 = 0;
					Yaw_E2 = 0;
					Roll_Pre = 0;
					Pitch_Pre = 0;
					Yaw_Pre = 0;
					Roll_u1 = 0;
					Pitch_u1 = 0;
					Yaw_u1 = 0;
					Mode = 5;
					break;
				}
				//Stand
				case '6':
					Mode = 6;
					break;
				//Sit
				case '7':
					Mode = 7;
					break;
				//Turn left
				case '8':
					Mode = 8;
					break;
				//Turn right
				case '9':
					Mode = 9;
					break;
				default:
					break;
				}
			}
			//Stop
			else {
				Mode = NRF_RxData[0] - 0x30;
				Mode *= 10;
				Mode += NRF_RxData[1] - 0x30;
//				Kp = 0;
//				Ki = 0;
//				Kd = 0;
//				Roll_E0 = 0;
//				Roll_E1 = 0;
//				Roll_E2 = 0;
//				Pitch_E0 = 0;
//				Pitch_E1 = 0;
//				Pitch_E2 = 0;
//				Yaw_E0 = 0;
//				Yaw_E1 = 0;
//				Yaw_E2 = 0;
//				Roll_Pre = 0;
//				Pitch_Pre = 0;
//				Yaw_Pre = 0;
//				Roll_u1 = 0;
//				Pitch_u1 = 0;
//				Yaw_u1 = 0;
			}

			if((Mode < 0) || Mode > 10){
				Mode = 10;
			}
//			if(NRF_RxData[1] == '5'){
//				Get_PID(NRF_RxData);//Cập nhật Kp, Ki, Kd
//			}
//			float distance = atof(NRF_RxData[3]);
//			//forward
//			int number_of_steps = (int) distance / 75;
//			if (((int) distance % 75) > 38) {
//				number_of_steps++;
//			}
//			if (NRF_RxData[1] == '3') {
//				for (int i = 0; i < number_of_steps; i++) {
//					Spider_Step_Forward(137, 75, 120);
//				}
//			} else if (NRF_RxData[1] == '4') {
//				for (int i = 0; i < number_of_steps; i++) {
//					Spider_Step_Backward_Arg(137, 75, 120);
//				}
//			}
		}
		//osThreadSuspend(ControlHandle);
		if(MPU_Flag){
			readAll(&hi2c1, &MPU9255);
			if((MPU9255.roll - roll1 < 20) && (MPU9255.roll - roll1 > -20))
				roll1 = MPU9255.roll;
			if((MPU9255.pitch - pitch1 < 20) && (MPU9255.pitch - pitch1 > -20))
				pitch1 = MPU9255.pitch;
	//		if((MPU9255.yaw - yaw1 < 50) && (MPU9255.yaw - yaw1 > -50))
				yaw1 = MPU9255.yaw;
	//		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
	//
			MPU_Counter++;
			//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
			osDelay(10);
		}

		//osThreadResume(ControlHandle);
		if(MPU_Counter >= 10){

			//osThreadSuspend(ControlHandle);
			//snprintf(NRF_Slave_Data, sizeof(NRF_Slave_Data), "%3.2f,%3.2f,%3.2f,%f, %3.2f\r\n", yaw1, pitch1, roll1, deltat, Now);
			sprintf(NRF_Slave_Data, "%3.2f,%3.2f,%3.2f\r\n", yaw1, pitch1, roll1);
			NRF24_Init();
			NRF24_TxMode(NRF_Slave_Address, 10);
			if (NRF24_Transmit(NRF_Slave_Data) == 1) {
				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
			}
			//sprintf(NRF_Slave_Data, "%3.2f,%3.2f,%3.2f\r\n", yaw1, pitch1, roll1);
//			else
//				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
			NRF24_Init();
			NRF24_RxMode(NRF_Master_Address, 10);
			//osThreadResume(ControlHandle);
			MPU_Counter = 0;
		 }
//		if(Mode == 5){
//			T_PID_Yaw_pre = T_PID_Yaw;
//			T_PID_Yaw = xTaskGetTickCount()* portTICK_PERIOD_MS;
//			//PID(roll1, pitch1, T_PID - T_PID_pre);
//			PID_Yaw(yaw1, T_PID_Yaw - T_PID_Yaw_pre);
//
//		}


		//readAll(&hi2c1, &MPU9255);
		//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);

	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartControl */
/**
 * @brief Function implementing the Control thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartControl */
void StartControl(void *argument)
{
  /* USER CODE BEGIN StartControl */
	//osThreadSuspend(CommunicationHandle);
	while (MPU9255_Init(&hi2c1) == 1);
	MPU_Flag = 1;
	PCA9685_Init(&hi2c2);
	//osDelay(1000);
	Spider_Init();
	//osThreadResume(CommunicationHandle);
	/* Infinite loop */
	for (;;) {
		//PID(roll1, pitch1, 20);
		//NRF_RxData
		switch (Mode) {
			//Stop
			case 0:
				osDelay(500);
				break;
			//Single leg 01:0137.00,-075.00,-120.00,2\r\n
			case 1:{
				float x = 0, y = 0, z = 0, temp;
				uint8_t Leg_id;
				Leg_id = NRF_RxData[27] - 0x30;
				temp = 1;
				for(int i=6; i>3; i--){
					x += (NRF_RxData[i] - 0x30)*temp;
					y += (NRF_RxData[i+8] - 0x30)*temp;
					z += (NRF_RxData[i+16] - 0x30)*temp;
					temp*=10;
				}
				temp = 0.01;
				for(int i=9; i>7; i--){
					x += (NRF_RxData[i] - 0x30)*temp;
					y += (NRF_RxData[i+8] - 0x30)*temp;
					z += (NRF_RxData[i+16] - 0x30)*temp;
					temp*=10;
				}
				if(NRF_RxData[3] == '-'){
					x = -x;
				}
				if(NRF_RxData[11] == '-'){
					y = -y;
				}
				if(NRF_RxData[19] == '-'){
					z = -z;
				}
				Inverse_Calc(array_Leg[Leg_id-1], x, y, z);
				ControlLeg(array_Leg[Leg_id-1]);
				Mode = 10;
				break;
			}

			//Single servo 02:02=0090
			case 2:{
				uint8_t servo_id, temp;
				float angle = 0;
				servo_id = (NRF_RxData[3] - 0x30)*10;
				servo_id += (NRF_RxData[4] - 0x30);
				temp = 1;
				for(int i=9; i>6; i--){
					angle += (NRF_RxData[i] - 0x30)*temp;
					temp*=10;
				}
				if(NRF_RxData[6] == '-'){
					angle = -angle;
				}
				angle += array_Servo[servo_id]->Angle_offset;
				ControlServo(array_Servo[servo_id],angle);
				Mode = 10;
				break;
			}

			//Forward 03:123456
			case 3:{
				float distance = 0;
				float temp = 1;
				for(int i=8; i>2; i--){
					distance += (NRF_RxData[i] - 0x30)*temp;
					temp*=10;
				}
				//76
				int number_of_steps = (int) distance / 152;
				if (((int) distance % 152) > 76) {
					number_of_steps++;
				}
				//end 76

				//100
//				int number_of_steps = (int) distance / 200;
//				if (((int) distance % 200) > 100) {
//					number_of_steps++;
//				}
				//end 100
				float each_step = distance/ (float) number_of_steps / 2;
				for(int i = 0; i < number_of_steps; i++){
					if((Mode == 10) || (Mode == 0)){
						break;
					}
					Spider_Step_Forward_Arg(160, each_step, -80);
					//Spider_Step_Forward_Arg(137, each_step, -120);
				}
				Mode = 10;
				break;
			}

			//Backward
			case 4:{
				float distance = 0;
				float temp = 1;
				for(int i=8; i>2; i--){
					distance += (NRF_RxData[i] - 0x30)*temp;
					temp*=10;
				}
				//76
				int number_of_steps = (int) distance / 152;
				if (((int) distance % 152) > 76) {
					number_of_steps++;
				}
				//end 76

				//100
//				int number_of_steps = (int) distance / 200;
//				if (((int) distance % 200) > 100) {
//					number_of_steps++;
//				}
				//end 100
				float each_step = distance/ (float) number_of_steps / 2;
				for(int i = 0; i < number_of_steps; i++){
					if((Mode == 10) || (Mode == 0)){
						break;
					}
					//Spider_Step_Backward_Arg(137, each_step, -120);
					Spider_Step_Backward_Arg(160, each_step, -80);
				}
				Mode = 10;
				break;
			}

			//PID 05/01.200/00.010/00.123/-20/-05/-135
			case 5:{
				//Roll: trước sau, Pitch: trái phải
				//Get_PID(NRF_RxData);
				if(Mode == 5){
					T_PID_pre = T_PID;
					T_PID = xTaskGetTickCount()* portTICK_PERIOD_MS;
					PID(roll1, pitch1, T_PID - T_PID_pre);

					Inverse_Calc(&LegLF, LegLF.x, LegLF.y, -120 + Pitch_u - Roll_u);
					ControlLeg(&LegLF);
					Inverse_Calc(&LegLR, LegLR.x, LegLR.y, -120 + Pitch_u + Roll_u);
					ControlLeg(&LegLR);
					Inverse_Calc(&LegRR, LegRR.x, LegRR.y, -120 - Pitch_u + Roll_u);
					ControlLeg(&LegRR);
					Inverse_Calc(&LegRF, LegRF.x, LegRF.y, -120 - Pitch_u - Roll_u);
					ControlLeg(&LegRF);
					//Spider_Step_Forward_Yaw_Arg(137, 75, -120);
					//osDelay(20);

					break;
				}

			}

			//Stand
			case 6:
				Spider_stand();
				Mode = 10;
				break;
			//Sit
			case 7:
				Spider_Sit();
				Mode = 10;
				break;
			//Turn left 08:01/r/n
			case 8:{
				int step = 0;
				step = NRF_RxData[3] - 0x30;
				step *= 10;
				step += NRF_RxData[4] - 0x30;
				for(int i=0; i < step; i++){
					if((Mode == 10) || (Mode == 0)){
						break;
					}
					Spider_Turn_Left();
				}
				Mode = 10;
				break;
			}

			//Turn right
			case 9:{
				int step = 0;
				step = NRF_RxData[3] - 0x30;
				step *= 10;
				step += NRF_RxData[4] - 0x30;
				for(int i=0; i < step; i++){
					if((Mode == 10) || (Mode == 0)){
						break;
					}
					Spider_Turn_Right();
				}
				Mode = 10;
				break;
			}

			case 10:
				osDelay(500);
				break;
			default:
				osDelay(500);
				break;
		}
//		PID(roll1, pitch1, 20);
//		//			MPU_Counter++;
//		//			if(MPU_Counter > 100){
//		Inverse_Calc(&LegLF, 120, LegLF.y, -100 + Pitch_u - Roll_u);
//		ControlLeg(&LegLF);
//		Inverse_Calc(&LegLR, 120, LegLR.y, -100 + Pitch_u + Roll_u);
//		ControlLeg(&LegLR);
//		Inverse_Calc(&LegRR, 120, LegRR.y, -100 - Pitch_u + Roll_u);
//		ControlLeg(&LegRR);
//		Inverse_Calc(&LegRF, 120, LegRF.y, -100 - Pitch_u - Roll_u);
//		ControlLeg(&LegRF);

		//DEBUG
//		();
//		osDelay(1000);
//		Spider_standSpider_Sit();
//		for(int Leg_number = 0; Leg_number < 4; Leg_number++){
//			InverseDirect(array_Leg[Leg_number], array_Leg[Leg_number]->target_x, array_Leg[Leg_number]->target_y, array_Leg[Leg_number]->target_z);
//		}
		//Spider_Step_Forward();
//		Spider_Step_Forward_Arg(137, 75, -120);
//		osDelay(2000);
//		Spider_Step_Backward_Arg(137, 75, -120);
//		osDelay(2000);
		////					}
//		Spider_Turn_Right();
//		osDelay(2000);
//		Spider_Turn_Left();
////		MPU_Counter = 0;
////		Spider_Step_Forward_Arg(137, 75, -120);
//		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
////		Spider_Step_Backward_Arg(137, 75, -120);
//		osDelay(2000);
	}
  /* USER CODE END StartControl */
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
//
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
//
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
	while (1) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, SET);
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
