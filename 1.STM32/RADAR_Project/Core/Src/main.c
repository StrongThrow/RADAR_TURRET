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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include "esp.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern TIM_HandleTypeDef htim1;
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
#define ARR_CNT 5
#define CMD_SIZE 50
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*디버그 모드*/
#define DEBUG_MODE 1

/*파이*/
#define PI 3.14159

/*서보모터 관련 설정*/

	//서보모터 각도 MIN, MAX값
#define SERVO_MIN 500
#define SERVO_MAX 2500

	//몇 도씩 스캔을 할 것인지
#define SCAN_DGREE 10

/*SR04 관련 설정*/

	//SR04관련 필터
#define SR04_TIMEOUT 100 //SR04 무응답 타임아웃 (단위 ms)
#define SR04_MAX_US_HW 25000 //초음파 돌아오는 시간의 최대값
#define SR04_MIN_US 120 //초음파 돌아오는 시간의 최솟값

	//각도당 스캔을 몇 번 할지
#define SCAN_CNT 5

	//탐색 모드에서 얼마나 탐색하고 지형지물을 업데이트 할지(숫자만큼 스캔하고 다시 지형지물 업데이트)
#define SCAN_TICK 5

	//몇 cm 까지 스캔 할 것인지
#define MAX_SCAN_CM 200

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* Definitions for oledTask */
osThreadId_t oledTaskHandle;
const osThreadAttr_t oledTask_attributes = {
  .name = "oledTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for servoTask */
osThreadId_t servoTaskHandle;
const osThreadAttr_t servoTask_attributes = {
  .name = "servoTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for btTask */
osThreadId_t btTaskHandle;
const osThreadAttr_t btTask_attributes = {
  .name = "btTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
volatile uint8_t sr04_state = 0;
volatile uint32_t ic_rising, ic_falling = 0;

typedef struct{
	double x;
	double y;
}Pos;

//모터의 각도
int degree = 0;
//측정한 거리
double distance = -1.0;
//계산한 각도
Pos pos;

uint8_t find_flag = 0;

//스캔 모드
typedef enum{
	get_landform,
	scan_object
}RADAR_STATE;


//모드 선언
volatile RADAR_STATE radar_state = get_landform;

//모든 각도의 지형지물 거리 값
double landform_data[(180 /  SCAN_DGREE)+ 1];

double scan_buffer[SCAN_CNT];

//ESP01 와이파이 모듈 관련 변수
uint8_t rx2char;
extern cb_data_t cb_data;
extern volatile unsigned char rx2Flag;
extern volatile char rx2Data[50];
char sendBuf[MAX_UART_COMMAND_LEN]={0};

//ESP01 태스크 큐
#define TXMSG_MAX 128

typedef struct{
	uint16_t len;
	char payload[TXMSG_MAX];
}TxMsg;

osMessageQueueId_t txQHandle;
const osMessageQueueAttr_t txQ_attributes = {
  .name = "txQ"
};

volatile uint8_t shoot_flag = 0;
char strBuff[MAX_ESP_COMMAND_LEN];
extern volatile unsigned char rx2Flag;
extern volatile char rx2Data[50];
extern cb_data_t cb_data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);

/* USER CODE BEGIN PFP */
uint16_t angle_to_ccr(int angle);
static inline void sr04_trigger_pulse();
void sr04_start_measure();
double sr04_read_cm_rtos(TickType_t timeout_ticks);
static inline void dwt_init(void);
static inline void delay_us(uint32_t us);
void calculate_degree(Pos *pos, double distance, int angle);
void scan_landform();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void esp_event(char * recvBuf)
{
  int i=0;
  char * pToken;
  char * pArray[ARR_CNT]={0};


  strBuff[strlen(recvBuf)-1] = '\0';	//'\n' cut
  printf("\r\nDebug recv : %s\r\n",recvBuf);

  pToken = strtok(recvBuf,"[@]");
  while(pToken != NULL)
  {
    pArray[i] = pToken;
    if(++i >= ARR_CNT)
      break;
    pToken = strtok(NULL,"[@]");
  }

  if(!strcmp(pArray[1],"OK"))
  {
  	shoot_flag = 1;
  }
  else if(!strncmp(pArray[1]," New conn",8))
  {
//	   printf("Debug : %s, %s\r\n",pArray[0],pArray[1]);
     return;
  }
  else if(!strncmp(pArray[1]," Already log",8))
  {
// 	    printf("Debug : %s, %s\r\n",pArray[0],pArray[1]);
	  esp_client_conn();
      return;
  }
  else
      return;

}

void calculate_degree(Pos *pos, double distance, int angle) {

	if (distance <= 0.0) {
		pos->x = 0.0;
		pos->y = 0.0;
		return;
	} // out-of-range 보호
	if (angle < 0)
		angle = 0;
	if (angle > 180)
		angle = 180;

	double rad = angle * (PI / 180.0);

	// 0°→x=-d, y=0 / 90°→x=0, y=+d / 180°→x=+d, y=0
	pos->x = distance * cos(rad); // before -distance
	pos->y = distance * sin(rad);

	// 부동소수 오차로 -0.0000이 찍히는 것 방지(선택)
	if (fabs(pos->x) < 1e-9)
		pos->x = 0.0;
	if (fabs(pos->y) < 1e-9)
		pos->y = 0.0;
}

// DWT 사이클 카운터 활성화 (부팅 후 1회)
static inline void dwt_init(void)
{
    // Trace enable
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // Reset counter
    DWT->CYCCNT = 0;
    // Enable cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// 마이크로초 지연 (busy-wait)
static inline void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = (SystemCoreClock / 1000000U) * us; // 1us당 사이클 수
    while ((DWT->CYCCNT - start) < ticks) { __NOP(); }
}

uint16_t angle_to_ccr(int angle){
	if(angle < 0) angle = 0;
	if(angle > 180) angle = 180;
	return SERVO_MIN + (SERVO_MAX - SERVO_MIN) * angle / 180;
}

static inline void sr04_trigger_pulse(){
	//PB1 10us HIGH 유지
	HAL_GPIO_WritePin(SR04_TRIGGER_GPIO_Port, SR04_TRIGGER_Pin, GPIO_PIN_RESET);
	delay_us(2);
	HAL_GPIO_WritePin(SR04_TRIGGER_GPIO_Port, SR04_TRIGGER_Pin, GPIO_PIN_SET);
	delay_us(10);
	HAL_GPIO_WritePin(SR04_TRIGGER_GPIO_Port, SR04_TRIGGER_Pin, GPIO_PIN_RESET);
}

void sr04_start_measure(){
	//sr04 상태 변수 초기화
	sr04_state &= ~(0x03);
	//TIM5 CNT값 0으로 초기화
	__HAL_TIM_SET_COUNTER(&htim5, 0);

	//입력 캡쳐 인터럽트 모드 시작
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1); // 라이징 엣지
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2); // 폴링 엣지

	//트리거 활성화
	sr04_trigger_pulse();
}

double sr04_read_cm_rtos(TickType_t timeout_ticks){

	sr04_start_measure();

	TickType_t start = xTaskGetTickCount();
	for(;;){
		//라이징, 폴링 엣지 모두 측정되면 break
		if((sr04_state & 0x03) == 0x03) break;

		//타임아웃
		if((xTaskGetTickCount() - start) > timeout_ticks){
			HAL_TIM_IC_Stop_IT(&htim5, TIM_CHANNEL_1);
			HAL_TIM_IC_Stop_IT(&htim5, TIM_CHANNEL_2);
			return -1.0;
		}
		vTaskDelay(1);
	}

	//타이머 인터럽트 종료
	HAL_TIM_IC_Stop_IT(&htim5, TIM_CHANNEL_1);
	HAL_TIM_IC_Stop_IT(&htim5, TIM_CHANNEL_2);

	uint32_t width_us = (uint32_t)(ic_falling - ic_rising);

	if (width_us < SR04_MIN_US || width_us > SR04_MAX_US_HW)
		return -1.0;

	double cm = (double) width_us * 0.01715;   // 왕복시간→거리(cm)
	if (cm > MAX_SCAN_CM)
		return -1.0;
	return cm;
	//return (double)width_us * 0.01715f;

}

static void bubble_sort(double arr[], int n){
	for(int i = 0; i < n - 1; i++){
		for(int j = 0; j < n - 1 - i; j++){
			if(arr[j] > arr[j + 1]){
				double temp = arr[j];
				arr[j] = arr[j + 1];
				arr[j + 1] = temp;
			}
		}
	}
}

void scan_landform() {
	//한 방위 5번 스캔
	for (int i = 0; i < SCAN_CNT; i++) {
		scan_buffer[i] = sr04_read_cm_rtos(pdMS_TO_TICKS(SR04_TIMEOUT));
	}

	//스캔한 거리값 데이터 정렬
	bubble_sort(scan_buffer, SCAN_CNT);

	//중앙값 추출 후 지형 데이터에 추가
	double value = scan_buffer[SCAN_CNT / 2];
	if(value == -1.0){
		for(int i = SCAN_CNT / 2 + 1; i < SCAN_CNT; i++){
			if(scan_buffer[i] != -1.0){
				value = scan_buffer[i];
				break;
			}
		}
	}
	landform_data[degree / SCAN_DGREE] = value;
}

void find_object() {

	//한 방위 5번 스캔
	for (int i = 0; i < SCAN_CNT; i++) {
		scan_buffer[i] = sr04_read_cm_rtos(pdMS_TO_TICKS(SR04_TIMEOUT));
	}
	//스캔한 거리값 데이터 정렬
	bubble_sort(scan_buffer, SCAN_CNT);

	//스캔 데이터 추출
	double value = scan_buffer[SCAN_CNT / 2];

	if(value == -1.0){
		for(int i = SCAN_CNT / 2 + 1; i < SCAN_CNT; i++){
			if(scan_buffer[i] != -1.0){
				value = scan_buffer[i];
				break;
			}
		}
	}

	distance = value;
	if(distance < 0.0) return;


	//지형지물 데이터에서 일정 범위 벗어난 측정이면
	if (landform_data[degree / SCAN_DGREE] - 10.0 > distance || distance > landform_data[degree / SCAN_DGREE] + 10.0) {
		//그 물체의 거리 계산
		calculate_degree(&pos, distance, degree);

		find_flag = 1;

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
	int ret = 0;
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
	MX_USART2_UART_Init();
	MX_TIM5_Init();
	MX_USART1_UART_Init();
	MX_USART6_UART_Init();
	/* USER CODE BEGIN 2 */
	dwt_init();
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	//ESP01 와이파이 접속
	printf("Start main() - wifi\r\n");
	//ret |= drv_uart_init();
	ret |= drv_esp_init();
	if (ret != 0) {
		printf("Esp response error\r\n");
		Error_Handler();

	}

	AiotClient_Init();
	txQHandle = osMessageQueueNew(10, sizeof(TxMsg), &txQ_attributes);
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
  /* creation of oledTask */
  oledTaskHandle = osThreadNew(StartDefaultTask, NULL, &oledTask_attributes);

  /* creation of servoTask */
  servoTaskHandle = osThreadNew(StartTask02, NULL, &servoTask_attributes);

  /* creation of btTask */
  btTaskHandle = osThreadNew(StartTask03, NULL, &btTask_attributes);

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
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 83;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 8;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 38400;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  HAL_GPIO_WritePin(SR04_TRIGGER_GPIO_Port, SR04_TRIGGER_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : SR04_TRIGGER_Pin */
  GPIO_InitStruct.Pin = SR04_TRIGGER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SR04_TRIGGER_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	//TIM5가 아니면 return
    if (htim->Instance != TIM5) return;

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        ic_rising = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        //__HAL_TIM_SET_COUNTER(htim, 0);
        sr04_state |= 0x01;
    } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
        ic_falling = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        sr04_state |= 0x02;
    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * @brief Function implementing the servoTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */

	static uint8_t scan_tick = 0;

	/* Infinite loop */
	for (;;) {

		//거리 측정
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, angle_to_ccr(degree));
		vTaskDelay(pdMS_TO_TICKS(100)); // 서보 정착시간(옵션, 10~20ms 권장)

		switch (radar_state) {
			case get_landform:
				scan_landform();
#if DEBUG_MODE
				if (radar_state == get_landform && (degree % 10) == 0) {
					char b[48];
					int len = snprintf(b, sizeof(b), "[cal] deg=%3d, dist = %.2f\n", degree, landform_data[degree / SCAN_DGREE]);
					HAL_UART_Transmit(&huart2, (uint8_t*)b, len, 50);
				}
#endif

				break;

			case scan_object:

				find_object();
				if (find_flag == 1) {

					sprintf(sendBuf, "[%s]%.2f@%.2f\n", "TURRET_1", pos.x, pos.y);
					esp_send_data(sendBuf);
					vTaskDelay(pdMS_TO_TICKS(5));
					memset(sendBuf, 0, sizeof(sendBuf));
					sprintf(sendBuf, "[%s]SETDB@%s@%.2f,%.2f\n", "SQL", "RADAR", pos.x, pos.y);
					esp_send_data(sendBuf);
					vTaskDelay(pdMS_TO_TICKS(5));
					memset(sendBuf, 0, sizeof(sendBuf));
					sprintf(sendBuf, "[%s]%.2f@%.2f\n", "13", pos.x, pos.y);
					esp_send_data(sendBuf);

					while(shoot_flag == 0){
						vTaskDelay(5);  // 한 틱 양보: busy-wait 방지
					}
					shoot_flag = 0;

#if DEBUG_MODE
					char buffer[128];
					if (distance < 0.0) {
						int len = snprintf(buffer, sizeof(buffer), "deg=%3d, dist=out-of-range\r\n", degree);
						HAL_UART_Transmit(&huart2, (uint8_t*) buffer, len, HAL_MAX_DELAY);
					} else {
						int len = snprintf(buffer, sizeof(buffer), "deg=%3d, landform =%.2f cm, dist=%.2f cm, xPos=%.2f cm, yPos=%.2f cm\r\n", degree, landform_data[degree / SCAN_DGREE], distance, pos.x,
								pos.y);
						HAL_UART_Transmit(&huart2, (uint8_t*) buffer, len, HAL_MAX_DELAY);
					}
#endif
					find_flag = 0;
				}




				break;

		}

		//각도 변환 및 상태 변경
		if ((sr04_state & 0x04) == 0x00) {
			degree += SCAN_DGREE;
			//180도에 도달하면
			if (degree >= 180) {
				//상태 플래그 변경(degree 증가로)
				degree = 180;
				sr04_state |= 0x04;
			}
		}
		else if ((sr04_state & 0x04) == 0x04) {
			degree -= SCAN_DGREE;
			//0도에 도달하면
			if (degree <= 0) {
				//상태 플래그 변경(degree 감소로)
				degree = 0;
				sr04_state &= ~(0x04);
				//만약 지형지물 탐색을 한 번 완료했으면
				if (radar_state == get_landform)
					//일반 탐색 모드로 변경
					radar_state = scan_object;

				//일반 탐색 모드이면
				else if (radar_state == scan_object)
					//스캔 틱 1씩 증가
					scan_tick++;
			}

			if (scan_tick > SCAN_TICK) {
				scan_tick = 0;
				radar_state = get_landform;
				//memset(landform_data, 0, sizeof(landform_data));
			}
		}

		vTaskDelay(pdMS_TO_TICKS(50));
	}
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the btTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument) {
	/* USER CODE BEGIN StartTask03 */

	/* Infinite loop */
	for (;;) {
		if (strstr((char*) cb_data.buf, "+IPD") && cb_data.buf[cb_data.length - 1] == '\n') {
			strcpy(strBuff, strchr((char*) cb_data.buf, '['));
			memset(cb_data.buf, 0x0, sizeof(cb_data.buf));
			cb_data.length = 0;
			esp_event(strBuff);
		}
		if (rx2Flag) {
			printf("recv2 : %s\r\n", rx2Data);
			rx2Flag = 0;
		}
		vTaskDelay(10);
	}


	/* USER CODE END StartTask03 */
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
  if (htim->Instance == TIM2)
  {
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
	while (1) {
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
