/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "serialport.h"
#include "string.h"
#include "utility.h"
#include "stdio.h"
#include "ltc_stack.h"
#include "soc_ekf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct analog_result_type{
	int16_t output_current;
	int16_t output_voltage;
	int16_t accumulator_voltage;
	int32_t output_power;
	uint32_t tick;
};

struct accumulator_result_type{
	int16_t total_voltage;
	int16_t min_cell_voltage;
	int16_t max_cell_voltage;
	int16_t temperature;
};

//#define LTC_DEVICES_IN_STACK	15
//struct ltc_stack_type{
//	SPI_HandleTypeDef *hspi;
//	GPIO_TypeDef *CS_GPIO;
//	uint16_t CS_PIN;
//
//	uint16_t cell_voltages[12 * LTC_DEVICES_IN_STACK];
//	int16_t temperatures[5 * LTC_DEVICES_IN_STACK];
//	uint8_t balance_flags[12 * LTC_DEVICES_IN_STACK];
//	uint8_t balancing_activation;
//	uint16_t balance_voltage;
//
//	uint8_t spi_comm_state;
//	uint32_t spi_comm_next_tick; // for delay control for conversions
//	// constant messages
//	uint8_t spi_msg_config[4 + 8*LTC_DEVICES_IN_STACK];
//	uint8_t spi_msg_adcv[4];
//	uint8_t spi_msg_adax[4];
//	uint8_t spi_msg_read_cell_a[4];
//	uint8_t spi_msg_read_cell_b[4];
//	uint8_t spi_msg_read_cell_c[4];
//	uint8_t spi_msg_read_cell_d[4];
//	uint8_t spi_msg_read_aux_a[4];
//	uint8_t spi_msg_read_aux_b[4];
//	// not constant messages buffers
//	uint8_t spi_tx_buffer[4 + 8*LTC_DEVICES_IN_STACK];
//	uint8_t spi_rx_buffer[4 + 8*LTC_DEVICES_IN_STACK];
//	uint8_t tx_busy;
//	uint8_t rx_ready;
//};

struct analog_type{
	ADC_HandleTypeDef *hadc;
	TIM_HandleTypeDef *htim;

	uint16_t raw_buf[400*4];
	uint16_t raw_tail;

	struct analog_result_type result_buf[10];
	uint16_t result_tail;
	uint16_t result_head;
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COEFF_VOLT_PER_ADC		1.7241f	// L 10k, 22k; H 200k, 10k
#define COEFF_AMPERE_PER_ADC	0.12891f

#define CANBUS_MODE_CAR		0x01
#define CANBUS_MODE_CHARGER	0x02
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
struct analog_type analog;

uint16_t analog_avg[4];
uint16_t analog_max;
uint16_t output_voltage;
int16_t output_current;

uint8_t sd_log_activated;
char filename[50];
char filename_current[50];
FATFS fat_fs;
FIL file;
FIL file_current;
//char line[100];
FRESULT fat_result;
uint8_t sd_buffer[3000];
uint16_t sd_buffer_tail = 0;
uint16_t sd_bytes_to_write = 0;
uint32_t aaa_us_cntr = 0;

uint32_t test_time = 0;

float test_soc = 0.0f;
//float voltages[10*DAISY_SIZE];

extern struct stack_data_type stack_data;

volatile uint8_t global_stack_fault = 0;

uint8_t can_mode = CANBUS_MODE_CAR;

// ------ SOC
float Batt_model_EPA642128HP[] = { 0.0028, 7.746989, 50.867289, 0.005505, 0.009222, 7.254};
const unsigned int Batt_model_length = sizeof(Batt_model_EPA642128HP) / sizeof(Batt_model_EPA642128HP[0]);
static_assert(6 == Batt_model_length, "invalid battery model");

const float OCV_poly_EPA642128HP[] = {-15155.047856, 103529.522834, -311035.126560, 542047.009130, -608536.117000,
									  462164.931617, -242600.493645, 88307.143064, -22000.025647, 3631.414052,
								      -374.018295, 21.868083, 3.148197 };
const unsigned int EPA642128HP_ocv_length = sizeof(OCV_poly_EPA642128HP) / sizeof(OCV_poly_EPA642128HP[0]);
static_assert(SOC_OCV_poli_coeff_lenght == EPA642128HP_ocv_length, "invalid number of coef in soc-ocv polynomial");

SoC_EKF stack_soc;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */

void CAN_Init_Charger();
void AnalogInit(ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim);
void AnalogThread();
uint16_t Analog_Get_Fill_Level();
void MainInterruptCallback();
void RelayControlThread();
void FaultThread();
void SdInitFile();
void SdSaveProcess();

void ConsoleSimple();
void CanbusThread();

// SOC
void SocInit();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void CAN_Init_Charger()
{
	  hcan1.Instance = CAN1;
	  hcan1.Init.Prescaler = 18;
	  hcan1.Init.Mode = CAN_MODE_NORMAL;
	  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
	  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
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
	  can_mode = CANBUS_MODE_CHARGER;
}

void AnalogInit(ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim)
{
	analog.hadc = hadc;
	analog.htim = htim;

	HAL_ADC_Start_DMA(hadc, (uint32_t*)&analog.raw_buf[0], 1600);
	HAL_TIM_Base_Start(htim);
}

void AnalogThread()
{
	if (Analog_Get_Fill_Level() >= 20)
	{
		// calculate values
		uint32_t sum[4] = {0};
		for (int i = analog.raw_tail; i < analog.raw_tail+20; i+=4)
		{
			sum[0] += analog.raw_buf[i];
			sum[1] += analog.raw_buf[i+1];
			sum[2] += analog.raw_buf[i+2];
			sum[3] += analog.raw_buf[i+3];
		}

		analog_avg[0] = sum[0] / 5;
		analog_avg[1] = sum[1] / 5;
		analog_avg[2] = sum[2] / 5;
		analog_avg[3] = sum[3] / 5;

		output_voltage = (uint16_t)((float)analog_avg[2]*COEFF_VOLT_PER_ADC);
		//output_current = (int16_t)
//		uint16_t max = 0;
//		for (int i = analog.raw_tail; i < analog.raw_tail+400; i+=4)
//		{
//			if (analog.raw_buf[i+1] > max)
//				max = analog.raw_buf[i+1];
//
//		}
//		analog_max = max;

		// advance tail
		analog.raw_tail += 20;
		analog.raw_tail %= 1600;
	}
}

uint16_t Analog_Get_Fill_Level()
{
	uint32_t head = 1600 - analog.hadc->DMA_Handle->Instance->NDTR;
	uint32_t tail = analog.raw_tail;
	if (head > tail) return head - tail;
	else if (head < tail) return 1600 - tail + head;
	else return 0;
}

void SdInitFile()
{
	char folder_path[20];
	RTC_DateTypeDef rtc_date;
	RTC_TimeTypeDef rtc_time;
	HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);
	if (BSP_SD_IsDetected() == SD_NOT_PRESENT)
	{
		sd_log_activated = 0;
		return;
	}

	f_mount(&fat_fs, SDPath, 0);

	sprintf(folder_path, "%02d%02d%02d", (rtc_date.Year%100), rtc_date.Month, rtc_date.Date);
	fat_result = f_mkdir(folder_path);

	sprintf(folder_path, "%s/%02d%02d", folder_path, rtc_time.Hours, rtc_time.Minutes);
	fat_result = f_mkdir(folder_path);

	// main log file
	uint16_t file_no = 1;
	do{
		sprintf(filename, "%s/log_%d.txt", folder_path, file_no);
		fat_result = f_open(&file, filename, FA_CREATE_NEW|FA_WRITE);
		file_no++;
	}while(fat_result == FR_EXIST && file_no != 0);

	// file with current data
	file_no = 1;
	do{
		sprintf(filename_current, "%s/I_%d.txt", folder_path, file_no);
		fat_result = f_open(&file_current, filename_current, FA_CREATE_NEW|FA_WRITE);
		file_no++;
	}while(fat_result == FR_EXIST && file_no != 0);

	if(fat_result == FR_OK)
		sd_log_activated = 1;
	else
		sd_log_activated = 0;
}

void SdSaveProcess()
{
	// main log
	if(sd_bytes_to_write != sd_buffer_tail && sd_log_activated == 1)
	{
		htim1.Instance->CNT = 0;
		//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
		uint16_t length_to_write = sd_bytes_to_write - sd_buffer_tail;
		//if(length_to_write > 750) length_to_write = 750;

		UINT bytes_written = 0;
		// write to file
		//fat_result = f_open(&file, filename, FA_WRITE | FA_OPEN_APPEND);
		fat_result = f_write(&file, &sd_buffer[sd_buffer_tail], length_to_write, &bytes_written);
		fat_result = f_sync(&file);

		sd_buffer_tail += length_to_write;

		if(fat_result != FR_OK || length_to_write != bytes_written)
		{
			sd_log_activated = 0;
			sd_buffer_tail = 0;
			sd_bytes_to_write = 0;
		}
		//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);

		aaa_us_cntr = htim1.Instance->CNT;
	}

//	static uint8_t sd_i_buffer[1000];
//	static uint16_t n = 0;
//	static uint32_t samples = 0;
//	// I log
//	if (output_current_buffer_ready == 1)
//	{
//		n = 0;
//		for (int i = 0; i < 50; i++)
//		{
//			samples++;
//			n += sprintf(&sd_i_buffer[n], "%d;%3.2f\n", samples, output_current_buffer[i]);
//		}
//
//		UINT bytes_written = 0;
//		fat_result = f_write(&file_current, &sd_i_buffer[0], n, &bytes_written);
//		fat_result = f_sync(&file_current);
//
//		if(fat_result != FR_OK || n != bytes_written)
//		{
//			sd_log_activated = 0;
//			sd_buffer_tail = 0;
//			sd_bytes_to_write = 0;
//		}
//
//		output_current_buffer_ready = 0;
//	}
}

void MainInterruptCallback()
{
	static uint8_t thread_busy = 0;

	if (thread_busy == 1) return;

	thread_busy = 1;

	// control of the bms
	RelayControlThread();
	AnalogThread();
	LtcCommunicationThread();
	ConsoleSimple();
	CanbusThread();

	FaultThread();


	thread_busy = 0;
}

/*
 * brief:	control of the AIRs
 * param:	none
 * retval:	none
 */
void RelayControlThread()
{
	static uint8_t thread_state = 0;
	static uint32_t next_tick = 0;

	uint8_t safety_state = SafetyStateRead();

	// switch relays off when safety is missing
	if (thread_state != 0 && safety_state == 0)
	{
		// switch AIRs off
		AIR_MINUS_Set(0);
		AIR_PLUS_Set(0);
		AIR_PRECHARGE_Set(0);
		// turn off led
		LedSet(2, 0);
		thread_state = 0;
	}

	if (thread_state == 0)
	{
		// switch on AIR MINUS when safety voltage detected
		if (safety_state == 1)
		{
			next_tick = HAL_GetTick() + 100;
			AIR_MINUS_Set(1);
			thread_state++;
		}
	}
	else if (thread_state == 1)
	{
		// wait after first switch
		if (next_tick <= HAL_GetTick())
		{
			// switch on the precharge
			AIR_PRECHARGE_Set(1);
			next_tick = HAL_GetTick() + 10000;
			thread_state++;
		}
	}
	else if (thread_state == 2)
	{
		// wait - the capacitors are precharging
		if (next_tick <= HAL_GetTick())
		{
			// switch on main PLUS
			AIR_PLUS_Set(1);
			next_tick = HAL_GetTick() + 100;
			thread_state++;
		}
	}
	else if (thread_state == 3)
	{
		// switch off precharge relay
		if (next_tick <= HAL_GetTick())
		{
			AIR_PRECHARGE_Set(0);
			LedSet(2, 1);
			thread_state++;
		}
	}
	else if (thread_state == 4)
	{
		// full on
	}
}

/*
 * brief:	controls the safety relay
 * param:	none
 * retval:	none
 */
void FaultThread()
{
	// initial
	// set high after init for the first time, only when there is no stack fault
	if (HAL_GetTick() > 500 && HAL_GetTick() < 2000)
	{
		if (LtcGetStackError() == 0)
		{
			FaultOutputSet(1);
			LedSet(1, 1);
		}
	}
	else if (HAL_GetTick() < 2500)
	{
		FaultOutputSet(0);
	}
	else
	{
		// when stack fault occurred switch off the safety relay
		if (LtcGetStackError() != 0)
		{
			FaultOutputSet(1);
			LedSet(1, 0);
		}
	}
}

void ConsoleSimple()
{
	static uint32_t last_data_tick = 0;
	if (last_data_tick == stack_data.data_refresh_tick) return;
	last_data_tick = stack_data.data_refresh_tick;

	char string[1000], dis_char;
	uint16_t n = 0;
	for (int i = 0; i < 27; i++)
	{
		dis_char = ' ';
		if (stack_data.discharge[i] == 1) dis_char = 'D';
		n += sprintf(&string[n], "%d%c\t", stack_data.voltages[i], dis_char);
	}
	string[n++] = '\r';
	string[n++] = '\n';
	for (int i = 0; i < 9; i++)
	{
		n += sprintf(&string[n], "%d\t", stack_data.temperatures[i]);
	}
	string[n++] = '\r';
	string[n++] = '\n';

	SerialportWrite((uint8_t*)string, n);
}

/*
 *
 */
void CanbusThread()
{
	uint32_t next_send_tick = 20;

	if (next_send_tick <= HAL_GetTick())
	{
		next_send_tick += 20;

		uint8_t data[8] = {0};
		// output current
		data[0] = output_current;
		data[1] = output_current >> 8;
		// output voltage
		uint16_t total_voltage = (uint16_t)(stack_data.total_voltage_mv / 100);
		data[2] = total_voltage;
		data[3] = total_voltage >> 8;
		// temperature max
		int16_t temp_can = stack_data.temperature_max / 5;
		if (temp_can > 127) temp_can = 127;
		data[4] = temp_can;
		// state of charge
		uint8_t soc_uint = (uint8_t)(stack_soc.get_SoC() * 100);
		data[5] = soc_uint;
		// status
		data[6] = 0;
		// errors
		data[7] = 0;

		CAN_TxHeaderTypeDef tx_header;
		uint32_t mailbox;
		tx_header.DLC = 8;
		tx_header.RTR = CAN_RTR_DATA;
		tx_header.StdId = 0x0E;
		tx_header.IDE = CAN_ID_STD;

		HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &mailbox);
	}
}

/*
 *
 */
void SocInit()
{
    stack_soc.set_single_cell_equivalent_model(Batt_model_EPA642128HP);
    stack_soc.set_single_cell_ocv_polinomial(OCV_poly_EPA642128HP, EPA642128HP_ocv_length);
    stack_soc.set_battery_configuration(1, 2);
    stack_soc.set_time_sampling(1.0f / 20.0f); // 20Hz
    stack_soc.set_update_matrix();
    stack_soc.set_full_battery();

    // set from backup
    htim1.Instance->CNT = 0;
    test_time = 0;
    stack_soc.update_SoC_based_on_voltage(3.8000f);
    test_time =  htim1.Instance->CNT;
    ;
    htim1.Instance->CNT = 0;
    stack_soc.update(0.0, 3.00);
    stack_soc.update(0.10, 3.30);
    stack_soc.update(0.0, 3.00);
    stack_soc.update(0.10, 3.30);
    test_time = htim1.Instance->CNT;
    test_soc = stack_soc.get_SoC();
    ;
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
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_SDIO_SD_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_TIM8_Init();
  MX_FATFS_Init();
  MX_TIM1_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim1);

  if (InputRead(1)) CAN_Init_Charger();

  SdInitFile();

  AnalogInit(&hadc1, &htim8);

  SerialportOpen(&huart1);

  LtcInit(&hspi2, SPI2_CS1_GPIO_Port, SPI2_CS1_Pin);

  SocInit();

  // timer for main threading
  HAL_TIM_Base_Start_IT(&htim9);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int i = 0;
  while(1)
  {
	  HAL_Delay(200);
	  memset(&stack_data.discharge[0], 0, 27);
	  stack_data.discharge[i++] = 1;
	  i %= 27;
  }

  uint32_t next_tick = 1000;
  char test_string[1100] = {0};
  while (1)
  {

	  if (next_tick < HAL_GetTick() && 0)
	  {
		  next_tick += 200;

		  RTC_DateTypeDef rtc_date;
		  RTC_TimeTypeDef rtc_time;
		  HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
		  HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);

		  uint32_t subsec = ((rtc_time.SecondFraction-rtc_time.SubSeconds)*1000)/(rtc_time.SecondFraction+1);
		  int n = sprintf(test_string, "%02d:%02d:%02d.%03d\n", (int)rtc_time.Hours, (int)rtc_time.Minutes, (int)rtc_time.Seconds, (int)subsec);

		  sd_bytes_to_write = 0;
		  sd_buffer_tail = 0;
		  for (int i = 0; i < n; i++)
		  {
			  sd_buffer[i] = test_string[i];
			  sd_bytes_to_write++;
		  }

		  for ( ; sd_bytes_to_write < 999; sd_bytes_to_write++) sd_buffer[sd_bytes_to_write] = '-';
		  sd_buffer[sd_bytes_to_write++] = '\n';
	  }

	  SdSaveProcess();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_SDIO
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
  PeriphClkInitStruct.SdioClockSelection = RCC_SDIOCLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hcan1.Init.Prescaler = 9;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
  return;
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim1.Init.Prescaler = 179;
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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 179;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 179;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 999;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, AIR_PLUS_Pin|AIR_MINUS_Pin|AIR_PRE_Pin|AMS_FAULT_MCU_Pin
                          |SPI2_CS1_Pin|SPI2_CS2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : AIR_PLUS_Pin AIR_MINUS_Pin AIR_PRE_Pin AMS_FAULT_MCU_Pin
                           SPI2_CS1_Pin SPI2_CS2_Pin */
  GPIO_InitStruct.Pin = AIR_PLUS_Pin|AIR_MINUS_Pin|AIR_PRE_Pin|AMS_FAULT_MCU_Pin
                          |SPI2_CS1_Pin|SPI2_CS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SAFETY_DETECTION_Pin */
  GPIO_InitStruct.Pin = SAFETY_DETECTION_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SAFETY_DETECTION_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CARD_DETECT_Pin */
  GPIO_InitStruct.Pin = CARD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CARD_DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIN_1_Pin DIN_2_Pin DIN_3_Pin */
  GPIO_InitStruct.Pin = DIN_1_Pin|DIN_2_Pin|DIN_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM9)
	{
		MainInterruptCallback();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	SerialportRxCallback();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	SerialportTxCallback();
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	LtcTxCallback();
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	LtcTxCallback();
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
