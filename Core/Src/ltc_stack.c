#include "main.h"
#include "ltc_stack.h"

#ifdef __cplusplus
extern "C" {
#endif

struct stack_data_type stack_data;

static SPI_HandleTypeDef *hspi;
static GPIO_TypeDef *cs_gpio;
static uint16_t cs_pin;

static uint8_t tx_busy;
static uint8_t comm_state;
static uint32_t comm_next_tick;
static uint32_t comm_next_refresh_tick;

static uint8_t balance_flags[9 * LTCS_IN_STACK];

static uint16_t pec15Table[256];
static uint16_t CRC15_POLY = 0x4599;

static uint8_t tx_buffer[4 + 8 * LTCS_IN_STACK];
static uint8_t rx_buffer[4 + 8 * LTCS_IN_STACK];

// ltc reading to temperature
static const int16_t temperature_map[29][2] = {
		//		ltc value ,  temperature *C
		{27190	,	-20} ,
		{26380	,	-15} ,
		{25400	,	-10} ,
		{24260	,	-5}	,
		{22960	,	0}	,
		{21520	,	5}	,
		{19960	,	10}	,
		{18330	,	15}	,
		{16660	,	20}	,
		{15000	,	25}	,
		{13380	,	30}	,
		{11850	,	35}	,
		{10420	,	40}	,
		{9120	,	45}	,
		{7940	,	50}	,
		{6890	,	55}	,
		{5970	,	60}	,
		{5170	,	65}	,
		{4470	,	70}	,
		{3860	,	75}	,
		{3350	,	80}	,
		{2900	,	85}	,
		{2520	,	90}	,
		{2190	,	95}	,
		{1910	,	100} ,
		{1660	,	105} ,
		{1450	,	110} ,
		{1270	,	115}
};

/*
 *
 */
void LtcInitPec15Table()
{
	uint16_t remainder;
	for (int i = 0; i < 256; i++)
	{
		remainder = i << 7;
		for (int bit = 8; bit > 0; --bit)
		{
			if (remainder & 0x4000)
			{
				remainder = ((remainder << 1));
				remainder = (remainder ^ CRC15_POLY);
			}
			else
			{
				remainder = ((remainder << 1));
			}
		}
		pec15Table[i] = remainder&0xFFFF;
	}
}

/*
 *
 */
uint16_t LtcPec15(uint8_t *data , int len)
{
	uint16_t remainder,address;
	remainder = 16;//PEC seed
	for (int i = 0; i < len; i++)
	{
		address = ((remainder >> 7) ^ data[i]) & 0xff;//calculate PEC table address
		remainder = (remainder << 8 ) ^ pec15Table[address];
	}
	return (remainder*2);//The CRC15 has a 0 in the LSB so the final value must be multiplied by 2
}

/*
 *
 */
void LtcInit(SPI_HandleTypeDef *ltc_hspi, GPIO_TypeDef *ltc_cs_gpio, uint16_t ltc_cs_pin)
{
	hspi = ltc_hspi;
	cs_gpio = ltc_cs_gpio;
	cs_pin = ltc_cs_pin;
	stack_data.minimum_cell_no = -1;

	LtcInitPec15Table();

	LtcWakeupSleep();
}

/**
 * Brief:	Sends wakeup 1 dummy byte, 4 times every 1ms; wakeup all ltcs from SLEEP state, use in init
 * Param:	None
 * Retval:	None
 */
void LtcWakeupSleep()
{
	uint8_t tab[2] = {0xff};

	for (int i = 0; i < 8; i++)
	{
		LtcCsPinSet(0);
		HAL_SPI_Transmit(hspi, tab, 2, 1);
		LtcCsPinSet(1);
		HAL_Delay(1);
	}
}

/*
 *
 */
void LtcWakeupIdle()
{
	tx_buffer[0] = 0x00;
	LtcCsPinSet(0);
	tx_busy = 1;
	HAL_SPI_Transmit_IT(hspi, tx_buffer, 1);
}

/*
 *
 */
void LtcTxCallback()
{
	LtcCsPinSet(1);
	tx_busy = 0;
}

/*
 *
 */
void LtcCsPinSet(uint8_t pin_state)
{
	HAL_GPIO_WritePin(cs_gpio, cs_pin, pin_state);
}

/*
 *
 */
void LtcSendConfig(uint8_t discharge_activation)
{
	uint16_t pec;
	uint8_t cfg_buf[8] = {0};
	cfg_buf[0] = (LTC_CFG_GPIO << 3) | (LTC_CFG_REFON << 2) | LTC_CFG_ADCOPT;

	tx_buffer[0] = 0;
	tx_buffer[1] = 1;
	pec = LtcPec15(&tx_buffer[0], 2);
	tx_buffer[2] = pec >> 8;
	tx_buffer[3] = pec;

	for (int dev = 0; dev < LTCS_IN_STACK; dev++)
	{
		uint16_t disch_cfg = 0;
		if (discharge_activation == 1)
		{
			disch_cfg |= (uint16_t)(stack_data.discharge[0 + (LTCS_IN_STACK-dev-1)*9]) << 0;
			disch_cfg |= (uint16_t)(stack_data.discharge[1 + (LTCS_IN_STACK-dev-1)*9]) << 1;
			disch_cfg |= (uint16_t)(stack_data.discharge[2 + (LTCS_IN_STACK-dev-1)*9]) << 2;
			disch_cfg |= (uint16_t)(stack_data.discharge[3 + (LTCS_IN_STACK-dev-1)*9]) << 3;
			disch_cfg |= (uint16_t)(stack_data.discharge[4 + (LTCS_IN_STACK-dev-1)*9]) << 4;
			disch_cfg |= (uint16_t)(stack_data.discharge[5 + (LTCS_IN_STACK-dev-1)*9]) << 6;
			disch_cfg |= (uint16_t)(stack_data.discharge[6 + (LTCS_IN_STACK-dev-1)*9]) << 7;
			disch_cfg |= (uint16_t)(stack_data.discharge[7 + (LTCS_IN_STACK-dev-1)*9]) << 8;
			disch_cfg |= (uint16_t)(stack_data.discharge[8 + (LTCS_IN_STACK-dev-1)*9]) << 9;
			cfg_buf[4] = disch_cfg;
			cfg_buf[5] = (disch_cfg >> 8) & 0x0f;
		}
		// update discharge pin here
		pec = LtcPec15(&cfg_buf[0], 6);
		cfg_buf[6] = pec >> 8;
		cfg_buf[7] = pec;
		//
		for (int b = 0; b < 8; b++)
			tx_buffer[4 + dev*8 + b] = cfg_buf[b];
	}

	LtcCsPinSet(0);
	tx_busy = 1;
	HAL_SPI_Transmit_IT(hspi, tx_buffer, 4 + LTCS_IN_STACK*8);
}

/*
 *
 */
void LtcStartAdcCell()
{
	uint16_t pec;
	uint16_t cmd = 0b01001100000 | (LTC_ADC_MD << 7) | (LTC_ADC_DCP << 4);
	tx_buffer[0] = cmd >> 8;
	tx_buffer[1] = cmd;
	pec = LtcPec15(&tx_buffer[0], 2);
	tx_buffer[2] = pec >> 8;
	tx_buffer[3] = pec;

	LtcCsPinSet(0);
	tx_busy = 1;
	HAL_SPI_Transmit_IT(hspi, tx_buffer, 4);
}

/*
 *
 */
void LtcStartAdcGpio()
{
	uint16_t pec;
	uint16_t cmd = 0b10001100000 | (LTC_ADC_MD << 7);

	tx_buffer[0] = cmd >> 8;
	tx_buffer[1] = cmd;
	pec = LtcPec15(&tx_buffer[0], 2);
	tx_buffer[2] = pec >> 8;
	tx_buffer[3] = pec;

	LtcCsPinSet(0);
	tx_busy = 1;
	HAL_SPI_Transmit_IT(hspi, tx_buffer, 4);
}

/*
 *
 */
void LtcStartReadCell(uint8_t group)
{
	uint16_t pec;
	uint16_t cmd = 0;
	if (group == 0) cmd = 0b100;
	else if (group == 1) cmd = 0b110;
	else if (group == 2) cmd = 0b1000;
	else if (group == 3) cmd = 0b1010;

	tx_buffer[0] = cmd >> 8;
	tx_buffer[1] = cmd;
	pec = LtcPec15(&tx_buffer[0], 2);
	tx_buffer[2] = pec >> 8;
	tx_buffer[3] = pec;

	LtcCsPinSet(0);
	tx_busy = 1;
	HAL_SPI_TransmitReceive_IT(hspi, tx_buffer, rx_buffer, 4 + LTCS_IN_STACK*8);
}

/*
 *
 */
void LtcStartReadGpio(uint8_t group)
{
	uint16_t pec;
	uint16_t cmd;
	if (group == 0) cmd = 0b1100;
	else if (group == 1) cmd = 0b1110;

	tx_buffer[0] = cmd >> 8;
	tx_buffer[1] = cmd;
	pec = LtcPec15(&tx_buffer[0], 2);
	tx_buffer[2] = pec >> 8;
	tx_buffer[3] = pec;

	LtcCsPinSet(0);
	tx_busy = 1;
	HAL_SPI_TransmitReceive_IT(hspi, tx_buffer, rx_buffer, 4 + LTCS_IN_STACK*8);
}

/*
 *
 */
void LtcReadCell(uint8_t group)
{
	for (int dev = 0; dev < LTCS_IN_STACK; dev++)
	{
		uint16_t cell_voltage[3] = {0};
		// first test pec
		uint16_t pec = LtcPec15(&rx_buffer[4 + dev*8], 6);
		uint16_t pec_read = ((uint16_t)rx_buffer[10 + dev*8] << 8) | (uint16_t)rx_buffer[11 + dev*8];
		if (pec == pec_read)
		{
			cell_voltage[0] = (((uint16_t)rx_buffer[5 + dev*8] << 8) | (uint16_t)rx_buffer[4 + dev*8])/10;
			cell_voltage[1] = (((uint16_t)rx_buffer[7 + dev*8] << 8) | (uint16_t)rx_buffer[6 + dev*8])/10;
			cell_voltage[2] = (((uint16_t)rx_buffer[9 + dev*8] << 8) | (uint16_t)rx_buffer[8 + dev*8])/10;
		}
		// write to adequate place in a table
		if (group == 0) // 1,2,3
		{
			stack_data.voltages[0 + dev*9] = cell_voltage[0];
			stack_data.voltages[1 + dev*9] = cell_voltage[1];
			stack_data.voltages[2 + dev*9] = cell_voltage[2];
		}
		else if (group == 1) // 4,5
		{
			stack_data.voltages[3 + dev*9] = cell_voltage[0];
			stack_data.voltages[4 + dev*9] = cell_voltage[1];
		}
		else if (group == 2) // 6, 7, 8
		{
			stack_data.voltages[5 + dev*9] = cell_voltage[0];
			stack_data.voltages[6 + dev*9] = cell_voltage[1];
			stack_data.voltages[7 + dev*9] = cell_voltage[2];
		}
		else if (group == 3) // 9
		{
			stack_data.voltages[8 + dev*9] = cell_voltage[0];
		}
	}
}

/*
 *
 */
void LtcReadGpio(uint8_t group)
{
	for (int dev = 0; dev < LTCS_IN_STACK; dev++)
	{
		uint16_t gpio_voltage[3] = {0};
		// first test pec
		uint16_t pec = LtcPec15(&rx_buffer[4 + dev*8], 6);
		uint16_t pec_read = ((uint16_t)rx_buffer[10 + dev*8] << 8) | (uint16_t)rx_buffer[11 + dev*8];
		if (pec == pec_read)
		{
			gpio_voltage[0] = ((uint16_t)rx_buffer[5 + dev*8] << 8) | (uint16_t)rx_buffer[4 + dev*8];
			gpio_voltage[1] = ((uint16_t)rx_buffer[7 + dev*8] << 8) | (uint16_t)rx_buffer[6 + dev*8];
			gpio_voltage[2] = ((uint16_t)rx_buffer[9 + dev*8] << 8) | (uint16_t)rx_buffer[8 + dev*8];
		}
		// write to adequate place in a table
		if (group == 0) // 1,2,3
		{
			stack_data.temperatures[0 + dev*3] = LtcCalculateTemperature(gpio_voltage[0]);
			stack_data.temperatures[1 + dev*3] = LtcCalculateTemperature(gpio_voltage[1]);
			stack_data.temperatures[2 + dev*3] = LtcCalculateTemperature(gpio_voltage[2]);
		}
		//		else if (group == 1) // 4,5, 2nd ref
		//		{
		//
		//		}
	}
}

/*
 * Brief:	Practical connection of the stack is {1,3,2},{4,6,5},...
 */
void LtcStackDataReorder()
{
	// switch 2 and 3, 5 and 6, 8 and 9, 11 and 12, 14 and 15
	for (int dev = 1; dev < LTCS_IN_STACK; dev += 3)
	{
		uint16_t temp_v;
		for (int cv = 0; cv < 9; cv++)
		{
			temp_v = stack_data.voltages[dev*9 + cv];
			stack_data.voltages[dev*9 + cv] = stack_data.voltages[(dev+1)*9 + cv];
			stack_data.voltages[(dev+1)*9 + cv] = temp_v;
		}
		int16_t temp_t;
		for (int ct = 0; ct < 3; ct++)
		{
			temp_t = stack_data.temperatures[dev*3 + ct];
			stack_data.temperatures[dev*3 + ct] = stack_data.temperatures[(dev+1)*3 + ct];
			stack_data.temperatures[(dev+1)*3 + ct] = temp_t;
		}
	}
}

/*
 *
 */
void LtcCommunicationThread()
{
	if (tx_busy == 1) return;

	if (comm_state == 0)
	{
		if (comm_next_refresh_tick <= HAL_GetTick())
		{
			comm_next_refresh_tick += LTC_REFRESH_PERIOD;
			LtcWakeupIdle();
			comm_state++;
		}
	}
	else if (comm_state == 1)
	{
		LtcSendConfig(0);
		comm_state++;
	}
	else if (comm_state == 2)
	{
		LtcStartAdcCell();
		comm_next_tick = HAL_GetTick() + LTC_ADC_DELAY;
		comm_state++;
	}
	else if (comm_state == 3)
	{
		if (comm_next_tick <= HAL_GetTick())
		{
			LtcWakeupIdle();
			comm_state++;
		}
	}
	else if (comm_state == 4)
	{
		LtcSendConfig(1);
		comm_state++;
	}
	else if (comm_state == 5)
	{
		LtcStartAdcGpio();
		comm_next_tick = HAL_GetTick() + LTC_ADC_DELAY;
		comm_state++;
	}
	else if (comm_state == 6)
	{
		if (comm_next_tick <= HAL_GetTick())
		{
			LtcWakeupIdle();
			comm_state++;
		}
	}
	else if (comm_state == 7)
	{
		LtcStartReadCell(0);
		comm_state++;
	}
	else if (comm_state == 8)
	{
		LtcReadCell(0);
		LtcStartReadCell(1);
		comm_state++;
	}
	else if (comm_state == 9)
	{
		LtcReadCell(1);
		LtcStartReadCell(2);
		comm_state++;
	}
	else if (comm_state == 10)
	{
		LtcReadCell(2);
		LtcStartReadCell(3);
		comm_state++;
	}
	else if (comm_state == 11)
	{
		LtcReadCell(3);
		LtcStartReadGpio(0);
		comm_state++;
	}
	else if (comm_state == 12)
	{
		LtcReadGpio(0);
		stack_data.data_refresh_tick = HAL_GetTick();
		LtcStackSummary();
		LtcErrorCheck();
		comm_state = 0;
	}
}

/**
 * Brief:	Calculation of temperature, from value of measured voltage
 * Param:	ltc_value:	Value of voltage, LSB -> 0.1 mV
 * Retval:	Temperature in 0.1*C, integer
 */
int16_t LtcCalculateTemperature(uint16_t gpio_voltage)
{
	if (gpio_voltage > INT16_MAX) gpio_voltage = INT16_MAX;
	float result = 0.0;
	for(int i = 1; i < 28; i++)
	{
		if(gpio_voltage >= temperature_map[i][0])
		{
			// approximation
			result = (float)(temperature_map[i][1])
							-
							5.0 * ((float)gpio_voltage-(float)(temperature_map[i][0]))
							/
							((float)(temperature_map[i-1][0]) - (float)(temperature_map[i][0]));
			break;
		}
	}

	return (int16_t)(result * 10);
}

/*
 *
 */
void LtcErrorCheck()
{
	for (int cv = 0; cv < 9*LTCS_IN_STACK; cv++)
	{
		if (stack_data.voltages[cv] > CELL_VOLTAGE_MAX || stack_data.voltages[cv] < CELL_VOLTAGE_MIN)
		{
			if (stack_data.voltages_err_cntr[cv] < CELL_VOLTAGE_ERR_CNTR_THRESHOLD) stack_data.voltages_err_cntr[cv] += 1;
			else stack_data.error |= CELL_ERROR_VOLTAGE_FLAG;
		}
		else if (stack_data.voltages_err_cntr[cv] > 0 && stack_data.voltages_err_cntr[cv] < CELL_VOLTAGE_ERR_CNTR_THRESHOLD)
			stack_data.voltages_err_cntr[cv] -= 1;
	}
	for (int ct = 0; ct < 3 * LTCS_IN_STACK; ct++)
	{
		if (stack_data.voltages[ct] > CELL_TEMPERATURE_MAX || stack_data.voltages[ct] < CELL_TEMPERATURE_MIN)
		{
			if (stack_data.temperatures_err_cntr[ct] < CELL_VOLTAGE_ERR_CNTR_THRESHOLD) stack_data.temperatures_err_cntr[ct] += 1;
			else stack_data.error |= CELL_ERROR_TEMPERATURE_FLAG;
		}
		else if (stack_data.temperatures_err_cntr[ct] > 0 && stack_data.temperatures_err_cntr[ct] < CELL_VOLTAGE_ERR_CNTR_THRESHOLD)
			stack_data.temperatures_err_cntr[ct] -= 1;
	}
}

/*
 *
 */
uint8_t LtcGetStackError()
{
	return stack_data.error;
}

/*
 *
 */
void LtcStackSummary()
{
	uint32_t voltage_sum = 0, minimum = UINT16_MAX, minimum_cell_no = -1;
	for (int cv = 0; cv < LTCS_IN_STACK*9; cv++)
	{
		voltage_sum += stack_data.voltages[cv];
		if (stack_data.voltages[cv] < minimum && stack_data.voltages[cv] != 0)
		{
			minimum = stack_data.voltages[cv];
			minimum_cell_no = cv;
		}
	}
	stack_data.total_voltage_mv = voltage_sum;
	stack_data.cell_minimum_voltage = minimum;
	if (stack_data.minimum_cell_no == -1) stack_data.minimum_cell_no = minimum_cell_no;
	int16_t temp_max = -1000;
	for (int ct = 0; ct < LTCS_IN_STACK*3; ct++)
	{
		if (stack_data.temperatures[ct] > temp_max) temp_max = stack_data.temperatures[ct];
	}
	stack_data.temperature_max = temp_max;
}

#ifdef __cplusplus
}
#endif
