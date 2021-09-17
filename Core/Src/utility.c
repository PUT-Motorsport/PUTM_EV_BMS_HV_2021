/*
 * utility.c
 *
 *  Created on: 6 wrz 2021
 *      Author: Tomek
 */

#include "utility.h"
#include "main.h"


/*
 * brief:	Sets output for selected LED
 * param:	led_no	selected led 1,2,3
 * param:	state	0 - off, 1 - on
 * retval:	none
 */
void LedSet(uint8_t led_no, uint8_t state)
{
	if (led_no == 1) HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, (GPIO_PinState)state);
	else if (led_no == 2) HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, (GPIO_PinState)state);
	else if (led_no == 3) HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, (GPIO_PinState)state);
}

/*
 * brief:	Reads input state
 * param:	input_no	selected input 1,2,3
 * retval:	0 - input low, 1 - input high
 */
uint8_t InputRead(uint8_t input_no)
{
	uint8_t retval = 1;
	if (input_no == 1) retval = HAL_GPIO_ReadPin(DIN_1_GPIO_Port, DIN_1_Pin);
	else if (input_no == 2) retval = HAL_GPIO_ReadPin(DIN_2_GPIO_Port, DIN_2_Pin);
	else if (input_no == 3) retval = HAL_GPIO_ReadPin(DIN_3_GPIO_Port, DIN_3_Pin);

	if (retval == 0) retval = 1;
	else retval = 0;

	return retval;
}

/*
 * brief:	Reads presence of satety voltage at AIR
 * param:	none
 * retval:	0 - not present, 1 - present
 */
uint8_t SafetyStateRead()
{
	uint8_t retval = 0;
	if (HAL_GPIO_ReadPin(SAFETY_DETECTION_GPIO_Port, SAFETY_DETECTION_Pin) == 0) retval = 1;

	return retval;
}

/*
 * brief:	Sets AMS fault output
 * param:	state	0 - low, 1 - high
 * retval:	none
 */
void FaultOutputSet(uint8_t state)
{
	HAL_GPIO_WritePin(AMS_FAULT_MCU_GPIO_Port, AMS_FAULT_MCU_Pin, (GPIO_PinState)state);
}

/*
 * brief:	Set state of AIR MINUS relay
 * param:	state	0 - turn off, 1 - turn on
 * retval:	none
 */
void AIR_MINUS_Set(uint8_t state)
{
	HAL_GPIO_WritePin(AIR_MINUS_GPIO_Port, AIR_MINUS_Pin, (GPIO_PinState)state);
}

/*
 * brief:	Set state of AIR PLUS relay
 * param:	state	0 - turn off, 1 - turn on
 * retval:	none
 */
void AIR_PLUS_Set(uint8_t state)
{
	HAL_GPIO_WritePin(AIR_PLUS_GPIO_Port, AIR_PLUS_Pin, (GPIO_PinState)state);
}

/*
 * brief:	Set state of AIR PRECHARGE relay
 * param:	state	0 - turn off, 1 - turn on
 * retval:	none
 */
void AIR_PRECHARGE_Set(uint8_t state)
{
	HAL_GPIO_WritePin(AIR_PRE_GPIO_Port, AIR_PRE_Pin, (GPIO_PinState)state);
}
