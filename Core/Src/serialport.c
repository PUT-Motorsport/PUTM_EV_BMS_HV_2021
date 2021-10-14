/*
 * serialport.c
 *
 *  Created on: 6 lip 2021
 *      Author: Tomek
 */

#include "serialport.h"
#include "stdlib.h"




static struct serialport_type serialport;

void SerialportOpen(UART_HandleTypeDef *huart)
{
	serialport.huart = huart;

	HAL_UART_Receive_IT(serialport.huart, &serialport.rx_buffer[0], 1);
}

void SerialportRxCallback()
{
	serialport.rx_head++;
	serialport.rx_head %= SERIAL_RX_BUF_SIZE;

	if (serialport.rx_head == serialport.rx_tail)
	{
		serialport.rx_tail++;
		serialport.rx_tail %= SERIAL_RX_BUF_SIZE;
	}

	HAL_UART_Receive_IT(serialport.huart, &serialport.rx_buffer[serialport.rx_head], 1);
}

void SerialportTxCallback()
{
	serialport.tx_tail += serialport.tx_length;
	serialport.tx_tail %= SERIAL_TX_BUF_SIZE;

	if (serialport.tx_head != serialport.tx_tail)
	{
		if (serialport.tx_head > serialport.tx_tail)
			serialport.tx_length = serialport.tx_head - serialport.tx_tail;
		else
			serialport.tx_length = SERIAL_TX_BUF_SIZE - serialport.tx_tail;

		serialport.tx_busy = 1;
		HAL_UART_Transmit_IT(serialport.huart, &serialport.tx_buffer[serialport.tx_tail], serialport.tx_length);
	}
	else serialport.tx_busy = 0;
}

uint8_t SerialportWrite(uint8_t *data, uint16_t length)
{
	uint8_t retval = 0;
	uint16_t free_tx = 0;
	if (serialport.tx_head > serialport.tx_tail) free_tx = SERIAL_TX_BUF_SIZE - serialport.tx_head;
	else if (serialport.tx_head < serialport.tx_tail) free_tx = SERIAL_TX_BUF_SIZE - serialport.tx_tail - serialport.tx_head;
	else free_tx = SERIAL_TX_BUF_SIZE;

	if (free_tx > length)
	{
		for (int i = 0; i < length; i++)
		{
			serialport.tx_buffer[serialport.tx_head++] = data[i];
			serialport.tx_head %= SERIAL_TX_BUF_SIZE;
		}
	}
	else retval = 1;

	if (serialport.tx_busy == 0)
	{
		if (serialport.tx_head > serialport.tx_tail)
			serialport.tx_length = serialport.tx_head - serialport.tx_tail;
		else
			serialport.tx_length = SERIAL_TX_BUF_SIZE - serialport.tx_tail;

		serialport.tx_busy = 1;
		HAL_UART_Transmit_IT(serialport.huart, &(serialport.tx_buffer[serialport.tx_tail]), serialport.tx_length);
	}

	return retval;
}

char* SerialportReadLine(uint16_t *length)
{
	char *retline = NULL;
	*length = 0;
	if (serialport.rx_tail != serialport.rx_head)
	{
		uint16_t search_head = serialport.rx_tail, len = 0;
		while (search_head != serialport.rx_head)
		{
			len++;
			if (serialport.rx_buffer[search_head] == '\n')
			{
				retline = malloc(len+1);
				memset(retline, 0, len+1);
				if (retline != NULL)
				{
					for(int i = 0; i < len; i++)
					{
						retline[i] = serialport.rx_buffer[serialport.rx_tail++];
						serialport.rx_tail %= SERIAL_RX_BUF_SIZE;
					}
					*length = len;
				}
				break;
			}

			search_head++;
			search_head %= SERIAL_RX_BUF_SIZE;
		}
	}

	return retline;
}



