#ifndef SERIALPORT_H
#define SERIALPORT_H

#include "stm32f4xx_hal.h"

#define SERIAL_RX_BUF_SIZE	50
#define SERIAL_TX_BUF_SIZE 2000

struct serialport_type{
	UART_HandleTypeDef *huart;

	uint8_t rx_buffer[SERIAL_RX_BUF_SIZE];
	uint16_t rx_head;
	uint16_t rx_tail;

	uint8_t tx_buffer[SERIAL_TX_BUF_SIZE];
	uint16_t tx_head;
	uint16_t tx_tail;

	uint8_t tx_busy;
	uint16_t tx_length;
};

void SerialportOpen(UART_HandleTypeDef *huart);
void SerialportRxCallback();
void SerialportTxCallback();
uint8_t SerialportWrite(uint8_t *data, uint16_t length);
uint8_t* SerialportReadLine(uint16_t *length);

#endif
