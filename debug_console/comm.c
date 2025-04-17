/*
 * COMM.c
 *
 *  Created on: 28 Apr 2020
 *      Author: Butch
 */

#include "comm.h"
#include "debug_console.h"
#include "assert.h"
#include "stdlib.h"
//#include "cmsis_os.h"

#ifndef _CMSIS_OS_H
#define	osDelay(X)	HAL_Delay(X)
#endif


void CommUARTInit(UART_DATA_t *uart_data)
{
#ifndef	TX_BUFFER_SIZE
	assert(uart_data->TX_Buffer_size>0);
#endif
	assert(uart_data->COMM_rcv!=NULL);

	uint32_t result = HAL_UART_AbortReceive(uart_data->huart);

	do {
		uint32_t buffer;
		result = HAL_UART_Receive(uart_data->huart, (uint8_t*)&buffer, 1, 1);
	} while (result!=HAL_TIMEOUT);

	__HAL_UART_CLEAR_OREFLAG(uart_data->huart);
	__HAL_UART_CLEAR_NEFLAG(uart_data->huart);
	__HAL_UART_CLEAR_FEFLAG(uart_data->huart);

	uart_data->RX_DMA_Buffer_head = 0;
	uart_data->RX_DMA_Buffer_tail = 0;
	uart_data->TX_Buffer_head = 0;
	uart_data->TX_Buffer_tail = 0;
	uart_data->TX_DMA_Lock = 0;
	if (uart_data->huart->hdmarx) {
		HAL_UART_Receive_DMA(uart_data->huart,
				uart_data->RX_DMA_Buffer, RX_DMA_BUFFER_SIZE);
#if	_USE_RX_DMA_ISR_
		__HAL_UART_ENABLE_IT(uart_data->huart, UART_IT_IDLE);
#endif
	}
}

#if 0	//Replaced by ProcessReceive() under interrupts
char CommRXTask(UART_DATA_t *uart_data)
{
#if _USE_RX_POLLING_
	char _busy = 0;
    CLEAR_BIT(uart_data->huart->Instance->CR1,
    		(USART_CR1_RXNEIE_RXFNEIE | USART_CR1_PEIE | USART_CR1_TXEIE_TXFNFIE));
    CLEAR_BIT(uart_data->huart->Instance->CR3, USART_CR3_EIE);

	while (__HAL_UART_GET_FLAG(uart_data->huart, UART_FLAG_RXNE))
	{
		uint8_t _rxchar = (uint8_t)(uart_data->huart->Instance->RDR);
		uart_data->COMM_rcv(_rxchar);
		_busy = 1;
	}
	return _busy;
#else
	uint16_t pos = __HAL_DMA_GET_COUNTER(uart_data->huart->hdmarx);
	if (pos==0) pos = RX_DMA_BUFFER_SIZE;
	pos = RX_DMA_BUFFER_SIZE - pos;
	if (pos != uart_data->RX_DMA_Buffer_head) {
		uart_data->RX_DMA_Buffer_head = pos;
		do {
			char _rxchar = uart_data->RX_DMA_Buffer[uart_data->RX_DMA_Buffer_tail++];
			if (uart_data->RX_DMA_Buffer_tail >= RX_DMA_BUFFER_SIZE)
				uart_data->RX_DMA_Buffer_tail = 0;
			uart_data->COMM_rcv(_rxchar);
		} while (uart_data->RX_DMA_Buffer_head != uart_data->RX_DMA_Buffer_tail);

		return 1;
	}
	return 0;
#endif
}
#endif

char CommTXTask(UART_DATA_t *uart_data)
{
	if (uart_data->TX_DMA_Lock) return 1;
	if (uart_data->TX_Buffer_head!=uart_data->TX_Buffer_tail)
	{
		uart_data->TX_DMA_Lock = 1;
		int16_t DMA_tx_buffer_len;
		if (uart_data->TX_Buffer_head > uart_data->TX_Buffer_tail) {
			DMA_tx_buffer_len = uart_data->TX_Buffer_head - uart_data->TX_Buffer_tail;
		} else {
#ifdef	TX_BUFFER_SIZE
			DMA_tx_buffer_len = TX_BUFFER_SIZE - uart_data->TX_Buffer_tail;
#else
			DMA_tx_buffer_len = uart_data->TX_Buffer_size - uart_data->TX_Buffer_tail;
#endif
			DMA_tx_buffer_len += uart_data->TX_Buffer_head;
		}
		if (DMA_tx_buffer_len>TX_DMA_BUFFER_SIZE) DMA_tx_buffer_len = TX_DMA_BUFFER_SIZE;
		for (uint16_t _i = 0; _i<DMA_tx_buffer_len; _i++) {
			uart_data->TX_DMA_Buffer[_i] = uart_data->TX_Buffer[uart_data->TX_Buffer_tail++];
#ifdef	TX_BUFFER_SIZE
			if (uart_data->TX_Buffer_tail >= TX_BUFFER_SIZE) uart_data->TX_Buffer_tail = 0;
#else
			if (uart_data->TX_Buffer_tail >= uart_data->TX_Buffer_size) uart_data->TX_Buffer_tail = 0;
#endif
		}

		UART_HandleTypeDef* huart = uart_data->huart;

		if (huart->hdmatx!=NULL)
		{
			if (HAL_UART_Transmit_DMA(huart, uart_data->TX_DMA_Buffer, DMA_tx_buffer_len)==HAL_OK)
			{
				return 1;
			}
		}
		else
		{
			if (HAL_UART_Transmit_IT(huart, uart_data->TX_DMA_Buffer, DMA_tx_buffer_len)==HAL_OK)
			{
				return 1;
			}
		}
		uart_data->TX_DMA_Lock = 0;
	}
	return 0;
}

void CommUARTPutChar(UART_DATA_t *uart_data, char ch)
{
	uint16_t _tx_head = uart_data->TX_Buffer_head;
	uart_data->TX_Buffer[_tx_head++] = ch;
#ifdef	TX_BUFFER_SIZE
	if (_tx_head>=TX_BUFFER_SIZE) _tx_head = 0;
#else
	if (_tx_head>=uart_data->TX_Buffer_size) _tx_head = 0;
#endif
	if (_tx_head==uart_data->TX_Buffer_tail) {
		while (uart_data->TX_DMA_Lock) osDelay(1);
		CommTXTask(uart_data);
	}
	uart_data->TX_Buffer_head = _tx_head;
}


#if 0
void CommUARTSend(UART_DATA_t *uart_data, unsigned char *message, int len)
/*
	send a string of characters to the BT port
*/
{
	for (int i=0; i<len ; i++)
	{
		unsigned char dat = message[i];
		CommUARTPutChar(uart_data, dat);
	}
}
#endif

void comm_task(void)
{
}

void enable_comm(void)
{
}

void remove_comm(void)
{

}

void comm_send(char *message)
{

}

#ifdef COMM3_huart
void COMM3_rcv(uint8_t dat);
__attribute__((section(".DMABufferSection"))) __attribute__((aligned(32)))
uint8_t RX_DMA_Buffer3[RX_DMA_BUFFER_SIZE];
__attribute__((section(".DMABufferSection"))) __attribute__((aligned(32)))
uint8_t TX_DMA_Buffer3[TX_DMA_BUFFER_SIZE];

UART_DATA_t uart3_data = {
	.huart = &COMM3_huart,
	.COMM_rcv = COMM3_rcv,
	.RX_DMA_Buffer = RX_DMA_Buffer3,
	.TX_DMA_Buffer = TX_DMA_Buffer3,
};
#endif

#ifdef COMM5_huart
void COMM5_rcv(uint8_t dat);
__attribute__((section(".DMABufferSection"))) __attribute__((aligned(32)))
uint8_t RX_DMA_Buffer5[RX_DMA_BUFFER_SIZE];
__attribute__((section(".DMABufferSection"))) __attribute__((aligned(32)))
uint8_t TX_DMA_Buffer5[TX_DMA_BUFFER_SIZE];

UART_DATA_t uart5_data = {
	.huart = &COMM5_huart,
	.COMM_rcv = COMM5_rcv,
	.RX_DMA_Buffer = RX_DMA_Buffer5,
	.TX_DMA_Buffer = TX_DMA_Buffer5,
};
#endif

#ifdef COMM6_huart
void COMM6_rcv(uint8_t dat);
__attribute__((section(".DMABufferSection"))) __attribute__((aligned(32)))
uint8_t RX_DMA_Buffer6[RX_DMA_BUFFER_SIZE];
__attribute__((section(".DMABufferSection"))) __attribute__((aligned(32)))
uint8_t TX_DMA_Buffer6[TX_DMA_BUFFER_SIZE];

UART_DATA_t uart6_data = {
	.huart = &COMM6_huart,
	.COMM_rcv = COMM6_rcv,
	.RX_DMA_Buffer = RX_DMA_Buffer6,
	.TX_DMA_Buffer = TX_DMA_Buffer6,
};
#endif

#ifdef COMM8_huart
void COMM8_rcv(uint8_t dat);
__attribute__((section(".DMABufferSection"))) __attribute__((aligned(32)))
uint8_t RX_DMA_Buffer8[RX_DMA_BUFFER_SIZE];
__attribute__((section(".DMABufferSection"))) __attribute__((aligned(32)))
uint8_t TX_DMA_Buffer8[TX_DMA_BUFFER_SIZE];

UART_DATA_t uart8_data = {
	.huart = &COMM8_huart,
	.COMM_rcv = COMM8_rcv,
  	.RX_DMA_Buffer = RX_DMA_Buffer8,
	.TX_DMA_Buffer = TX_DMA_Buffer8,
};
#endif

#ifdef huart_debug
extern UART_DATA_t debug_data;
#endif

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
#ifdef uart3_data
	if (huart==uart3_data.huart)
	{
		uart3_data.TX_DMA_Lock = 0;
		return;
	}
#endif
#ifdef uart5_data
	if (huart==uart5_data.huart)
	{
		uart5_data.TX_DMA_Lock = 0;
		return;
	}
#endif
#ifdef uart6_data
	if (huart==uart6_data.huart)
	{
		uart6_data.TX_DMA_Lock = 0;
		return;
	}
#endif
#ifdef uart8_data
	if (huart==uart8_data.huart)
	{
		uart8_data.TX_DMA_Lock = 0;
		return;
	}
#endif
#ifdef huart_debug
	if (huart==debug_data.huart)
	{
		debug_data.TX_DMA_Lock = 0;
		return;
	}
//	if (huart==&huart_debug)
//	{
//		Debug_TxCpltCallback();
//		return;
//	}
#endif
}

void Process_ErrorCallback(UART_DATA_t* uart_data, uint32_t error)
{
	uart_data->ErrorCode = error;
	if (error&(HAL_UART_ERROR_FE|HAL_UART_ERROR_NE|HAL_UART_ERROR_ORE))
	{
		uart_data->RX_DMA_Buffer_head = 0;
		uart_data->RX_DMA_Buffer_tail = 0;

		HAL_UART_Receive_DMA(uart_data->huart, uart_data->RX_DMA_Buffer, RX_DMA_BUFFER_SIZE);
	}
	if (error&HAL_UART_ERROR_DMA)
	{
		uart_data->TX_DMA_Lock = 0;
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	uint32_t error = huart->ErrorCode;
	huart->ErrorCode = 0;

#ifdef COMM3_huart
	if (huart==uart3_data.huart)
	{
		DebugPrint("\r\n UART3 Error = %8lX", error);
		Process_ErrorCallback(&uart3_data, error);
		return;
	}
#endif
#ifdef COMM5_huart
	if (huart==uart5_data.huart)
	{
		DebugPrint("\r\n UART5 Error = %8lX", error);
		Process_ErrorCallback(&uart5_data, error);
		return;
	}
#endif
#ifdef COMM6_huart
	if (huart==uart6_data.huart)
	{
		DebugPrint("\r\n UART6 Error = %8lX", error);
		Process_ErrorCallback(&uart6_data, error);
		return;
	}
#endif
#ifdef COMM8_huart
	if (huart==uart8_data.huart)
	{
		DebugPrint("\r\n UART8 Error = %8lX", error);
		Process_ErrorCallback(&uart8_data, error);
		return;
	}
#endif
#ifdef huart_debug
	if (huart==debug_data.huart)
	{
		DebugPrint("\r\n Debug UART Error = %8lX", error);
		Process_ErrorCallback(&debug_data, error);
		return;
	}
#endif
}

void ProcessReceive(UART_DATA_t* uart_data)
{
	uint16_t pos = RX_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(uart_data->huart->hdmarx);
	if (pos != uart_data->RX_DMA_Buffer_head)
	{
		uart_data->RX_DMA_Buffer_head = pos;
		if (pos>uart_data->RX_DMA_Buffer_tail)
		{
			for (uint16_t count = pos - uart_data->RX_DMA_Buffer_tail; count>0; count--)
			{
				uint8_t _rxchar = uart_data->RX_DMA_Buffer[uart_data->RX_DMA_Buffer_tail++];
				uart_data->COMM_rcv(_rxchar);
			}
		}
		else
		{
			for (uint16_t count = RX_DMA_BUFFER_SIZE - uart_data->RX_DMA_Buffer_tail; count>0; count--)
			{
				uint8_t _rxchar = uart_data->RX_DMA_Buffer[uart_data->RX_DMA_Buffer_tail++];
				uart_data->COMM_rcv(_rxchar);
			}
			uart_data->RX_DMA_Buffer_tail = 0;
			if (pos>0)
			{
				for (uint16_t count = pos; count>0; count--)
				{
					uint8_t _rxchar = uart_data->RX_DMA_Buffer[uart_data->RX_DMA_Buffer_tail++];
					uart_data->COMM_rcv(_rxchar);
				}
			}
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#ifdef COMM3_huart
	if (huart==uart3_data.huart)
	{
		ProcessReceive(&uart3_data);
		return;
	}
#endif
#ifdef COMM5_huart
	if (huart==uart5_data.huart)
	{
		ProcessReceive(&uart5_data);
		return;
	}
#endif
#ifdef COMM6_huart
	if (huart==uart6_data.huart)
	{
		ProcessReceive(&uart6_data);
		return;
	}
#endif
#ifdef COMM8_huart
	if (huart==uart8_data.huart)
	{
		ProcessReceive(&uart8_data);
		return;
	}
#endif
#ifdef huart_debug
	if (huart==debug_data.huart)
	{
		ProcessReceive(&debug_data);
		return;
	}
#endif
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_RxCpltCallback(huart);
}
