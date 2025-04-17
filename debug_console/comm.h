/*
 * COMM.h
 *
 *  Created on: 28 Apr 2020
 *      Author: Butch
 */

#ifndef COMM_H_
#define COMM_H_

#include "usart.h"

#define	RX_DMA_BUFFER_SIZE	256
#define	TX_DMA_BUFFER_SIZE	256
#define	TX_BUFFER_SIZE    	1024

#define	_USE_D2_SRAM_	1
#define	_USE_RX_DMA_ISR_	1


typedef	struct {
	UART_HandleTypeDef* huart;
	void (*COMM_rcv)(uint8_t dat);
	volatile uint32_t ErrorCode;
	volatile int16_t RX_DMA_Buffer_head;
	volatile int16_t RX_DMA_Buffer_tail;
	//uint16_t RX_DMA_Buffer_size;
#if _USE_D2_SRAM_
	uint8_t *RX_DMA_Buffer;
#else
	uint8_t RX_DMA_Buffer[RX_DMA_BUFFER_SIZE];
#endif
	int16_t TX_Buffer_head;
	int16_t TX_Buffer_tail;
#ifndef	TX_BUFFER_SIZE
	int16_t TX_Buffer_size;
	uint8_t *TX_Buffer;
#else
	uint8_t TX_Buffer[TX_BUFFER_SIZE];
#endif
#if _USE_D2_SRAM_
	uint8_t *TX_DMA_Buffer;
#else
	uint8_t TX_DMA_Buffer[TX_DMA_BUFFER_SIZE];
#endif
	volatile uint8_t TX_DMA_Lock;
} UART_DATA_t;

extern UART_DATA_t uart8_data;	//RS232 communications
extern UART_DATA_t uart6_data;	//Bluetooth communications
//extern UART_DATA_t uart5_data;	//Debug communications
extern UART_DATA_t uart3_data;	//Front panel communications

void CommUARTInit(UART_DATA_t *uart_data);

char CommRXTask(UART_DATA_t *uart_data);

char CommTXTask(UART_DATA_t *uart_data);

void CommUARTPutChar(UART_DATA_t *uart_data, char ch);

void CommUARTSend(UART_DATA_t *uart_data, unsigned char *message, int len);

extern void comm_task(void);

extern void enable_comm(void);

extern void remove_comm(void);

extern void comm_send(char *message);

#endif /* COMM_H_ */
