/*----------------------------------------------------------*/
/*  Filename :  DEBUG_CONSOLE.C     Created    : 2014-12-23 */
/*  Released :                      Revision # :            */
/*  Author   :  Butch Tabios                                */
/*                                                          */
/*  Routine List:                                           */
/*                                                          */
/*  Update List:                                            */
/*                                                          */
/*  Files needed for linking:                               */
/*                                                          */
/*  Comments:                                               */
/*    1. Debug console for the AVR32 mini board.            */
/*                                                          */
/*----------------------------------------------------------*/

#include "main.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include "debug_console.h"
//#include "Timer.h"
//#include "usbd_cdc_if.h"
#ifndef __USBD_CDC_IF_H__
#include "comm.h"
#endif
//#include "adc.h"
//#include "cmsis_os.h"

#ifndef EOF
#define EOF (-1)            /* End of file indicator */
#endif

#ifndef BS
#define	BS       	0x08
#endif
#ifndef LF
#define	LF       	0x0a
#endif
#ifndef CR
#define	CR       	0x0d
#endif

#ifndef	true
#define	true	1
#endif

#ifndef	false
#define	false	0
#endif

#ifndef DEBUGTIMER
#define DEBUGTIMER	DEBUG_TIMER
#endif

#define	SendDebugPrompt	DebugSend("\r\n->\0")
#define	SendError		DebugSend("\r\n Error???\0")

typedef uint8_t 	BYTE;
typedef uint16_t	WORD;

uint32_t _debug_dump_beg = 0;
uint32_t _old_debug_dump_beg = 0;
uint32_t _debug_dump_end = 0;
uint8_t *debug_port = (unsigned char *)0x80000000;

char bTest = 0;

#define	DBG_RX_BUFFER_SIZE   	256
#define	DBG_TX_BUFFER_SIZE  	2048
#define	DBG_DMA_BUFFER_SIZE 	2048

#ifdef	DBG_RX_BUFFER_SIZE
uint8_t dbg_rx_buffer[DBG_RX_BUFFER_SIZE];
int16_t dbg_rx_head=0, dbg_rx_tail=0;
#endif

#ifdef	DBG_TX_BUFFER_SIZE
uint8_t dbg_tx_buffer[DBG_TX_BUFFER_SIZE];
int16_t dbg_tx_head=0, dbg_tx_tail=0;
#endif

#ifdef	DBG_DMA_BUFFER_SIZE
uint8_t dbg_dma_buffer[DBG_DMA_BUFFER_SIZE];
#endif

uint8_t SPI_TxData[5] = {0};
uint8_t SPI_RxData[5] = {0};

int __io_putchar(int ch);
void __io_serial(void);

void debug_idle(void);
void debug_parse(char *cmd_line);
void debug_rcv(uint8_t ch);
unsigned int do_dump(void);

void DebugPutChar(char ch);
void DebugSend(char *message);

#ifdef huart_debug
uint8_t dbg_RX_DMA_Buffer[RX_DMA_BUFFER_SIZE] DMA_BUFFER_SECTION;
uint8_t dbg_TX_DMA_Buffer[TX_DMA_BUFFER_SIZE] DMA_BUFFER_SECTION;

UART_DATA_t debug_data = {
	.huart = &huart_debug,
	.COMM_rcv = debug_rcv,
	.RX_DMA_Buffer = dbg_RX_DMA_Buffer,
	.TX_DMA_Buffer = dbg_TX_DMA_Buffer,
};
#endif

unsigned int do_dump(void)
{
	int i1;
	unsigned char *pByte_Val, byte_val;
	char ascii_result[18];
	_old_debug_dump_beg = _debug_dump_beg;

	DebugPrint("\r\n %08x - ",_debug_dump_beg & 0xfffffff0);
	strcpy(ascii_result,"................");
	for (i1=0;i1<(_debug_dump_beg & 0x000f);i1++) DebugSend("   ");
	while (_debug_dump_beg <= _debug_dump_end)
	{
		pByte_Val = (unsigned char *)_debug_dump_beg++;
		byte_val = *pByte_Val;
		DebugPrint("%02x ",byte_val);
		if (!iscntrl(byte_val&0x7f)) ascii_result[i1] = byte_val;
		i1++;
		if (!(_debug_dump_beg & 0x000f))
		{
			DebugSend("  ");
			DebugSend(ascii_result);
			i1 = 0;
			strcpy(ascii_result,"................");
			if (_debug_dump_beg==0) break;
			if (_debug_dump_beg <= _debug_dump_end)
			{
				DebugPrint("\r\n %08x - ",_debug_dump_beg & 0xfffffff0);
			}
		}
	}
	return (_debug_dump_beg);
}


void debug_parse(char *cmd_line)
{
	int params;
	static uint32_t temp1=0,temp2=0,temp3=0,temp4=0;
	uint16_t tempword;
	uint8_t tempbyte;
	char *next_line;

	while (*cmd_line == ' ') cmd_line++;

	switch (toupper(*cmd_line++))
	{
	case '?':
		DebugPrint("\r\n Compile Date: %s, Time: %s",__DATE__,__TIME__);
		break;
	case 'A':  // assemble
	{
	}
		break;
	case 'B':
	{
	}
		break;
	case 'C':  // compare
	{
	}
		break;
	case 'D':  // dump
		next_line = cmd_line;
		temp2 = strtoul(next_line, &next_line, 16);
		if (temp2>0) {
			_debug_dump_beg = temp2;
			temp3 = strtoul(next_line, &next_line, 16);
			if (temp3>temp2) {
				_debug_dump_end = temp3;
			} else {
				_debug_dump_end = _debug_dump_beg + 127;
			}
		} else {
			_debug_dump_end = _debug_dump_beg + 127;
		}
		do_dump();
		break;
	case 'E':  // read unsigned int
		tempbyte = 0;
		if (sscanf(cmd_line,"%lx",&temp1)==1) {
			tempbyte = temp1;
		}
		break;
	case 'F':  // fill
		params = sscanf(cmd_line,"%lx %lx %lx",&temp2,&temp3,&temp4);
		switch (params) {
		case 3:
			break;
		default:
			{
				DebugSend(" ?? \r\n");
			}
			break;
		}
		break;
	case 'G':  // go
		break;
	case 'H':  // hex
		break;
	case 'I':  // read byte
		if (sscanf(cmd_line,"%lx",&temp1)==1)
		{
			debug_port = (unsigned char*)temp1;
			tempbyte = *(unsigned char*)debug_port;
			DebugPrint("\r\n %08x -> %02x",(int)debug_port,tempbyte);
			debug_port += sizeof(tempbyte);
		}
		break;
	case 'J':  // read word
		if (sscanf(cmd_line,"%lx",&temp1)==1)
		{
			debug_port = (unsigned char*)(temp1&0xFFFFFFFE);
		}
		tempword = *(unsigned short*)debug_port;
		DebugPrint("\r\n %08X -> %04X",(int)debug_port,tempword);
		break;
	case 'K':
		break;
	case 'L':  // load
		DebugPrint("\r\n HAL_RCC_GetSysClockFreq() = %8ld;", HAL_RCC_GetSysClockFreq()/1000000l);
		DebugPrint("\r\n HAL_RCC_GetHCLKFreq() = %8ld;", HAL_RCC_GetHCLKFreq()/1000000l);
		DebugPrint("\r\n HAL_RCC_GetPCLK1Freq() = %8ld;", HAL_RCC_GetPCLK1Freq()/1000000l);
		break;
	case 'M':  // write unsigned int
		if (sscanf(cmd_line,"%lX %lX",&temp1,&temp2)==2)
		{
			debug_port = (unsigned char*)(temp1&0xFFFFFFFC);
			DebugPrint("\r\n %08X <- %08X",(int)debug_port,temp2);
			*(unsigned int*)debug_port = temp2;
			debug_port += sizeof(int);
		}
		else
		{
			DebugSend(" ?? \r\n");
		}
		break;
	case 'N':  // name
		while ((*cmd_line==' ')&&(*cmd_line!=0)) cmd_line++;
		DebugSend("\r\n");
		while (*cmd_line!=0) {
			DebugPutChar(*cmd_line);
			cmd_line++;
		}
		break;
	case 'O':  // output byte
		if (sscanf(cmd_line,"%lx %lx",&temp1,&temp2)==2)
		{
			debug_port = (unsigned char*)temp1;
			tempbyte = temp2;
			DebugPrint("\r\n %08X <- %02X",(int)debug_port,tempbyte);
			*debug_port = tempbyte;
			debug_port += sizeof(tempbyte);
		}
		else
		{
			DebugSend(" ?? \r\n");
		}
		break;
	case 'P':  // proceed
		if (sscanf(cmd_line,"%ld",&temp1)==1)
		{
		}
		else
		{
		}
		break;
	case 'Q':  // quit
		break;
	case 'R':  // register
		break;
	case 'S':  // search
		break;
	case 'T':  // Test
		if (sscanf(cmd_line,"%ld",&temp1)==1)
		{
		}
		else
		{
			bTest = !bTest;
			if (bTest)
			{
				DebugSend("\r\n Test ON!");
			}
			else
			{
				DebugSend("\r\n Test OFF!");
			}
		}
		break;
	case 'U':
		if (sscanf(cmd_line,"%ld %ld",&temp1,&temp2)==2)
		{
		}
		else
		if (sscanf(cmd_line,"%ld",&temp1)==1)
		{
			switch (temp1){
			case 0:
				break;
			case 1:	//U1
				break;
			case 2:	//U2
				break;
			case 3:	//U3
				break;
			case 4:	//U4
				break;
			case 5:	//U5
				break;
			case 6:	//U6
				break;
			case 7:	//U7
				break;
			case 8:	//U8
				break;
			case 9:	//U9
				break;
			case 10:	//U10
				break;
			case 11:	//U11
				break;
			case 12:	//U12
				break;
			case 13:	//U13
				break;
			case 14:	//U14
				break;
			case 15:	//U15
				break;
			case 16:	//U16
				break;
			case 17:	//U17
				break;
			case 18:	//U18
				break;
			}
		}
		else
		{
		}
		break;
	case 'V':
		if (sscanf(cmd_line,"%ld",&temp1)==1)
		{
			switch (temp1){
			case 0:	//V0
				break;
			case 1:	//V1
				break;
			case 2:	//V2
				break;
			case 3:	//V3
				break;
			case 4:	//V4
				break;
			case 5:	//V5
				break;
			case 6:	//V6
				break;
			case 7:	//V7
				break;
			case 8:	//V8
				break;
			case 9:	//V9
				break;
			}
		}
		break;
	case 'W':  // write word
		if (sscanf(cmd_line,"%lx %lx",&temp1,&temp2)==2)
		{
			debug_port = (unsigned char*)(temp1&0xFFFFFFFE);
			tempword = temp2;
			DebugPrint("\r\n %08X <- %04X",(int)debug_port,tempword);
			*(unsigned short*)debug_port = tempword;
			debug_port += sizeof(tempword);
		}
		else if (sscanf(cmd_line,"%ld",&temp1)==1)
		{
			switch (temp1){
			case 0:
			{
			}
			break;
			case 1:
			{
			}
			break;
			case 2:
			{
			}
			break;
			case 3:
			{
			}
			break;
			case 4:
			{
			}
			break;
			case 5:
				break;
			case 6:
				break;
			case 7:
				break;
			case 8:
				break;
			case 9:
				{
				}
				break;
			}
		}
		break;
	case 'X':
		if (sscanf(cmd_line,"%ld",&temp1)==1)
		{
			//DoDebugMain(temp1);
		}
		else
		{
		}
		break;
	case 'Y':
		temp4 = sscanf(cmd_line,"%ld %lx",&temp1,&temp2);
		{
			switch (temp1){
			case 0:	//Y0
				{
				}
				break;
			case 1:
				{
				}
				break;
			case 2:	//Y2
				{
				}
				break;
			case 3:	//Y3
				{
				}
				break;
			case 4:	//Y4
				{
				}
				break;
			case 5:	//Y5
				{
				}
				break;
			case 6:
				{
				}
				break;
			case 7:	//Y7
				{
				}
				break;
			case 8:	//Y8
				{
				}
				break;
			case 9:	//Y9
				{
				}
				break;
			case 10:	//Y10
				{
				}
				break;
			case 11:	//Y11
				{
				}
				break;
			case 12:	//Y12
				{
				}
				break;
			case 13:	//Y13
				{
				}
				break;
			case 14:	//Y14
				{
				}
				break;
			case 15:	//Y15
				{
				}
				break;
			}
		}
		break;
	case 'Z':
		if (sscanf(cmd_line,"%li %li",&temp1,&temp2)==2)
		{
			DebugMain(temp1);
		}
		else
		if (sscanf(cmd_line,"%li",&temp1)==1)
		{
			DebugMain(temp1);
		}
		else
		{
			_debug_dump_beg = _old_debug_dump_beg;
			_debug_dump_end = _debug_dump_beg + 127;
			do_dump();
		}
		break;
	default:
		;
	}
}

static int  lineptr = 0;
static char linebuff[256];

void debug_rcv(uint8_t ch)
{
	if ((ch=='\r') || (lineptr==255))
	{
		linebuff[lineptr] = 0;
		if (lineptr)
		{
			debug_parse(linebuff);
		}
		lineptr = 0;
		SendDebugPrompt;
	}
	else if (iscntrl(ch))
	{
		switch (ch)
		{
		case BS:
			if (lineptr)
			{
				DebugPutChar(ch);
				lineptr--;
			}
			break;
		}
	}
	else
	{
		linebuff[lineptr++] = ch;
		DebugPutChar(ch);
		//__io_putchar(ch);
	}
	//ResetTimer(DEBUGTIMER);
}

void debug_idle(void)
{
	if (bTest)
	{
	}
}

void DebugInit(void)
{
#ifdef huart_debug
	debug_data.huart = &huart_debug;
#ifndef	TX_BUFFER_SIZE
	debug_data.TX_Buffer_size = DBG_TX_BUFFER_SIZE;
	debug_data.TX_Buffer = dbg_tx_buffer;
#endif
	debug_data.COMM_rcv = debug_rcv;
	debug_data.RX_DMA_Buffer_head = 0;
	debug_data.RX_DMA_Buffer_tail = 0;
	debug_data.TX_Buffer_head = 0;
	debug_data.TX_Buffer_tail = 0;
	debug_data.TX_DMA_Lock = 0;
	if (debug_data.huart->hdmarx) {
		HAL_UART_Receive_DMA(debug_data.huart,
				debug_data.RX_DMA_Buffer, RX_DMA_BUFFER_SIZE);
	}
#endif
	DebugPrint("\r\n  Welcome to Debug Console ver STM1.1!");
	DebugPrint("\r\n Compile Date: %s, Time: %s",__DATE__,__TIME__);
	SendDebugPrompt;
}

static char DebugLock = 0;

#define	_USE_SOF_	0

void DebugTask(void)
{
#ifdef huart_debug
	if (!DebugLock)
	{
		DebugLock = 1;	//prevent recursion
		if (huart_debug.hdmarx==NULL)
		{
			uint8_t rx_char;
			if (HAL_UART_Receive(&huart_debug, &rx_char, 1, 0)==HAL_OK)
			{
				debug_rcv(rx_char);
			}
			else
			{
	#if DEBUGTIMER
				StartTimer(DEBUGTIMER, 250);
				if (TimerOut(DEBUGTIMER))
				{
					ResetTimer(DEBUGTIMER);
					debug_idle();
				}
	#endif
			}
		}
		else
		{
	#if DEBUGTIMER
			StartTimer(DEBUGTIMER, 250);
			if (TimerOut(DEBUGTIMER))
			{
				ResetTimer(DEBUGTIMER);
				debug_idle();
			}
	#endif
		}
		DebugLock = 0;
	}
	CommTXTask(&debug_data);
#else
	if (!DebugLock)
	{
		DebugLock = 1;	//prevent recursion
		if (dbg_rx_head != dbg_rx_tail) {
			do {
				char _rxchar = dbg_rx_buffer[dbg_rx_tail++];
				if (dbg_rx_tail >= DBG_RX_BUFFER_SIZE)	dbg_rx_tail = 0;
				debug_rcv(_rxchar);
			} while (dbg_rx_head!=dbg_rx_tail);
		} else {
			//StartTimer(DEBUGTIMER,50);
//			if (TimerOut(DEBUGTIMER))
//			{
//				ResetTimer(DEBUGTIMER);
//				debug_idle();
//			}
		}
		DebugLock = 0;
	}
	#if (_USE_SOF_==0)
	DoDebugSerial();
	#endif
#endif
}

#if _USE_SOF_
volatile uint8_t USB_Lock = 0;
#endif

void DebugPutChar(char ch)
{
#ifdef huart_debug
	CommUARTPutChar(&debug_data, ch);
#else
 #if _USE_SOF_
	USB_Lock = 1;
 #endif
	dbg_tx_buffer[dbg_tx_head++] = ch;
	if (dbg_tx_head>=DBG_TX_BUFFER_SIZE) dbg_tx_head = 0;
	if (dbg_tx_head==dbg_tx_tail) {
		dbg_tx_tail++;	//discard oldest
		if (dbg_tx_tail>=DBG_TX_BUFFER_SIZE) dbg_tx_tail = 0;
	}
 #if _USE_SOF_
	USB_Lock = 0;
 #endif
#endif
}

void DebugSend(char *message)
{
	char ch = *message;

	while (ch != '\0')
	{
		DebugPutChar(ch);
		message++;
		ch = *message;
	}
}

void DebugPrint(const char *format, ...)
{
	char debug_result[82];
	va_list argptr;
    va_start(argptr, format);
    vsprintf(debug_result, format, argptr);
    va_end(argptr);
	DebugSend(debug_result);
}

extern uint8_t USB_Transmit(uint8_t* Buf, uint16_t Len);

void DoDebugSerial(void)
{
 #if _USE_SOF_
	if (USB_Lock) return;
 #endif
	if (dbg_tx_head!=dbg_tx_tail)
	{
		int16_t usb_tx_len = dbg_tx_head - dbg_tx_tail;
		if (usb_tx_len<0) usb_tx_len += DBG_TX_BUFFER_SIZE;
		if (usb_tx_len>DBG_DMA_BUFFER_SIZE) usb_tx_len = DBG_DMA_BUFFER_SIZE;
		for (uint16_t _i = 0; _i<usb_tx_len; _i++) {
			dbg_dma_buffer[_i] = dbg_tx_buffer[dbg_tx_tail++];
			if (dbg_tx_tail >= DBG_TX_BUFFER_SIZE) dbg_tx_tail = 0;
		}
		USB_Transmit(dbg_dma_buffer, usb_tx_len);
	}
}

inline char IsTest(void)
{
	return bTest;
}

/* redirect printf to debug port */
int __io_putchar(int ch)
{
	DebugPutChar(ch);
	return ch;
}

#if defined(__USBD_CDC_IF_H__)
volatile uint8_t SOF_count;

uint8_t DEBUG_USB_SOF(USBD_HandleTypeDef *pdev)
{
	SOF_count++;
	//if (!SOF_count)	HAL_GPIO_TogglePin(LED_Out_GPIO_Port, LED_Out_Pin);
#if _USE_SOF_
	DoDebugSerial();
#endif
	return USBD_OK;
}
#endif

#ifdef __USBD_CDC_IF_H__
uint8_t USB_Receive(uint8_t* Buf, uint16_t length)
{
	for (uint16_t _i = 0; _i<length; _i++) {
		dbg_rx_buffer[dbg_rx_head++] = Buf[_i];
		if (dbg_rx_head >= DBG_RX_BUFFER_SIZE) dbg_rx_head = 0;
		if (dbg_rx_head==dbg_rx_tail) {
			dbg_rx_tail++;
			if (dbg_rx_tail >= DBG_RX_BUFFER_SIZE) dbg_rx_tail = 0;
		}
	}
	return USBD_OK;
}
#endif
