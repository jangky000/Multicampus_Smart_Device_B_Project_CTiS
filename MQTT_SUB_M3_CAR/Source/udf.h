#include <stdio.h>
#include "utype.h"
#include "socket.h"	// Just include one header for WIZCHIP

///////////////////////////////////////////
//////////////ERROR CODE///////////////////
///////////////////////////////////////////
#define EC_SUCCESS		(0)
#define EC_FAILED			(1)
#define EC_DATA_FULL	(100)
#define EC_DATA_EMPTY	(101)


typedef struct _CONFIG_TYPE_DEF
{
	uint16_t port;
	uint8_t destip[4];
}CHCONFIG_TYPE_DEF;

// uart.c
void Uart1_Init(int baud);
void Uart1_Printf(char *fmt,...);

void Uart2_Init(int baud);
void Uart2_Printf(char *fmt,...);

// timer.c
void TIM_Config(void);
void TIM_Delay_ms(unsigned int time);
