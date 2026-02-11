/**
************************************************************
* @file         gizwits_product.h
* @brief        Corresponding gizwits_product.c header file (including product hardware and software version definition)
* @author       Gizwits
* @date         2017-07-19
* @version      V03030000
* @copyright    Gizwits
* 
* @note         机智云.只为智能硬件而生
*               Gizwits Smart Cloud  for Smart Products
*               链接|增值ֵ|开放|中立|安全|自有|自由|生态
*               www.gizwits.com
*
***********************************************************/
#ifndef _GIZWITS_PRODUCT_H
#define _GIZWITS_PRODUCT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stm32f1xx.h>
#include "gizwits_protocol.h"

/**
* MCU software version
*/
#define SOFTWARE_VERSION "03030000"
/**
* MCU hardware version
*/
#define HARDWARE_VERSION "03010100"


/**
* Communication module model
*/
#define MODULE_TYPE 0 //0,WIFI ;1,GPRS


/**@name TIM3 related macro definition
* @{
*/
#define TIMER 					TIM3
#define TIMER_IRQ 				TIM3_IRQn
#define TIMER_RCC 				RCC_APB1Periph_TIM3
#define TIMER_IRQ_FUN 			TIM3_IRQHandler
/**@} */

/**@name USART related macro definition
* @{
*/
#define UART_BAUDRATE 			9600
#define UART_PORT     			2
#define UART          			USART2
#define UART_IRQ      			USART2_IRQn
#define UART_IRQ_FUN  			USART2_IRQHandler

#if (UART_PORT == 1)
#define UART_GPIO_Cmd          RCC_APB2PeriphClockCmd
#define UART_GPIO_CLK          RCC_APB2Periph_GPIOA

#define UART_AFIO_Cmd          RCC_APB2PeriphClockCmd
#define UART_AFIO_CLK          RCC_APB2Periph_AFIO

#define UART_CLK_Cmd           RCC_APB2PeriphClockCmd
#define UART_CLK               RCC_APB2Periph_USART1 

#define UART_GPIO_PORT         GPIOA
#define UART_RxPin             GPIO_Pin_10
#define UART_TxPin             GPIO_Pin_9
#endif

#if (UART_PORT == 2)
#define UART_GPIO_Cmd          RCC_APB2PeriphClockCmd
#define UART_GPIO_CLK          RCC_APB2Periph_GPIOA

#define UART_AFIO_Cmd          RCC_APB2PeriphClockCmd
#define UART_AFIO_CLK          RCC_APB2Periph_AFIO

#define UART_CLK_Cmd           RCC_APB1PeriphClockCmd
#define UART_CLK               RCC_APB1Periph_USART2 

#define UART_GPIO_PORT         GPIOA
#define UART_RxPin             GPIO_Pin_3
#define UART_TxPin             GPIO_Pin_2
#endif


#if (UART_PORT == 3)

#define UART_GPIO_Cmd          RCC_APB2PeriphClockCmd
#define UART_GPIO_CLK          RCC_APB2Periph_GPIOC

#define UART_AFIO_Cmd          RCC_APB2PeriphClockCmd
#define UART_AFIO_CLK          RCC_APB2Periph_AFIO

#define UART_CLK_Cmd           RCC_APB1PeriphClockCmd
#define UART_CLK               RCC_APB1Periph_USART3 

#define UART_GPIO_PORT         GPIOC
#define UART_RxPin             GPIO_Pin_11
#define UART_TxPin             GPIO_Pin_10

#endif
/**@} */

/** User area the current device state structure*/
extern dataPoint_t currentDataPoint;

#define USART3_REC_LEN  200	//定义USART2最大接收字节数
extern unsigned char u3NewBuffer[USART3_REC_LEN];			//串口3接收缓存
extern uint16_t USART3_RX_STA ;//接收状态标记



void gizTimerMs(void);
void timerInit(void);
void uartInit(void);
void userInit(void);
void userHandle(void);
void mcuRestart(void);
uint32_t gizGetTimerCount(void);
int32_t uartWrite(uint8_t *buf, uint32_t len);
int8_t gizwitsEventProcess(eventInfo_t *info, uint8_t *data, uint32_t len);


void CZL_USART_UART_Init(UART_HandleTypeDef huart); //串口DMA接收模式初始化函数   需在 MX_USARTX_UART_Init(void) 函数结束时调用
void CZL_UART_RxCpltCallback(UART_HandleTypeDef* huart); //串口中断回调函数   需在 USART1_IRQHandler() 函数结束时调用

#ifdef __cplusplus
}
#endif

#endif
