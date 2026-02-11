#ifndef __DHT11_H
#define __DHT11_H 
#include "main.h" 
#include "stm32f1xx_hal.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK精英STM32F103开发板V1
//DHT11数字温湿度传感器驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2015/1/16
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
    
//IO方向设置
#define DHT11_HIGH     HAL_GPIO_WritePin(GPIOA, DHT11_Pin,	GPIO_PIN_SET)
#define DHT11_LOW      HAL_GPIO_WritePin(GPIOA, DHT11_Pin, GPIO_PIN_RESET)
#define DHT11_DQ_IN    HAL_GPIO_ReadPin(GPIOA, DHT11_Pin)
#define u8 uint8_t


u8 DHT11_Init(void);//初始化DHT11
u8 DHT11_Read_Data(u8 *temp,u8 *humi);//读取温湿度
u8 DHT11_Read_Byte(void);	//读出一个字节
u8 DHT11_Read_Bit(void);	//读出一个位
u8 DHT11_Check(void);		//检测是否存在DHT11
void DHT11_Rst(void);		//复位DHT11    
#endif















