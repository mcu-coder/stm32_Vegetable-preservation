#ifndef __OLED_H
#define __OLED_H

#include "main.h"
#include "stm32f1xx_hal.h"
/*“˝Ω≈≈‰÷√*/


#define OLED_W_SDA(x)		HAL_GPIO_WritePin(OLED_SDA_GPIO_Port,OLED_SDA_Pin,(GPIO_PinState)(x))
#define OLED_W_SCL(x)		HAL_GPIO_WritePin(OLED_SCL_GPIO_Port,OLED_SCL_Pin,(GPIO_PinState)(x))

void OLED_Init(void);
void OLED_Clear(void);
//void OLED_SetHead(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowChinese(uint8_t Line, uint8_t Column, uint8_t num);



void WS_OLED_Printf(int  x,int  y , char *p,... );


#endif


