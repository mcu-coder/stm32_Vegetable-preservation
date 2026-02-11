/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2025 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "hal_key.h"
#include "gizwits_product.h"
#include "common.h"

#include "dht11.h"
#include "OLED.h"
#include "motor.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

char timerFlag_1ms = 0;
char timerFlag_10ms = 0;
char timerFlag_1000ms = 0;

 char Mode = 1 ;
 uint8_t Temp , Humi ;
 char Set_Mode = 0;
 uint8_t Temp_HIGHT=30,Temp_LOW=20;
 uint8_t Humi_HIGHT=80,Humi_LOW=40;
 uint16_t CO2_HIGHT =400;
 uint16_t CO2_x  =  0;
 
 char Flay_num = 2;
 char FLay_num_last =0  ;
 u8 motor_flag;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void softwareTimer(void);
void openBuzzerimer(int time);
void usart3(void);

char KEY_scanf(void);

void GET_DATA(void)
{
   DHT11_Read_Data(&Temp,&Humi); 
}

void Meun_Show(void)
{
   
    //显示“温度：  C”
	OLED_ShowChinese(3,1,8);OLED_ShowChinese(3,2,9); OLED_ShowChinese(3,5,10);OLED_ShowChinese(3,6,9);
    //显示“二氧化碳： ”
    OLED_ShowChinese(4, 1, 24);OLED_ShowChinese(4, 2, 25);OLED_ShowChinese(4, 3, 26);OLED_ShowChinese(4, 4, 27);
    
    
    if(Mode == 1)
    {
        OLED_ShowChinese(2, 6, 23);
		OLED_ShowChinese(2, 7, 22);	 
    }
}
 
void Vol_Mode(void)
{
    
    if(CO2_x>CO2_HIGHT)
    {
        //打开通风口
			if(motor_flag==0)
			{
       Stepper_Move(1,512);
				motor_flag=1;
			}
    }
    else{
        //关闭通风口
			if(motor_flag==1)
			{
        Stepper_Move(0,512);
				motor_flag=0;
			}
    }
}

void Set_show(void)
{
    //界面标题
    OLED_ShowChinese(1, 1, 17);OLED_ShowChinese(1, 2, 18);OLED_ShowChinese(1, 3, 30);OLED_ShowChinese(1, 4, 31);
    OLED_ShowChinese(1, 5, 32);OLED_ShowChinese(1, 6, 33);OLED_ShowChinese(1, 7, 34);OLED_ShowChinese(1, 8, 35);
}

void Temp_set_show(void)
{
	OLED_ShowString(2, 3, "  ");
    //显示“温度上限：”
	OLED_ShowChinese(2, 3, 8);OLED_ShowChinese(2, 4, 9);OLED_ShowChinese(2, 5, 36);OLED_ShowChinese(2, 6, 37);	
	OLED_ShowChar(2, 11, ':');
	 
    
}

void Humi_set_show(void)
{
   
	//显示“湿度下限：”
	OLED_ShowChinese(3, 3, 10);OLED_ShowChinese(3, 4, 9);OLED_ShowChinese(3, 5, 38);OLED_ShowChinese(3, 6, 39);	
	OLED_ShowChar(3, 11, ':');
}

void CO2_set_show(void)
{   
    //显示“二氧化碳：”
	OLED_ShowChinese(2, 2, 24);OLED_ShowChinese(2, 3, 25);OLED_ShowChinese(2, 4, 26);OLED_ShowChinese(2, 5, 27);
	OLED_ShowChar(2, 11, ':');	   
}


/*****二氧化碳读取*******/
//void JIAN_call (unsigned char *buffer)
//{
//   if (buffer[0] == 0x2C)
//   {
//       CO2_x= buffer[1]+buffer[2]*256;
//   }
//}

void usart3(void)
{
    static char  RxState = 0 ;
	if (USART3_RX_STA == 1)
	{
		USART3_RX_STA = 0; 
        /*
            HAL_UART_Transmit_IT(&huart3,u3NewBuffer,sizeof u3NewBuffer);   串口接收调试口
            memset(u3NewBuffer, '\0', strlen((char *)u3NewBuffer)); // 清除数据
            对串口进行调试的时候请关闭清楚函数 ,否者发送的数据会出现问题
            使用方法：
            1、将需要进行测试的串口，通过USB-TTL接入电脑打开串口助手 bps9600。
            2、在串口3的uart3()函数中打开 HAL_UART_Transmit_IT 注释清除数据。
            3、电脑发送数据并且查看接收的数据是否完整。
        */
       switch(RxState)
       {
            
           case 0:
               if(u3NewBuffer[5] == (uint8_t)(u3NewBuffer[0] + u3NewBuffer[1]+ u3NewBuffer[2] + u3NewBuffer[3] + u3NewBuffer[4]))	//验证接收到的数据是否正确
               {
                  RxState = 0;
                  CO2_x = u3NewBuffer[1]*256+u3NewBuffer[2];
               } 
               else
               {
                   RxState = 0;
               }
       }	
	}
}


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
    char  key_NUM  = 0 ;
    static char operation = 0 ;
    static char flay=0,flay_1=0,flay_2=0;
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();

  /* USER CODE BEGIN 2 */
    timerInit();
    uartInit();
 
    
    OLED_Init();
    OLED_Clear();
    
    CZL_USART_UART_Init(huart3);//uart3串口初始化,开启DMA接收 
    GIZWITS_LOG("MCU Init Success , SoftVersion = %s\r\n",SOFTWARE_VERSION);
  
    

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        
          key_NUM = KEY_scanf();    
           // 数据读取
          if(timerFlag_1ms  != 0 )
		  {
				timerFlag_1ms = 0;  
                GET_DATA();
					if(Mode == 1)//自动模式
					{
						Vol_Mode(); // 上电默认为自动模式
					}
						softwareTimer();
		  }
      
           if(key_NUM > 0)
           {
               if(key_NUM  ==  1)
               {
                   Mode++;
                   if(Mode >3)
                   {
                       if(flay_2==0)
                       {
                           flay_2=~flay_2;
                           OLED_Clear();
                       }
                       flay=~flay;
                       flay_1=~flay_1;
                       flay_2=~flay_2;
                       Mode =1;
                       Set_Mode =0;
                   }
               }
               
              
               
               if(Mode ==3) //设置值模式
               {
                   
                   if(Set_Mode ==0)
                   {
                       if(key_NUM ==2)
                       {
                           Temp_HIGHT++;
                       }
                       if(key_NUM ==3)
                       {
                           Temp_HIGHT--;
                       }
                       Temp_set_show();
                       OLED_ShowChar(2,2,'>');
                       WS_OLED_Printf(2,12,"%d    ",Temp_HIGHT);
                       WS_OLED_Printf(3,12,"%d    ",Temp_LOW);
                   }
                    
                   if(Set_Mode ==3)
                   {
                        if(key_NUM ==2)
                       {
                           Humi_LOW++;
                       }
                       if(key_NUM ==3)
                       {
                           Humi_LOW--;
                       } 
                       
                       Humi_set_show();
                       OLED_ShowChar(2,2,' ');
                       OLED_ShowChar(3,2,'>');
                       WS_OLED_Printf(2,12,"%d    ",Humi_HIGHT);
                       WS_OLED_Printf(3,12,"%d    ",Humi_LOW);
                   }
                  
               }
                openBuzzerimer(1);
           }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
        userHandle();
        gizwitsHandle((dataPoint_t *)&currentDataPoint);
    }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** NVIC Configuration
*/
static void MX_NVIC_Init(void)
{
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USART2_IRQn interrupt configuration */ 
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7200-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10-1; 
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE; 
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  __HAL_AFIO_REMAP_SWJ_NOJTAG();
    
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BUZZ_Pin|MOTOR_A_Pin|MOTOR_B_Pin|MOTOR_D_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OLED_SDA_Pin|Feng_Pin|JIA_Pin|MOTOR_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : BUZZ_Pin DHT11_Pin MOTOR_A_Pin MOTOR_B_Pin
                           MOTOR_D_Pin */
  GPIO_InitStruct.Pin = BUZZ_Pin|DHT11_Pin|MOTOR_A_Pin|MOTOR_B_Pin
                          |MOTOR_D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_SDA_Pin OLED_SCL_Pin Feng_Pin JIA_Pin
                           MOTOR_C_Pin */
  GPIO_InitStruct.Pin = OLED_SDA_Pin|OLED_SCL_Pin|Feng_Pin|JIA_Pin
                          |MOTOR_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; 

  /*Configure GPIO pins : K4_Pin K3_Pin K2_Pin K1_Pin */
  GPIO_InitStruct.Pin = K4_Pin|K3_Pin|K2_Pin|K1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
char KEY_scanf(void)
{
            uint8_t KeyNum = 0;
            if (HAL_GPIO_ReadPin(GPIOB, K1_Pin) == 0)
            {
                HAL_Delay(20);
                while (HAL_GPIO_ReadPin(GPIOB, K1_Pin) == 0);
                HAL_Delay(20);
                KeyNum = 1;
            }
            
          
            return KeyNum; 
}

void softwareTimer_IRQ(void) // 1ms定时中断
{
	static char count_10ms = 0;
	static int count_1000ms = 0;
	if (++count_10ms == 10)
	{
		count_10ms = 0;
		timerFlag_10ms = 1;
	}
	if (++count_1000ms == 100)
	{
		count_1000ms = 0;
		timerFlag_1000ms = 1;
	}
	timerFlag_1ms = 1;
}


int  buzzerTimerCount = 0;
//  控制蜂鸣器打开时间的函数
//  time:  打开时间，单位MS
void openBuzzerimer(int time)
{
     HAL_GPIO_WritePin(BUZZ_GPIO_Port,BUZZ_Pin,GPIO_PIN_SET)  ;
	 buzzerTimerCount = time ;
}
// 软件定时器 ，  定时调用，间隔1MS
void softwareTimer(void)
{	
   if(buzzerTimerCount > 0)
		{
		  if( -- buzzerTimerCount == 0)
			{
                HAL_GPIO_WritePin(BUZZ_GPIO_Port,BUZZ_Pin,GPIO_PIN_RESET)  ;   
			}
		}		
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while(1) 
    {
    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
        ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
