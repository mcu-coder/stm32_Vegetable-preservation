/**
************************************************************
* @file         gizwits_product.c
* @brief        Gizwits control protocol processing, and platform-related       hardware initialization 
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

#include <stdio.h>
#include <string.h>
#include "hal_key.h"
#include "gizwits_product.h"
#include "common.h"
#include "motor.h"
static uint32_t timerMsCount;
uint8_t aRxBuffer;

/** User area the current device state structure*/
dataPoint_t currentDataPoint;

extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern uint8_t Temp , Humi ;
extern uint8_t motorFlag;

unsigned char u3NewBuffer[USART3_REC_LEN];			//串口3接收缓存
uint16_t USART3_RX_STA =0;//接收状态标记
extern uint8_t  CO2_x ;

extern char Mode ;
extern uint8_t Temp_HIGHT,Temp_LOW;
extern uint8_t Humi_HIGHT,Humi_LOW;
extern uint8_t CO2_HIGHT ;
//extern
//extern

/**@} */
/**@name Gizwits User Interface
* @{
*/

/**
* @brief Event handling interface

* Description:

* 1. Users can customize the changes in WiFi module status

* 2. Users can add data points in the function of event processing logic, such as calling the relevant hardware peripherals operating interface

* @param [in] info: event queue
* @param [in] data: protocol data
* @param [in] len: protocol data length
* @return NULL
* @ref gizwits_protocol.h
*/
int8_t gizwitsEventProcess(eventInfo_t *info, uint8_t *gizdata, uint32_t len)
{
    uint8_t i = 0;
    dataPoint_t *dataPointPtr = (dataPoint_t *)gizdata;
    moduleStatusInfo_t *wifiData = (moduleStatusInfo_t *)gizdata;
    protocolTime_t *ptime = (protocolTime_t *)gizdata;
    
#if MODULE_TYPE
    gprsInfo_t *gprsInfoData = (gprsInfo_t *)gizdata;
#else
    moduleInfo_t *ptModuleInfo = (moduleInfo_t *)gizdata;
#endif

    if((NULL == info) || (NULL == gizdata))
    {
        return -1;
    }

    for(i=0; i<info->num; i++)
    {
        switch(info->event[i])
        {
        case EVENT_motor:
            currentDataPoint.valuemotor = dataPointPtr->valuemotor;
            GIZWITS_LOG("Evt: EVENT_motor %d \n", currentDataPoint.valuemotor);
            if(0x01 == currentDataPoint.valuemotor)
            {
							if(Mode == 2)
							{
                Stepper_Move(1,512);
							}
            //user handle
            }
            else
            {
							if(Mode == 2)
							{
                Stepper_Move(0,512);
							}
            //user handle    
            }
            break;
        case EVENT_air_conditioner:
            currentDataPoint.valueair_conditioner = dataPointPtr->valueair_conditioner;
            GIZWITS_LOG("Evt: EVENT_air_conditioner %d \n", currentDataPoint.valueair_conditioner);
            if(0x01 == currentDataPoint.valueair_conditioner)
            {
							{
								if(Mode == 2)
							{
                 HAL_GPIO_WritePin(Feng_GPIO_Port,Feng_Pin,GPIO_PIN_SET);
							}
							}
            //user handle
            }
            else
            {
							if(Mode == 2)
							{
                HAL_GPIO_WritePin(Feng_GPIO_Port,Feng_Pin,GPIO_PIN_RESET);
							}
            //user handle    
            }
            break;
        case EVENT_humidifier:
            currentDataPoint.valuehumidifier = dataPointPtr->valuehumidifier;
            GIZWITS_LOG("Evt: EVENT_humidifier %d \n", currentDataPoint.valuehumidifier);
            if(0x01 == currentDataPoint.valuehumidifier)
            {
            //user handle
							if(Mode == 2)
							{
                HAL_GPIO_WritePin(JIA_GPIO_Port,JIA_Pin,GPIO_PIN_SET);
							}
            }
            else
            {
            //user handle   
							if(Mode == 2)
							{							
                HAL_GPIO_WritePin(JIA_GPIO_Port,JIA_Pin,GPIO_PIN_RESET);
							}
            }
            break;

        case EVENT_model:
            currentDataPoint.valuemodel = dataPointPtr->valuemodel;
            GIZWITS_LOG("Evt: EVENT_model %d\n", currentDataPoint.valuemodel);
            switch(currentDataPoint.valuemodel)
            {
            case model_VALUE0:
                Mode = 1;
                //user handle
                break;
            case model_VALUE1:
                Mode = 2;
                //user handle
                break;
            default:
                break;
            }
            break;

        case EVENT_tempUpperThreshold:
            currentDataPoint.valuetempUpperThreshold = dataPointPtr->valuetempUpperThreshold;
            GIZWITS_LOG("Evt:EVENT_tempUpperThreshold %d\n",currentDataPoint.valuetempUpperThreshold);
            //user handle
            Temp_HIGHT =  currentDataPoint.valuetempUpperThreshold;
            break;
        case EVENT_tempLowerThreshold:
            currentDataPoint.valuetempLowerThreshold = dataPointPtr->valuetempLowerThreshold;
            GIZWITS_LOG("Evt:EVENT_tempLowerThreshold %d\n",currentDataPoint.valuetempLowerThreshold);
            //user handle
            Temp_LOW  =currentDataPoint.valuetempLowerThreshold;
            break;
        case EVENT_humiUpperThreshold:
            currentDataPoint.valuehumiUpperThreshold = dataPointPtr->valuehumiUpperThreshold;
            GIZWITS_LOG("Evt:EVENT_humiUpperThreshold %d\n",currentDataPoint.valuehumiUpperThreshold);
            //user handle
            Humi_HIGHT =currentDataPoint.valuehumiUpperThreshold;
            break;
        case EVENT_humiLowerThreshold:
            currentDataPoint.valuehumiLowerThreshold = dataPointPtr->valuehumiLowerThreshold;
            GIZWITS_LOG("Evt:EVENT_humiLowerThreshold %d\n",currentDataPoint.valuehumiLowerThreshold);
            //user handle
            Humi_LOW =currentDataPoint.valuehumiLowerThreshold;
            break;
        case EVENT_CO2Value:
            currentDataPoint.valueCO2Value = dataPointPtr->valueCO2Value;
            GIZWITS_LOG("Evt:EVENT_CO2Value %d\n",currentDataPoint.valueCO2Value);
            //user handle
            CO2_HIGHT= currentDataPoint.valueCO2Value;
            break;
        case WIFI_SOFTAP:
            break;
        case WIFI_AIRLINK:
            break;
        case WIFI_STATION:
            break;
        case WIFI_CON_ROUTER:
 
            break;
        case WIFI_DISCON_ROUTER:
 
            break;
        case WIFI_CON_M2M:
 
            break;
        case WIFI_DISCON_M2M:
            break;
        case WIFI_RSSI:
            GIZWITS_LOG("RSSI %d\n", wifiData->rssi);
            break;
        case TRANSPARENT_DATA:
            GIZWITS_LOG("TRANSPARENT_DATA \n");
            //user handle , Fetch data from [data] , size is [len]
            break;
        case WIFI_NTP:
            GIZWITS_LOG("WIFI_NTP : [%d-%d-%d %02d:%02d:%02d][%d] \n",ptime->year,ptime->month,ptime->day,ptime->hour,ptime->minute,ptime->second,ptime->ntp);
            break;
        case MODULE_INFO:
            GIZWITS_LOG("MODULE INFO ...\n");
#if MODULE_TYPE
            GIZWITS_LOG("GPRS MODULE ...\n");
            //Format By gprsInfo_t
            GIZWITS_LOG("moduleType : [%d] \n",gprsInfoData->Type);
#else
            GIZWITS_LOG("WIF MODULE ...\n");
            //Format By moduleInfo_t
            GIZWITS_LOG("moduleType : [%d] \n",ptModuleInfo->moduleType);
#endif
        break;
        default:
            break;
        }
    }

    return 0;
}

/**
* User data acquisition

* Here users need to achieve in addition to data points other than the collection of data collection, can be self-defined acquisition frequency and design data filtering algorithm

* @param none
* @return none
*/
void userHandle(void)
{
    currentDataPoint.valuetemperature = Temp;//Add Sensor Data Collection
    currentDataPoint.valuehumidity = Humi;//Add Sensor Data Collection
    currentDataPoint.valueCO2 = CO2_x+256;//Add Sensor Data Collection
}

/**
* Data point initialization function

* In the function to complete the initial user-related data
* @param none
* @return none
* @note The developer can add a data point state initialization value within this function
*/
void userInit(void)
{
    memset((uint8_t*)&currentDataPoint, 0, sizeof(dataPoint_t));
    
    /** Warning !!! DataPoint Variables Init , Must Within The Data Range **/ 
    /*
    currentDataPoint.valuemotor = ;
    currentDataPoint.valueair_conditioner = ;
    currentDataPoint.valuehumidifier = ;
    currentDataPoint.valuemodel = ;
    currentDataPoint.valuetemperature = ;
    currentDataPoint.valuehumidity = ;
    currentDataPoint.valuetempUpperThreshold = ;
    currentDataPoint.valuetempLowerThreshold = ;
    currentDataPoint.valuehumiUpperThreshold = ;
    currentDataPoint.valuehumiLowerThreshold = ;
    currentDataPoint.valueCO2 = ;
    currentDataPoint.valueCO2Value = ;
    */

}


/**
* @brief Millisecond timing maintenance function, milliseconds increment, overflow to zero

* @param none
* @return none
*/
void gizTimerMs(void)
{
    timerMsCount++;
}

/**
* @brief Read millisecond count

* @param none
* @return millisecond count
*/
uint32_t gizGetTimerCount(void)
{
    return timerMsCount;
}

/**
* @brief MCU reset function

* @param none
* @return none
*/
void mcuRestart(void)
{
    __set_FAULTMASK(1);
    HAL_NVIC_SystemReset();
}

/**@} */

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
 
  return ch;
}

/**
  * @brief  Period elapsed callback in non blocking mode 
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim2)
	{
			keyHandle();
			gizTimerMs();
	}
}

/**
* @brief Timer TIM3 init function

* @param none
* @return none
*/
void timerInit(void)
{
	HAL_TIM_Base_Start_IT(&htim2);
}

void CZL_USART_UART_Init(UART_HandleTypeDef huart)
{	
    if (huart.Instance == USART3)
    {
        __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
        HAL_UART_Receive_DMA(&huart3, u3NewBuffer, sizeof(u3NewBuffer));
    }
		HAL_Delay(3);
}

void CZL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	int temp,size;
	if(__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE)!=RESET)
	{
		__HAL_UART_CLEAR_FLAG(huart,UART_FLAG_IDLE|UART_FLAG_RXNE);
		
		temp=huart->Instance->DR;
		temp=huart->Instance->SR;
		
		HAL_UART_DMAStop(huart);
		size=huart->RxXferSize - __HAL_DMA_GET_COUNTER(huart->hdmarx);
		huart->RxXferCount=size;		
		size=temp;
		
		__HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);
		
				
		if(huart->Instance==USART3)
		{
			HAL_UART_Receive_DMA(huart,u3NewBuffer,sizeof(u3NewBuffer));			
		}
		
		HAL_UART_RxCpltCallback(huart);
	}
}	


/**
  * @brief  This function handles USART IDLE interrupt.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef*UartHandle)  
{  
    UNUSED(UartHandle);
    if(UartHandle->Instance == USART2)  
    {  
				gizPutData((uint8_t *)&aRxBuffer, 1);

        HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBuffer, 1);//开启下一次接收中断  
    }  
    
    if(UartHandle->Instance == USART3)
    {
       USART3_RX_STA = 1;
    }
}  

/**
* @brief USART init function

* Serial communication between WiFi modules and device MCU
* @param none
* @return none
*/
void uartInit(void)
{
	HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBuffer, 1);//开启下一次接收中断  
}

/**
* @brief Serial port write operation, send data to WiFi module
*
* @param buf      : buf address
* @param len      : buf length
*
* @return : Return effective data length;-1，return failure
*/
int32_t uartWrite(uint8_t *buf, uint32_t len)
{
		uint8_t crc[1] = {0x55};
    uint32_t i = 0;
	
    if(NULL == buf)
    {
        return -1;
    }

    for(i=0; i<len; i++)
    {
        HAL_UART_Transmit_IT(&huart2, (uint8_t *)&buf[i], 1);
				while (huart2.gState != HAL_UART_STATE_READY);//Loop until the end of transmission

        if(i >=2 && buf[i] == 0xFF)
        {
						HAL_UART_Transmit_IT(&huart2, (uint8_t *)&crc, 1);
						while (huart2.gState != HAL_UART_STATE_READY);//Loop until the end of transmission
        }
    }
#ifdef PROTOCOL_DEBUG
    GIZWITS_LOG("MCU2WiFi[%4d:%4d]: ", gizGetTimerCount(), len);
    for(i=0; i<len; i++)
    {
        GIZWITS_LOG("%02x ", buf[i]);

        if(i >=2 && buf[i] == 0xFF)
        {
            GIZWITS_LOG("%02x ", 0x55);
        }
    }
    GIZWITS_LOG("\n");
#endif
		
		return len;
}  
