/**
************************************************************
* @file         hal_key.h
* @brief        hal_key.c对应头文件
* @author       Gizwits
* @date         2022-09-22
* @version      V03010101
* @copyright    Gizwits
* 
* @note         机智云.只为智能硬件而生
*               Gizwits Smart Cloud  for Smart Products
*               链接|增值ֵ|开放|中立|安全|自有|自由|生态
*               www.gizwits.com
*
***********************************************************/
#ifndef _HAL_KEY_H
#define _HAL_KEY_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stm32f1xx.h>


#define KEY_TIMER_MS                            1
#define KEY_MAX_NUMBER                          12
#define DEBOUNCE_TIME                           30
#define KEY_INTERVAL                            130
#define PRESS_LONG_TIME                         1000
#define PRESS_LAST_TIME                         5000

#define CLICK                 1                //Key click
#define DbLCLICK              2                //Key double click

//Key Registry
typedef enum  
{
    KEY1,                     // KEY Name:KEY1
    KEY2,                     // KEY Name:KEY2
    /*
    ...
    Users can add keys as required
    ...
    */
    KEY_NUM,                  // Number of keys,Must be placed at the bottom of the registry
}keyList;

typedef enum
{ 
    Bit_RESET = 0,
    Bit_SET
}BitAction_t;

/*Key status enumeration*/
typedef enum
{
    KEY_NULL,             // Key no action
    KEY_SURE,             // Key quaking elimination
    KEY_UP,               // Key release
    KEY_DOWN,             // Key down
    KEY_LONG,             // Key long
}keyStatus_t;

/*Key event enumeration*/
typedef enum 
{
	  NULL_Event,     // Empty event
    DOWN_Event,     // Down event
	  SHORT_Event,    // Short event
	  LONG_Event,     // Long event
    LAST_Event,     // Continuous Events
    DBCL_Event      // Double click event
}keyEvent_t;

__packed typedef struct
{
    GPIO_TypeDef     *keyPort;
    uint32_t         keyPin;
}keyGpio_t;

/*Key state machine structure*/
__packed typedef struct
{
    bool keyFlag;                 //Key count flag bit
    uint8_t keyFrequency;         //Number of key presses
    uint16_t keyCount;            //Long press key counting
    uint32_t keyInterval;         //Key interval time
    FunctionalState keyShield;    //Key enabling position
    keyGpio_t keyGpio;            //Key GPIO
    BitAction_t keyLevel;         //Judge whether the key is pressed, press: 1, lift: 0
    BitAction_t keyDownLevel;     //Level state of IO port when the key is pressed
    keyStatus_t keyStatus;        //Current state of the key
    keyEvent_t eventType;         //Key Event Type
}keyFSM_t;

/*Callback functions corresponding to different events*/
__packed typedef struct
{
    void (*nullPressCb)(void);
    void (*downPressCb)(void);
    void (*ShortPressCb)(void);
    void (*longPressCb)(void);
    void (*lastPressCb)(void);
    void (*dbclPressCb)(void);
} keyFunc_t;

/*Key class*/
__packed typedef struct
{
    keyFSM_t  fsm;
    keyFunc_t func;
}keyCategory_t;

void keyParaInit(keyCategory_t *keys);
void keyHandle(void);

#endif /*_HAL_KEY_H*/

