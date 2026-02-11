#ifndef __MOTOR_H_
#define __MOTOR_H_


#include  "main.h"
#include "stm32f1xx_hal.h"

void Set_Motor_Phase(uint8_t phase) ;
void Stepper_Move(uint8_t dir, uint32_t steps);

#endif



