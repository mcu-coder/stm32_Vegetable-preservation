

#include "motor.h"

// 步进电机相位表（四相八拍）
const uint8_t phase_table[8] = {
    0x09, // 1001 (A+D)
    0x08, // 1000 (A)
    0x0C, // 1100 (A+B)
    0x04, // 0100 (B)
    0x06, // 0110 (B+C)
    0x02, // 0010 (C)
    0x03, // 0011 (C+D)
    0x01  // 0001 (D)
};


// 设置电机相位（适配新引脚）
void Set_Motor_Phase(uint8_t phase) {
    HAL_GPIO_WritePin(GPIOA, MOTOR_A_Pin, (phase & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET); // A
    HAL_GPIO_WritePin(GPIOA, MOTOR_B_Pin, (phase & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET); // B
    HAL_GPIO_WritePin(GPIOB, MOTOR_C_Pin, (phase & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET); // C
    HAL_GPIO_WritePin(GPIOA, MOTOR_D_Pin, (phase & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET); // D
}

// 步进电机旋转（方向+步数）
void Stepper_Move(uint8_t dir, uint32_t steps) {
    static int8_t current_phase = 0;
    for (uint32_t i = 0; i < steps; i++) {
        if (dir) { // 正转
            current_phase = (current_phase + 1) % 8;
        } else {   // 反转
            current_phase = (current_phase - 1 + 8) % 8;
        }
        Set_Motor_Phase(phase_table[current_phase]);
        HAL_Delay(1); // 控制速度（延迟越小，转速越快）
    }
    // 停止时断电保护
    Set_Motor_Phase(0x00);
}