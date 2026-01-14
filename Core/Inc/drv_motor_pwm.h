#ifndef DRV_MOTOR_PWM_H
#define DRV_MOTOR_PWM_H

#include "main.h"

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
} MotorPWM_Handle_t;

void Drv_Motor_Init(MotorPWM_Handle_t *h_motor, TIM_HandleTypeDef *htim, uint32_t channel);
void Drv_Motor_SetRaw(MotorPWM_Handle_t *h_motor, uint32_t raw_pwm_value);
void Drv_Motor_Stop(MotorPWM_Handle_t *h_motor);

#endif // DRV_MOTOR_PWM_H
