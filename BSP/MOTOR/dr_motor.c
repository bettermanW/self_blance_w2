//
// Created by 14419 on 25-9-16.
//
#include "dr_motor.h"

#include "tim.h"

void motor_init() {

    // HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    // HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

void AX_MOTOR_A_SetSpeed(const int16_t speed) {
    int16_t temp = speed;

    // 限幅
    if(temp > 3600) temp = 3600;
    if(temp < -3600) temp = -3600;

    if(temp > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 3600);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, (3600 - temp));
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 3600);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (3600 + temp));
    }
}

void AX_MOTOR_B_SetSpeed(const int16_t speed) {
    int16_t temp = speed;

    // 限幅
    if(temp > 3600) temp = 3600;
    if(temp < -3600) temp = -3600;

    if(temp > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 3600);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, (3600 - temp));
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 3600);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (3600 + temp));
    }
}