//
// Created by 14419 on 25-9-16.
//
#include "dr_encoder.h"

void encoder_init(void) {

    /*  MA Reset counter */
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    /* Start TIM2 in encoder mode */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    // MB
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    ENCODER_A_SetCounter(30000);
    ENCODER_B_SetCounter(30000);

}

uint16_t ENCODER_A_GetCounter(void) {
    return (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
}

uint16_t ENCODER_B_GetCounter(void) {
    return (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
}

void ENCODER_A_SetCounter(const uint16_t count) {
    __HAL_TIM_SET_COUNTER(&htim2, count);
}

void ENCODER_B_SetCounter(const uint16_t count) {
    __HAL_TIM_SET_COUNTER(&htim3, count);
}
