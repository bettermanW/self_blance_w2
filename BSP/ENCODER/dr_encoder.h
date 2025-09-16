//
// Created by 14419 on 25-9-16.
//

#ifndef DR_ENCODER_H
#define DR_ENCODER_H

#include "common.h"

//接口函数
void encoder_init(void);          //编码器初始化
uint16_t ENCODER_A_GetCounter(void);          //编码器获取计数器数值
uint16_t ENCODER_B_GetCounter(void);          //编码器获取计数器数值


void ENCODER_A_SetCounter(uint16_t count);    //编码器设置计数器数值
void ENCODER_B_SetCounter(uint16_t count);    //编码器设置计数器数值
#endif //DR_ENCODER_H
