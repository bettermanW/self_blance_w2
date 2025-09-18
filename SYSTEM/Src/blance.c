//
// Created by 14419 on 25-9-17.
//
#include "blance.h"
#include <math.h>
volatile bool mpu6050_data_ready = false;
float gyro_dps[3], accel_g[3], mpu_temperature;
int16_t gyro_raw[3], accel_raw[3];




void OLED_ShowGyroData(float gyro_dps[3], float accel_g[3], float temperature) {
    OLED_NewFrame();
    char buff[30];


    // 显示温度
    sprintf(buff, "Temp: %.1f C", temperature);
    OLED_PrintASCIIString(0, 0, buff, &afont8x6, 0);

    OLED_DrawLine(0, 8, 127, 18, 1);

    // 显示陀螺仪数据（角速度）
    OLED_PrintASCIIString(0, 10, "Gyro (dps):", &afont8x6, 0);
    sprintf(buff, "X:%7.2f", gyro_dps[0]);
    OLED_PrintASCIIString(65, 10, buff, &afont8x6, 0);
    sprintf(buff, "Y:%7.2f", gyro_dps[1]);
    OLED_PrintASCIIString(65, 20, buff, &afont8x6, 0);
    sprintf(buff, "Z:%7.2f", gyro_dps[2]);
    OLED_PrintASCIIString(65, 30, buff, &afont8x6, 0);

    OLED_DrawLine(0, 40, 127, 50, 1);

    // 显示加速度计数据
    OLED_PrintASCIIString(0, 40, "Accel (g):", &afont8x6, 0);
    sprintf(buff, "X:%6.3f", accel_g[0]);
    OLED_PrintASCIIString(65, 40, buff, &afont8x6, 0);
    sprintf(buff, "Y:%6.3f", accel_g[1]);
    OLED_PrintASCIIString(65, 48, buff, &afont8x6, 0);
    sprintf(buff, "Z:%6.3f", accel_g[2]);
    OLED_PrintASCIIString(65, 56, buff, &afont8x6, 0);

    OLED_ShowFrame();
}

void mpu_6050_data_ready() {

    if (mpu6050_data_ready) {
        // 重置标志位
        mpu6050_data_ready = false;

        // 在OLED上显示数据
        OLED_ShowGyroData(gyro_dps, accel_g,mpu_temperature);


    }

    // 这里可以添加其他任务
    // 例如：按键检测、状态机处理等

    // 短暂延时，避免CPU占用率100%
    HAL_Delay(10);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == interrupt_6050_Pin) {
        // 读取温度值
        mpu_temperature = AX_MPU6050_GetTempValue();

        // 读取陀螺仪原始数据
        AX_MPU6050_GetGyroData(gyro_raw);

        // 读取加速度计原始数据
        AX_MPU6050_GetAccData(accel_raw);


        // 转换为角速度（2000dps量程）
        for (int i = 0; i < 3; i++) {

            gyro_dps[i] = (float)gyro_raw[i] / 16.384f;
        }

        // 转换为加速度（±2g量程）
        for (int i = 0; i < 3; i++) {
            accel_g[i] = (float)accel_raw[i] / 16384.0f;
        }

        // 设置数据就绪标志
        mpu6050_data_ready = true;

        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
    }
}



