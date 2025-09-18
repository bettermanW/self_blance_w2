#ifndef __SOFT_I2C_H
#define __SOFT_I2C_H

#include "common.h"

/* 引脚定义 (根据你的硬件连接修改) */
#define SOFT_I2C_SCL_GPIO_PORT    SOFT_6050_SCL_GPIO_Port
#define SOFT_I2C_SCL_PIN          SOFT_6050_SCL_Pin
#define SOFT_I2C_SDA_GPIO_PORT    SOFT_6050_SDA_GPIO_Port
#define SOFT_I2C_SDA_PIN          SOFT_6050_SDA_Pin

/* 电平操作宏 */
#define IIC_SCL_Hi    HAL_GPIO_WritePin(SOFT_I2C_SCL_GPIO_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_SET)
#define IIC_SCL_Low   HAL_GPIO_WritePin(SOFT_I2C_SCL_GPIO_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_RESET)
#define IIC_SDA_Hi    HAL_GPIO_WritePin(SOFT_I2C_SDA_GPIO_PORT, SOFT_I2C_SDA_PIN, GPIO_PIN_SET)
#define IIC_SDA_Low   HAL_GPIO_WritePin(SOFT_I2C_SDA_GPIO_PORT, SOFT_I2C_SDA_PIN, GPIO_PIN_RESET)
#define IIC_Read_SDA HAL_GPIO_ReadPin(SOFT_I2C_SDA_GPIO_PORT, SOFT_I2C_SDA_PIN)

/* 函数声明 */
void IIC_Init(void);
void IIC_Start(void);
void IIC_Stop(void);
uint8_t IIC_WaitAck(void);
void IIC_SendAck(void);
void IIC_SendNAck(void);
void IIC_SendByte(uint8_t data);
uint8_t IIC_ReadByte(uint8_t ack);

HAL_StatusTypeDef MY_I2C_Master_Transmit(uint16_t DevAddress, const uint8_t *pData, uint16_t Size);
void I2C_Scan(void);
void us_delay(uint32_t us);
#endif