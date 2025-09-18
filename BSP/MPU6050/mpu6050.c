//
// Created by 14419 on 25-9-11.
//


#include "mpu6050.h"


static void MPU6050_WriteRegister(uint8_t reg_address, uint8_t data);
static void MPU6050_ReadRegister(uint8_t reg_address, uint8_t *pdata, uint16_t len);

/**
 * @brief 初始化MPU6050
 */
void MPU6050_Init(void) {
    // 1. 解除休眠并设置时钟源
    MPU6050_WriteRegister(MPU6050_PWR_MGMT1_REG, 0x80);
    HAL_Delay(50); // 等待传感器稳定

    MPU6050_WriteRegister(MPU6050_PWR_MGMT1_REG, 0x00); //  唤醒
    // 2. 配置陀螺仪量程 (根据应用调整)
    MPU6050_WriteRegister(MPU6050_GYRO_CFG_REG, 0x18); // 例如: ±2000°/s

    // 3. 配置加速度计量程
    MPU6050_WriteRegister(MPU6050_ACCEL_CFG_REG, 0x00); // ±2g

    // 4. 设置采样率分频器
    MPU6050_WriteRegister(MPU6050_SAMPLE_RATE_REG, 0x07); // 125Hz

    // 5. 配置数字低通滤波器DLPF (根据应用调整)
    MPU6050_WriteRegister(MPU6050_CFG_REG, 0x03); // 例如: 带宽44Hz (ACC) / 42Hz (GYRO)

    // 6. 中断配置
    MPU6050_WriteRegister(MPU6050_INT_EN_REG, 0x01);    // 开启Data Ready中断
    MPU6050_WriteRegister(MPU6050_INTBP_CFG_REG, 0x80); // INT引脚低电平有效

    // 7. 再次确认时钟源（可选，但确保设置）
    MPU6050_WriteRegister(MPU6050_PWR_MGMT1_REG, 0x01); // 确保使用PLL with X gyro ref
    // 8. 确保所有轴都使能
    MPU6050_WriteRegister(MPU6050_PWR_MGMT2_REG, 0x00); // 加速度和陀螺仪所有轴都工作
}


/**
  * @简  述  MPU6050设置加速度量程
  * @参  数  range： 加速度量程2g、4g、8g、16g
  *          可设置的加速度量程ACC_RANGE_2G、ACC_RANGE_4G、ACC_RANGE_8G、ACC_RANGE_16G
  * @返回值  无
  */
void AX_MPU6050_SetAccRange(uint8_t range)
{
    MPU6050_WriteRegister(MPU6050_ACCEL_CFG_REG,range<<3);
}

/**
  * @简  述  MPU6050设置陀螺仪量程
  * @参  数  range 陀螺仪量程250°/S、500°/S、1000°/S、2000°/S
  *          可设置的陀螺仪量程GYRO_RANGE_250、GYRO_RANGE_500、GYRO_RANGE_1000、GYRO_RANGE_2000
  * @返回值  无
  */
void AX_MPU6050_SetGyroRange(uint8_t range)
{
    MPU6050_WriteRegister(MPU6050_GYRO_CFG_REG,range<<3);
}

/**
  * @简  述  MPU6050设置陀螺仪采样率
  * @参  数  smplrate 陀螺仪采样率，范围10~1000Hz
  * @返回值  无
  */
void AX_MPU6050_SetGyroSmplRate(uint16_t smplrate)
{
    if(smplrate>1000)
        smplrate = 1000;
    if(smplrate<10)
        smplrate = 10;

    MPU6050_WriteRegister(MPU6050_SAMPLE_RATE_REG,(uint8_t)(1000/smplrate -1));
}

/**
  * @简  述  MPU6050设置低通滤波器带宽
  * @参  数  bandwidth 低通滤波器带宽
  *          可设置的带宽： DLPF_ACC184_GYRO188、DLPF_ACC94_GYRO98、DLPF_ACC44_GYRO42、
  *                        DLPF_ACC21_GYRO20、DLPF_ACC10_GYRO10、DLPF_ACC5_GYRO5
  * @返回值  无
  */
void AX_MPU6050_SetDLPF(uint8_t bandwidth)
{
    MPU6050_WriteRegister(MPU6050_CFG_REG,bandwidth);
}


/**********************************读取函数****************************************/

/**
 * @brief 获取温度值
 * @return 返回温度
 */
float AX_MPU6050_GetTempValue(void)
{
    uint8_t buf[2];  // 创建2字节缓冲区

    // 读取温度寄存器（MPU6050_TEMP_OUTH_REG）的2个字节数据
    MPU6050_ReadRegister(MPU6050_TEMP_OUTH_REG, buf, 2);

    // 将两个8位数据合并为一个16位有符号整数
    const uint16_t raw_temp = ((uint16_t)buf[0] << 8) | buf[1];
    const int16_t tmp = (int16_t)raw_temp;  // 明确的类型转换
    // const int16_t tmp = (buf[0] << 8) | buf[1];

    // 根据MPU6050手册的转换公式：T(°C) = TEMP_OUT / 340 + 36.53
    return 36.53f + (float)tmp / 340.0f;;
}

/**
  * @brief  MPU6050获取三轴加速度寄存器输出值
  * @参  数  pbuf：读取的数据缓冲区指针
  * @返回值  无
  */
// 加速度计 - 类型安全版本
void AX_MPU6050_GetAccData(int16_t *pbuf)
{
    uint8_t buf[6];
    MPU6050_ReadRegister(MPU6050_ACCEL_XOUTH_REG, buf, 6);

    pbuf[0] = (int16_t)((uint16_t)buf[0] << 8 | (uint16_t)buf[1]);
    pbuf[1] = (int16_t)((uint16_t)buf[2] << 8 | (uint16_t)buf[3]);
    pbuf[2] = (int16_t)((uint16_t)buf[4] << 8 | (uint16_t)buf[5]);
    printf("X: %04X, Y: %04X, Z: %4X\n",
           (uint16_t)pbuf[0], (uint16_t)pbuf[1], (uint16_t)pbuf[2]);
}

// 陀螺仪 - 保持原样
void AX_MPU6050_GetGyroData(int16_t *pbuf)
{
    uint8_t buf[6];
    MPU6050_ReadRegister(MPU6050_GYRO_XOUTH_REG, buf, 6);

    pbuf[0] = (buf[0] << 8) | buf[1];
    pbuf[1] = (buf[2] << 8) | buf[3];
    pbuf[2] = (buf[4] << 8) | buf[5];
}




/**********************************I2C操作函数*************************************/

/**
 * @brief 一个周期 根据MPU6050的时序图写数据
 * @param dev_addr 从机地址，这里是MPU6050
 * @param reg_addr 写哪一个寄存器
 * @param len 数据长度
 * @param data  写的数据
 * @return 返回是否成功
 */
uint8_t MPU6050_I2C_Write(uint8_t dev_addr, uint8_t reg_addr, const uint8_t len, const uint8_t *data)
{
    IIC_Start();
    IIC_SendByte(dev_addr << 1 ); // 左移动1位，前面7个是地址，后面0是方向
    // 是否等到从机回应
    if (IIC_WaitAck() == 1) {
        IIC_Stop(); // 没有直接停止
        return 1;
    }
    // 写寄存器
    IIC_SendByte(reg_addr);
    IIC_WaitAck();

    // 开始写数据
    for (int i = 0; i < len; i++) {
        IIC_SendByte(data[i]);
        if (IIC_WaitAck() == 1) {
            IIC_Stop();
            return 1;
        }
    }
    IIC_Stop();
    return 0;
}


/**
 * @brief 读取一个周期的数据
 * @param dev_addr MPU6050地址
 * @param reg_addr 读哪一个寄存器
 * @param len  数据长度
 * @param data 数据
 * @return
 */
uint8_t MPU6050_I2C_Read(uint8_t dev_addr, uint8_t reg_addr, uint8_t len, uint8_t *data)
{
    IIC_Start();

    // 1. 发送设备地址（写模式）
    IIC_SendByte(dev_addr << 1);
    if (IIC_WaitAck() == 1) {
        IIC_Stop();
        return 1; // 设备无响应
    }

    // 2. 发送要读取的寄存器地址
    IIC_SendByte(reg_addr);
    if (IIC_WaitAck() == 1) {
        IIC_Stop();
        return 2; // 寄存器地址错误
    }

    // 3. 发送重复起始条件（不是停止后再开始！）
    IIC_Start(); // 这是重复起始，不是新的起始

    // 4. 发送设备地址（读模式）
    IIC_SendByte((dev_addr << 1) | 0x01);
    if (IIC_WaitAck() == 1) {
        IIC_Stop();
        return 3; // 读模式切换失败
    }

    // 5. 读取数据
    while (len > 0) {
        if (len == 1) {
            *data = IIC_ReadByte(0); // 最后一个字节，发送NACK
        } else {
            *data = IIC_ReadByte(1); // 非最后一个字节，发送ACK
        }
        data++;
        len--;
    }

    IIC_Stop();
    return 0; // 成功
}

/**
 * @brief 写寄存器
 * @param reg_address
 * @param data 使用 &data是为了将单字节变量 data的地址传递给需要指针参数的
 */
static void MPU6050_WriteRegister(const uint8_t reg_address, const uint8_t data)
{
    MPU6050_I2C_Write(MPU6050_ADDR,reg_address,1,&data);
}

static void MPU6050_ReadRegister(const uint8_t reg_address, uint8_t *pdata, const uint16_t len)
{
    MPU6050_I2C_Read(MPU6050_ADDR,reg_address,len,pdata);
}


/***********************************测试******************************************/

// 添加一个函数来检查MPU6050中断状态
void MPU6050_CheckInterruptStatus(void) {
    printf("hello world\n");
    uint8_t int_status;
    MPU6050_ReadRegister(0x3A, &int_status, 1);
    printf("INT Status: 0x%02X\r\n", int_status);

    uint8_t int_enable;
    MPU6050_ReadRegister(MPU6050_INT_EN_REG, &int_enable, 1);
    printf("INT Enable: 0x%02X\r\n", int_enable);
}


// 添加一个简单的MPU6050检测函数
// 添加一个简单的MPU6050检测函数
uint8_t MPU6050_CheckConnection(void) {
    uint8_t buf[6];
    uint8_t whoami;
    // WHO_AM_I 寄存器地址 = 0x75
    const uint8_t ret = MPU6050_I2C_Read(MPU6050_ADDR, 0x75, 1, &whoami);

    if (ret != 0) {
        printf("❌ I2C通信失败! 错误码: %d\r\n", ret);
        return 0;
    }

    // WHO_AM_I 返回固定值 0x68（或者 0x69, 取决于 AD0 脚）
    if (whoami == 0x68 || whoami == 0x69) {
        printf("✅ MPU6050连接正常, WHO_AM_I: 0x%02X\r\n", whoami);
        return 1;
    }
    printf("❌ WHO_AM_I 错误! 读取到: 0x%02X\r\n", whoami);

    // const HAL_StatusTypeDef status = MPU6050_ReadRegister(MPU6050_ACCEL_XOUTH_REG, buf, 6);
    // if (status != HAL_OK) {
    //     printf("I2C Read Error: %d\n", status);
    // }

    return 0;
}





void AX_MPU6050_GetAccData_Safe(int16_t *pbuf)
{
    uint8_t buf[2];

    // 分别读取各轴
    MPU6050_ReadRegister(MPU6050_ACCEL_XOUTH_REG, buf, 2);
    pbuf[0] = (int16_t)((buf[0] << 8) | buf[1]);

    MPU6050_ReadRegister(MPU6050_ACCEL_YOUTH_REG, buf, 2);
    pbuf[1] = (int16_t)((buf[0] << 8) | buf[1]);

    MPU6050_ReadRegister(MPU6050_ACCEL_ZOUTH_REG, buf, 2);
    pbuf[2] = (int16_t)((buf[0] << 8) | buf[1]);



    // printf("X: %0d, Y: %0d, Z: %0d\n",
    //        (uint16_t)pbuf[0], (uint16_t)pbuf[1], (uint16_t)pbuf[2]);
    printf("X: %04X, Y: %04X, Z: %4X\n",
           (uint16_t)pbuf[0], (uint16_t)pbuf[1], (uint16_t)pbuf[2]);
}


// 陀螺仪 - 保持原样
void AX_MPU6050_GetGyroData_Safe(int16_t *pbuf)
{
    uint8_t buf[2];

    // 分别读取各轴
    MPU6050_ReadRegister(MPU6050_GYRO_XOUTH_REG, buf, 2);
    pbuf[0] = (int16_t)((buf[0] << 8) | buf[1]);

    MPU6050_ReadRegister(MPU6050_GYRO_YOUTH_REG, buf, 2);
    pbuf[1] = (int16_t)((buf[0] << 8) | buf[1]);

    MPU6050_ReadRegister(MPU6050_GYRO_ZOUTH_REG, buf, 2);
    pbuf[2] = (int16_t)((buf[0] << 8) | buf[1]);
}


