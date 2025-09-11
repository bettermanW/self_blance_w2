#include "soft_i2c.h"

// /**
//  * @brief  初始化IIC的GPIO
//  * @note   PB10, PB11 设置为开漏输出，并默认拉高
//  */
// void IIC_Init(void)
// {
//     GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//     /* 使能GPIOB时钟 */
//     __HAL_RCC_GPIOB_CLK_ENABLE();
//
//     /* 配置SCL (PB10) */
//     GPIO_InitStruct.Pin = SOFT_I2C_SCL_PIN;
//     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;  // 开漏输出模式[2,5](@ref)
//     GPIO_InitStruct.Pull = GPIO_PULLUP;         // 上拉[2,3](@ref)
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//     HAL_GPIO_Init(SOFT_I2C_SCL_GPIO_PORT, &GPIO_InitStruct);
//
//     /* 配置SDA (PB11) */
//     GPIO_InitStruct.Pin = SOFT_I2C_SDA_PIN;
//     HAL_GPIO_Init(SOFT_I2C_SDA_GPIO_PORT, &GPIO_InitStruct);
//
//     /* 初始状态拉高SDA和SCL */
//     IIC_SCL_Hi;
//     IIC_SDA_Hi;
// }

/**
 * @brief  产生IIC起始信号
 * @note   当SCL为高电平时，SDA由高变低
 */
void IIC_Start(void)
{
    IIC_SDA_Hi;
    IIC_SCL_Hi;
     us_delay(5);  // 短暂延 us_delay(5)时，可根据实际时序要求调整
    IIC_SDA_Low;   // START: SCL高电平期间，SDA从高变低[3,6](@ref)
     us_delay(5);
    IIC_SCL_Low;   // 钳住I2C总线，准备发送或接收数据[6](@ref)
}

/**
 * @brief  产生IIC停止信号
 * @note   当SCL为高电平时，SDA由低变高
 */
void IIC_Stop(void)
{
    IIC_SDA_Low;
    IIC_SCL_Low;
     us_delay(5);
    IIC_SCL_Hi;
     us_delay(5);
    IIC_SDA_Hi;    // STOP: SCL高电平期间，SDA从低变高[3,6](@ref)
     us_delay(5);
}

/**
 * @brief  等待应答信号
 * @retval 0: 收到ACK
 *         1: 收到NACK
 */
uint8_t IIC_WaitAck(void)
{
    uint8_t wait_time = 0;
    
    IIC_SDA_Hi;  // 主机释放SDA线[6,7](@ref)
    IIC_SCL_Hi;
     us_delay(5);
    
    while(IIC_Read_SDA == GPIO_PIN_SET)  // 等待SDA被从机拉低（ACK）[6](@ref)
    {
        wait_time++;
        if(wait_time > 250)
        {
            IIC_Stop();
            return 1;  // 超时，返回NACK
        }
         us_delay(5);
    }
    
    IIC_SCL_Low;
     us_delay(5);
    
    return 0;  // 收到ACK
}

/**
 * @brief  发送ACK应答
 */
void IIC_SendAck(void)
{
    IIC_SDA_Low;  // SDA拉低表示ACK[6,7](@ref)
    IIC_SCL_Hi;
     us_delay(5);
    IIC_SCL_Low;
     us_delay(5);
}

/**
 * @brief  发送NACK非应答
 */
void IIC_SendNAck(void)
{
    IIC_SDA_Hi;  // SDA保持高电平表示NACK[6,7](@ref)
    IIC_SCL_Hi;
     us_delay(5);
    IIC_SCL_Low;
     us_delay(5);
}

/**
 * @brief  IIC发送一个字节
 * @param  data: 要发送的字节
 */
void IIC_SendByte(uint8_t data)
{
    for(uint8_t i = 0; i < 8; i++)
    {
        IIC_SCL_Low;
         us_delay(5);
        
        /* 准备数据位 */
        if(data & 0x80)
            IIC_SDA_Hi;
        else
            IIC_SDA_Low;
        
        data <<= 1;
         us_delay(5);
        
        IIC_SCL_Hi;  // 拉高SCL，从机采样数据位[3](@ref)
         us_delay(5);
    }
    
    IIC_SCL_Low;
}

/**
 * @brief  IIC读取一个字节
 * @param  ack: 是否发送ACK (0:发送ACK, 1:发送NACK)
 * @retval 读取到的字节
 */
uint8_t IIC_ReadByte(uint8_t ack)
{
    uint8_t receive = 0;
    
    IIC_SDA_Hi;  // 主机释放SDA线
    
    for(uint8_t i = 0; i < 8; i++)
    {
        receive <<= 1;
        IIC_SCL_Low;
         us_delay(5);
        IIC_SCL_Hi;
         us_delay(5);
        
        if(IIC_Read_SDA)
            receive |= 0x01;
    }
    
    if(ack)
        IIC_SendAck();
    else
        IIC_SendNAck();
    
    return receive;
}

/**
  * @brief  模拟HAL_I2C_Master_Transmit函数，通过软件IIC发送数据
  * @param  DevAddress: 目标从设备地址（7位地址，函数内部会左移1位并添加写标志）
  * @param  pData: 指向要发送数据缓冲区的指针
  * @param  Size: 要发送的数据字节数
  * @retval HAL_StatusTypeDef: 传输状态，HAL_OK 成功，HAL_ERROR 失败
  */
HAL_StatusTypeDef MY_I2C_Master_Transmit(uint16_t DevAddress, const uint8_t *pData, uint16_t Size)
{
    /* 1. 发送起始信号 */
    IIC_Start();

    /* 2. 发送设备地址 + 写标志 (Bit0为0表示写) */
    IIC_SendByte((uint8_t)DevAddress & 0xFE); // 确保最低位是0（写操作）
    if (IIC_WaitAck() != 0) // 等待从机应答
    {
        IIC_Stop();
        return HAL_ERROR; // 从机无应答，返回错误
    }

    /* 3. 循环发送数据 */
    while (Size--)
    {
        IIC_SendByte(*pData++); // 发送一个字节数据
        if (IIC_WaitAck() != 0) // 等待从机应答
        {
            IIC_Stop();
            return HAL_ERROR; // 从机无应答，返回错误
        }
    }

    /* 4. 发送停止信号 */
    IIC_Stop();

    return HAL_OK; // 传输成功
}

void I2C_Scan(void)
{
    for(uint8_t addr = 1; addr < 127; addr++)
    {
        IIC_Start();
        IIC_SendByte(addr << 1);
        if(IIC_WaitAck() == 0)
        {
            printf("Device found at: 0x%02X\n", addr);
        }
        IIC_Stop();
        HAL_Delay(100);
    }
}

/**
 * @brief  微秒级延时函数
 * @param  us: 延时的微秒数
 */
void us_delay(uint32_t us)
{
    uint32_t ticks = us * (SystemCoreClock / 1000000) / 20;
    for(uint32_t i = 0; i < ticks; i++)
    {
        __NOP();
    }
}