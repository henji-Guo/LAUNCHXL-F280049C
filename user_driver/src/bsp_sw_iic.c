/*
 * bsp_sw_iic.c
 *
 *  Created on: 2023年6月23日
 *      Author: GHJ
 */
#include "bsp_sw_iic.h"

void SDA_OUT();      //SDA输出模式
void SDA_INPUT();    //SDA输入模式
void bsp_iic_timeout();  //IIC 超时
void bsp_iic_delay(); //IIC delay

/* IIC GPIO PIN 初始化 */
void bsp_iic_init()
{


    /* 初始化 SCL */
    GPIO_setPinConfig(GPIO_37_GPIO37);
    GPIO_setPadConfig(SCL_PIN, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(SCL_PIN, GPIO_DIR_MODE_OUT);
    /* 初始化 SDA */
    SDA_OUT();
}

/**
 * IIC起始信号
 * SCL 高电平
 * SDA 高->低
 **/
void bsp_iic_start()
{
    SDA_OUT(); // SDA输出模式
    bsp_iic_delay();
    /* 拉低时钟和总线 */
    SCL(0);
    SDA(0);
    bsp_iic_delay();
    /* 拉高总线 和 时钟 */
    SDA(1);
    SCL(1);
    bsp_iic_delay();
    /* 拉低总线 */
    SDA(0);
    bsp_iic_delay();
    /* 拉低时钟 准备发送数据 */
    SCL(0);
}

/**
 * IIC停止信号
 * SCL 高电平
 * SDA 低->高
 **/
void bsp_iic_stop()
{
    /* 先拉低时钟,SDA输出模式 */
    SCL(0);
    SDA_OUT();
    bsp_iic_delay();
    /* 拉低时钟和总线 */
    SCL(0);
    SDA(0);
    bsp_iic_delay();
    /* 先拉高时钟 */
    SCL(1);
    bsp_iic_delay();
    /* 拉高总线 等待停止ACK*/
    SDA(1);
    bsp_iic_delay();
    SCL(0);
}

/**
 * 主机发送命令等待从机ACK回应
 * SCL 高电平
 * SDA输入模式,等待SDA响应
 * (SDA 高无效，低有效)
 **/
uint8_t bsp_iic_wait_ack()
{
    /* 先拉低时钟,SDA改为输入模式 */
    SCL(0);
    SDA_INPUT();
    bsp_iic_delay();
    /* 拉高SCL,准备接收数据 */
    SCL(1);
    /* 等待从机响应 */
    uint8_t timeout = 0;
    while (SDA_Read())
    {
        if (timeout > 10)
        {
            return ACK_ERROR;
        }
        timeout++;
        bsp_iic_timeout();
    }
    SCL(0);
    return ACK_OK;
}

/**
 * 主机接收从机数据,回应从机ACK
 * SCL 高电平
 * SDA输出模式(SDA 高无效，低有效)
 **/
void bsp_iic_ack()
{
    /* 先拉低时钟,SDA输出模式 */
    SCL(0);
    SDA_OUT();
    /* 准备ACK 拉低SDA */
    SDA(0);
    bsp_iic_delay();
    /* 拉高时钟 */
    SCL(1);
    bsp_iic_delay();
    /* 再拉低时钟,结束第九脉冲 */
    SCL(0);
    bsp_iic_delay();
}

/**
 * 主机接收从机数据,回应从机NOACK
 * SCL 高电平
 * SDA输出模式(SDA 高无效，低有效)
 **/
void bsp_iic_nack()
{
    /* 先拉低时钟,SDA输出模式 */
    SCL(0);
    SDA_OUT();
    /* 准备ACK 拉高SDA */
    SDA(1);
    bsp_iic_delay();
    /* 拉高时钟 */
    SCL(1);
    bsp_iic_delay();
    /* 再拉低时钟,结束第九脉冲 */
    SCL(0);
    bsp_iic_delay();
}

/**
 * 主机发送一字节的数据
 * SCL 高电平 SDA输出模式(SDA 高为1，低为0)
 * SCL 低电平 准备数据
 **/
void bsp_iic_sendonebyte(uint8_t SendData)
{
    /* 拉低时钟,SDA输出模式 */
    SCL(0);
    SDA_OUT();
    bsp_iic_delay();
    /* 发送数据 */
    for (int i = 0; i < 8; i++)
    {
        /* 准备数据,从最高位发送 */
        if((SendData & 0x80)>>7)
            SDA(1);
        else
            SDA(0);
        SendData <<= 1;
        /* 拉高时钟,发送数据 */
        SCL(1);
        bsp_iic_delay();
        /* 拉低时钟,准备数据 */
        SCL(0);

    }
    /* 拉低时钟,准备接收ACK*/
    SCL(0);
}

/**
 * 主机接收一字节的数据
 * SCL 高电平 SDA输出模式(SDA 高为1，低为0)
 **/
uint8_t bsp_iic_readonebyte()
{
    uint8_t RecData = 0;
    /* 拉低时钟,SDA输入模式 */
    SCL(0);
    SDA_INPUT();
    bsp_iic_delay();
    /* 接收数据 */
    for (int i = 0; i < 8; i++)
    {
        /* 拉低时钟,准备接收下一位 */
        SCL(0);
        bsp_iic_delay();
        /* 拉高时钟,从最高位接收 */
        SCL(1);
        /* 空出最低位,用来接收数据 */
        RecData <<= 1;
        if (SDA_Read())
            RecData |= 0x01;
        bsp_iic_delay();

    }
    return RecData;
}

//SDA输出模式
void SDA_OUT()
{
    /* 初始化 SDA */
    GPIO_setPinConfig(GPIO_35_GPIO35);
    GPIO_setPadConfig(SDA_PIN, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(SDA_PIN, GPIO_DIR_MODE_OUT);
}

//SDA输入模式
void SDA_INPUT()
{
    /* 初始化 SDA  */
    GPIO_setPinConfig(GPIO_35_GPIO35);
    GPIO_setPadConfig(SDA_PIN, GPIO_PIN_TYPE_PULLUP);
    GPIO_setDirectionMode(SDA_PIN, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(SDA_PIN, GPIO_QUAL_SYNC);
}

void bsp_iic_delay()
{
    SysCtl_delay(50);
}

void bsp_iic_timeout()
{
    DEVICE_DELAY_US(2);
}


