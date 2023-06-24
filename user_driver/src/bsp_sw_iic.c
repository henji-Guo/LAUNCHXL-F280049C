/*
 * bsp_sw_iic.c
 *
 *  Created on: 2023��6��23��
 *      Author: GHJ
 */
#include "bsp_sw_iic.h"

void SDA_OUT();      //SDA���ģʽ
void SDA_INPUT();    //SDA����ģʽ
void bsp_iic_timeout();  //IIC ��ʱ
void bsp_iic_delay(); //IIC delay

/* IIC GPIO PIN ��ʼ�� */
void bsp_iic_init()
{


    /* ��ʼ�� SCL */
    GPIO_setPinConfig(GPIO_37_GPIO37);
    GPIO_setPadConfig(SCL_PIN, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(SCL_PIN, GPIO_DIR_MODE_OUT);
    /* ��ʼ�� SDA */
    SDA_OUT();
}

/**
 * IIC��ʼ�ź�
 * SCL �ߵ�ƽ
 * SDA ��->��
 **/
void bsp_iic_start()
{
    SDA_OUT(); // SDA���ģʽ
    bsp_iic_delay();
    /* ����ʱ�Ӻ����� */
    SCL(0);
    SDA(0);
    bsp_iic_delay();
    /* �������� �� ʱ�� */
    SDA(1);
    SCL(1);
    bsp_iic_delay();
    /* �������� */
    SDA(0);
    bsp_iic_delay();
    /* ����ʱ�� ׼���������� */
    SCL(0);
}

/**
 * IICֹͣ�ź�
 * SCL �ߵ�ƽ
 * SDA ��->��
 **/
void bsp_iic_stop()
{
    /* ������ʱ��,SDA���ģʽ */
    SCL(0);
    SDA_OUT();
    bsp_iic_delay();
    /* ����ʱ�Ӻ����� */
    SCL(0);
    SDA(0);
    bsp_iic_delay();
    /* ������ʱ�� */
    SCL(1);
    bsp_iic_delay();
    /* �������� �ȴ�ֹͣACK*/
    SDA(1);
    bsp_iic_delay();
    SCL(0);
}

/**
 * ������������ȴ��ӻ�ACK��Ӧ
 * SCL �ߵ�ƽ
 * SDA����ģʽ,�ȴ�SDA��Ӧ
 * (SDA ����Ч������Ч)
 **/
uint8_t bsp_iic_wait_ack()
{
    /* ������ʱ��,SDA��Ϊ����ģʽ */
    SCL(0);
    SDA_INPUT();
    bsp_iic_delay();
    /* ����SCL,׼���������� */
    SCL(1);
    /* �ȴ��ӻ���Ӧ */
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
 * �������մӻ�����,��Ӧ�ӻ�ACK
 * SCL �ߵ�ƽ
 * SDA���ģʽ(SDA ����Ч������Ч)
 **/
void bsp_iic_ack()
{
    /* ������ʱ��,SDA���ģʽ */
    SCL(0);
    SDA_OUT();
    /* ׼��ACK ����SDA */
    SDA(0);
    bsp_iic_delay();
    /* ����ʱ�� */
    SCL(1);
    bsp_iic_delay();
    /* ������ʱ��,�����ھ����� */
    SCL(0);
    bsp_iic_delay();
}

/**
 * �������մӻ�����,��Ӧ�ӻ�NOACK
 * SCL �ߵ�ƽ
 * SDA���ģʽ(SDA ����Ч������Ч)
 **/
void bsp_iic_nack()
{
    /* ������ʱ��,SDA���ģʽ */
    SCL(0);
    SDA_OUT();
    /* ׼��ACK ����SDA */
    SDA(1);
    bsp_iic_delay();
    /* ����ʱ�� */
    SCL(1);
    bsp_iic_delay();
    /* ������ʱ��,�����ھ����� */
    SCL(0);
    bsp_iic_delay();
}

/**
 * ��������һ�ֽڵ�����
 * SCL �ߵ�ƽ SDA���ģʽ(SDA ��Ϊ1����Ϊ0)
 * SCL �͵�ƽ ׼������
 **/
void bsp_iic_sendonebyte(uint8_t SendData)
{
    /* ����ʱ��,SDA���ģʽ */
    SCL(0);
    SDA_OUT();
    bsp_iic_delay();
    /* �������� */
    for (int i = 0; i < 8; i++)
    {
        /* ׼������,�����λ���� */
        if((SendData & 0x80)>>7)
            SDA(1);
        else
            SDA(0);
        SendData <<= 1;
        /* ����ʱ��,�������� */
        SCL(1);
        bsp_iic_delay();
        /* ����ʱ��,׼������ */
        SCL(0);

    }
    /* ����ʱ��,׼������ACK*/
    SCL(0);
}

/**
 * ��������һ�ֽڵ�����
 * SCL �ߵ�ƽ SDA���ģʽ(SDA ��Ϊ1����Ϊ0)
 **/
uint8_t bsp_iic_readonebyte()
{
    uint8_t RecData = 0;
    /* ����ʱ��,SDA����ģʽ */
    SCL(0);
    SDA_INPUT();
    bsp_iic_delay();
    /* �������� */
    for (int i = 0; i < 8; i++)
    {
        /* ����ʱ��,׼��������һλ */
        SCL(0);
        bsp_iic_delay();
        /* ����ʱ��,�����λ���� */
        SCL(1);
        /* �ճ����λ,������������ */
        RecData <<= 1;
        if (SDA_Read())
            RecData |= 0x01;
        bsp_iic_delay();

    }
    return RecData;
}

//SDA���ģʽ
void SDA_OUT()
{
    /* ��ʼ�� SDA */
    GPIO_setPinConfig(GPIO_35_GPIO35);
    GPIO_setPadConfig(SDA_PIN, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(SDA_PIN, GPIO_DIR_MODE_OUT);
}

//SDA����ģʽ
void SDA_INPUT()
{
    /* ��ʼ�� SDA  */
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


