/*
 * bsp_sw_iic.h
 *
 *  Created on: 2023��6��23��
 *      Author: GHJ
 */

#ifndef USER_DRIVER_INCLUDE_BSP_SW_bsp_iic_H_
#define USER_DRIVER_INCLUDE_BSP_SW_bsp_iic_H_

#include "device.h"
#include "f28004x_device.h"

/**
 *  SDA   GPIO35
 *  SCL   GPIO37
  */

#define SDA_PIN 35
#define SCL_PIN 37

/**
 * SDA SCL
 * �������
  */
#define SCL(n)      (n==1?GPIO_writePin(SCL_PIN,1):GPIO_writePin(SCL_PIN,0))
#define SDA(n)      (n==1?GPIO_writePin(SDA_PIN,1):GPIO_writePin(SDA_PIN,0))

/**
 * SDA
 * ��ȡ����
 **/
#define SDA_Read()  GPIO_readPin(SDA_PIN)

/**
 * ACK ��Ӧ״̬
 **/
#define ACK_OK 0
#define ACK_ERROR 1

void bsp_iic_init();                        //IIC��ʼ��
void bsp_iic_start();                       //IIC��ʼ�ź�
void bsp_iic_stop();                        //IICֹͣ�ź�
uint8_t bsp_iic_wait_ack();                //IIC�����ȴ��ӻ���ӦACK
void bsp_iic_ack();                         //IIC������ӻ�����ACK
void bsp_iic_nack();                        //IIC������ӻ�����NOACK
void bsp_iic_sendonebyte(uint8_t SendData); //IIC��������
uint8_t bsp_iic_readonebyte();              //IIC�ӻ�����





#endif /* USER_DRIVER_INCLUDE_BSP_SW_bsp_iic_H_ */
