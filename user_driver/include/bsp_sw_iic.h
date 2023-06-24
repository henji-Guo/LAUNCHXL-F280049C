/*
 * bsp_sw_iic.h
 *
 *  Created on: 2023年6月23日
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
 * 输出操作
  */
#define SCL(n)      (n==1?GPIO_writePin(SCL_PIN,1):GPIO_writePin(SCL_PIN,0))
#define SDA(n)      (n==1?GPIO_writePin(SDA_PIN,1):GPIO_writePin(SDA_PIN,0))

/**
 * SDA
 * 读取操作
 **/
#define SDA_Read()  GPIO_readPin(SDA_PIN)

/**
 * ACK 回应状态
 **/
#define ACK_OK 0
#define ACK_ERROR 1

void bsp_iic_init();                        //IIC初始化
void bsp_iic_start();                       //IIC起始信号
void bsp_iic_stop();                        //IIC停止信号
uint8_t bsp_iic_wait_ack();                //IIC主机等待从机响应ACK
void bsp_iic_ack();                         //IIC主机向从机发送ACK
void bsp_iic_nack();                        //IIC主机向从机发送NOACK
void bsp_iic_sendonebyte(uint8_t SendData); //IIC主机发送
uint8_t bsp_iic_readonebyte();              //IIC从机接收





#endif /* USER_DRIVER_INCLUDE_BSP_SW_bsp_iic_H_ */
