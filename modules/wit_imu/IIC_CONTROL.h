#ifndef _IIC_CONTROL_H
#define _IIC_CONTROL_H
#include <stdint.h>
#include "main.h"
// #include "sys.h"
//IO方向设置
#define SDA_IN()  (GPIOF->MODER &= ~(3 << (0 * 2)))  // PF0 配置为输入模式

#define SDA_OUT() (GPIOF->MODER = (GPIOF->MODER & ~(3 << (0 * 2))) | (1 << (0 * 2)))  // PF0 配置为输出模式

// #define SDA_IN()  {GPIOC->CRH&=0XFFFF0FFF;GPIOC->CRH|=(u32)8<<12;}	//PC12输入模式
// #define SDA_OUT() {GPIOC->CRH&=0XFFFF0FFF;GPIOC->CRH|=(u32)3<<12;} 	//PC12输出模式

//IO操作
#define BITBAND_ADDR(addr, bitnum) ((volatile uint32_t *)(0x42000000 + ((addr - 0x40000000) * 32) + (bitnum * 4)))

#define GPIOF_ODR_Addr (GPIOF_BASE + 0x0C)  // GPIOF ODR 寄存器地址
#define IIC_SDA        (*BITBAND_ADDR(GPIOF_ODR_Addr, 0))  // PF0 位带映射
#define IIC_SCL        (*BITBAND_ADDR(GPIOF_ODR_Addr, 1)) 
#define READ_SDA ((GPIOF->IDR & (1 << 0)) ? 1 : 0)  // 读取 PF0 的电平状态
// #define IIC_SCL   *(volatile unsigned int *)((GPIOF_BASE + 0x0C) + (1 << 1))
// #define IIC_SDA   BIT_ADDR(GPIOF_ODR_Addr,0) 
// #define IIC_SCL   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET)//PCout(12) //SCL
// #define IIC_SDA   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET)//PCout(11) //SDA
// #define READ_SDA  PCin(11)  //输入SDA

#define u8 uint8_t  
//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	 

int32_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length);
int32_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t* data, uint32_t length);

#endif



