#include "soft_i2c.h"
#include "stm32f0xx.h"
#include "main.h"

void softiic_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_OD;
	GPIO_InitStruct.GPIO_Pin= GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void SCL(uint8_t BitVal)
{
	GPIO_WriteBit(GPIOA, GPIO_Pin_9, (BitAction)BitVal);
	ms(1);
}

void SDA(uint8_t BitVal)
{
	GPIO_WriteBit(GPIOA, GPIO_Pin_10, (BitAction)BitVal);
	ms(1);
}

uint8_t SCL_IN(void)
{
	return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_9);
}

uint8_t SDA_IN(void)
{
	return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_10);
}

void sendstart(void)
{
	SCL(0);
	SDA(1);
	SCL(1);
	SDA(0);
	SCL(0);
}

void sendstop(void)
{
	SCL(0);
	SDA(0);
	SCL(1);
	SDA(1);
	SCL(0);
}

uint8_t receive(uint8_t ack)
{
uint8_t x, d=0;
  SDA(1); 
  for(x=0; x<8; x++) {
    d <<= 1;
    SCL(1);
    if(SDA_IN()) d |= 1;
    SCL(0);
  } 
  if(ack) SDA(0);
  else SDA(1);
  SCL(1);
  SCL(0);
  SDA(1);
  return d;
}

uint8_t transmit(uint8_t d)
{
uint8_t x, b;
  for(x=8; x; x--) {
    if(d&0x80) SDA(1);
    else SDA(0);
    SCL(1);
    d <<= 1;
    SCL(0);
  }
  SDA(1);
  SCL(1);
  b = SDA_IN();          // possible ACK bit
  SCL(0);
  return b;
}
