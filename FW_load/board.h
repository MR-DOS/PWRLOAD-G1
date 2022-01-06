#ifndef __BOARD_H
#define __BOARD_H

#include <stdint.h>
#include "stm32f0xx.h"

#define SysTick_CLKSource_HCLK_Div8    ((uint32_t)0xFFFFFFFB)
#define SysTick_CLKSource_HCLK         ((uint32_t)0x00000004)
#define IS_SYSTICK_CLK_SOURCE(SOURCE)  (((SOURCE) == SysTick_CLKSource_HCLK) || \
                                       ((SOURCE) == SysTick_CLKSource_HCLK_Div8))

#define LED_PORT 							GPIOB
#define LED_PIN 							GPIO_Pin_1
#define LED_OK								ENABLE
#define LED_ERROR							DISABLE

#define	AN_PORT								GPIOA

#define READ_INTEGRAL_PORT		AN_PORT
#define READ_INTEGRAL_PIN 		GPIO_Pin_0
#define READ_I_PORT						AN_PORT
#define READ_I_PIN 						GPIO_Pin_1
#define TEMPERATURE_PORT			AN_PORT
#define	TEMPERATURE_PIN				GPIO_Pin_2
#define READ_U_PORT						AN_PORT
#define READ_U_PIN 						GPIO_Pin_3

#define OVR1_PORT							GPIOF
#define OVR1_PIN							GPIO_Pin_0
#define OVR2_PORT							GPIOF
#define OVR2_PIN							GPIO_Pin_1

#define PASTE_EVAL(x,y)				x##y
#define PASTE(x,y)						PASTE_EVAL(x,y)
#define GPIO_PIN_TEMPLATE			GPIO_Pin_

#define I2C_PORT							GPIOA
#define SCL_PIN_NUMBER				9
#define	SCL_PIN								PASTE(GPIO_PIN_TEMPLATE, SCL_PIN_NUMBER)
#define SDA_PIN_NUMBER				10
#define	SDA_PIN								PASTE(GPIO_PIN_TEMPLATE, SDA_PIN_NUMBER)

#define SPI_PORT							GPIOB
#define SPI_NSS_PIN_NUMBER		4
#define SPI_SCK_PIN_NUMBER		5
#define SPI_MISO_PIN_NUMBER		6
#define SPI_MOSI_PIN_NUMBER		7
#define	SPI_NSS_PIN						PASTE(GPIO_PIN_TEMPLATE, SPI_NSS_PIN_NUMBER)
#define	SPI_SCK_PIN						PASTE(GPIO_PIN_TEMPLATE, SPI_SCK_PIN_NUMBER)
#define	SPI_MISO_PIN					PASTE(GPIO_PIN_TEMPLATE, SPI_MISO_PIN_NUMBER)
#define	SPI_MOSI_PIN					PASTE(GPIO_PIN_TEMPLATE, SPI_MISO_PIN_NUMBER)

#define GENERAL_CALL_RESET		0x06
#define GENERAL_CALL_WAKEUP		0x09

#define MCP4725_ADDR					0xC0
		//the device code part of address of MCP4725, do not change

typedef enum
{
	NO_OVERRIDE, SOFT_OVERRIDE, HARD_OVERRIDE, COMPLETE_OVERRIDE
} OverrideState;

void BoardInit(void);
void DAC_Write_Fast(uint16_t DAC_value);
void ChangeLEDState(FunctionalState NewState);
uint16_t ADC_Read(uint32_t channel);
//void SysTick_CLKSourceConfig(uint32_t SysTick_CLKSource);
void SystemCoreClockUpdate(void);
void OverrideMode(OverrideState mode);
void SetCurrent(int32_t current);

#endif //__BOARD_H
