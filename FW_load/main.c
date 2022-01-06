#include <stdint.h>
#include <stdbool.h>
#include "board.h"
#include "stm32f0xx.h"
#include "stm32f0xx_misc.h"


uint16_t tmp=0;

volatile uint16_t millis=0;
volatile uint32_t voltage=0, current=0, power=0, temperature=0;
volatile uint32_t max_current=0, max_power=0;
//volatile uint16_t read_i=0, read_integral=0; 
typedef enum {CURRENT, POWER} op_mode;
volatile uint16_t req_current=0, req_power=0;
op_mode load_mode=CURRENT;

void SPI1_IRQHandler(void)
{
	uint16_t temp;
	ChangeLEDState(LED_OK);
	temp=SPI_I2S_ReceiveData16(SPI1);
	if (temp & 0x8000)
		{
			load_mode=POWER;
			req_power=temp & 0x7FFF;
		}
		else
		{
			load_mode=CURRENT;
			req_current=temp & 0x7FFF;
		}
	SPI_I2S_ClearFlag(SPI1, SPI_I2S_FLAG_RXNE);
}

void SysTick_Handler(void)
{
	millis++;
}

void ms(uint16_t time)
{
	uint16_t temp=millis;
	while ((millis-temp)<=time) {}
}

int main(void)
{
	SystemCoreClockUpdate();
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
	SysTick_Config(SystemCoreClock / 1000);
	BoardInit();
	
	while(0)
	{
		OverrideMode(NO_OVERRIDE);
		DAC_Write_Fast(tmp);
	}
	
	while(1)
	{
		voltage=50*ADC_Read(ADC_Channel_3); 													//returns voltage in millivolts
		temperature=(754*(uint32_t)ADC_Read(ADC_Channel_2))/9500-50;	//calculate temperaure in degrees Centigrade
		//read_i=10*ADC_Read(ADC_Channel_1);														//read integraled voltage
		//read_integral=ADC_Read(ADC_Channel_0);												//read current
		max_power=280000-(2200*((int32_t)temperature-25));						//calculate maximum power in watts, works for IRFP260N
		if (temperature>125) max_power=0;															//cap temperature to 130 °C
		if (max_power>240000) max_power=240000;												//down-coerce maximum power to 240 W
		max_current=max_power*1000/voltage;														//calculate maximum possible current
		if (voltage<2000) 
			{
				SetCurrent(0);
				ms(100);
			}
		if (max_current>20000) max_current=20000;											//down-coerce maximum current to 20 A
		
		
		if (load_mode==CURRENT) //CURRENT mode
			{
				if(req_current>max_current)
					{current=max_current;} else
					current=req_current;
				SetCurrent(current);
			} else		//POWER mode
			{
				if(10*req_power>max_power)
					{power=max_power;} else
					power=10*req_power;
				SetCurrent((1000*power)/voltage);
			}
	}
}

void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct)
{
  uint32_t tmppriority = 0x00;
  
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NVIC_InitStruct->NVIC_IRQChannelCmd));
  assert_param(IS_NVIC_PRIORITY(NVIC_InitStruct->NVIC_IRQChannelPriority));  
    
  if (NVIC_InitStruct->NVIC_IRQChannelCmd != DISABLE)
  {
    /* Compute the Corresponding IRQ Priority --------------------------------*/    
    tmppriority = NVIC->IP[NVIC_InitStruct->NVIC_IRQChannel >> 0x02];
    tmppriority &= (uint32_t)(~(((uint32_t)0xFF) << ((NVIC_InitStruct->NVIC_IRQChannel & 0x03) * 8)));
    tmppriority |= (uint32_t)((((uint32_t)NVIC_InitStruct->NVIC_IRQChannelPriority << 6) & 0xFF) << ((NVIC_InitStruct->NVIC_IRQChannel & 0x03) * 8));    
    
    NVIC->IP[NVIC_InitStruct->NVIC_IRQChannel >> 0x02] = tmppriority;
    
    /* Enable the Selected IRQ Channels --------------------------------------*/
    NVIC->ISER[0] = (uint32_t)0x01 << (NVIC_InitStruct->NVIC_IRQChannel & (uint8_t)0x1F);
  }
  else
  {
    /* Disable the Selected IRQ Channels -------------------------------------*/
    NVIC->ICER[0] = (uint32_t)0x01 << (NVIC_InitStruct->NVIC_IRQChannel & (uint8_t)0x1F);
  }
}
