#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_misc.h"
#include "board.h"
#include "soft_i2c.h"
#include "main.h"

__I uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
uint32_t SystemCoreClock    = 8000000;
volatile uint8_t mcp4725_addr = 0;

//const int32_t calib1=16208;	//calibration constant for slope
//const int32_t calib0=516541;	//calibration constant for offset
const int32_t calib1=16428;	//calibration constant for slope
const int32_t calib0=580847;	//calibration constant for offset

void SystemInit(void)
{
	RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_12);
	//set PLL to HSI*12/2=48 MHz
	RCC_PLLCmd(ENABLE);
	//enable PLL
		
	RCC_HCLKConfig(RCC_SYSCLK_Div1);
	//set HCLK to 48 MHz
	RCC_PCLKConfig(RCC_HCLK_Div1);
	//set PCLK to 48 MHz
	ADC_ClockModeConfig(ADC1, ADC_ClockMode_AsynClk);
	//set ADC clock to HSI14
	
	FLASH_SetLatency(1);
	//set FLASH latency to 1
	FLASH_PrefetchBufferCmd(ENABLE);
	//enable prefetch buffer
		
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY)==RESET){}
	//wait for PLLRDY
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	//set SYSCLK to PLLCLK
	while (RCC_GetSYSCLKSource()!=0x08) {}
	//wait for PLLCLK to be selected as SYSCLK
}

void SPIInit(void)
{
	SPI_InitTypeDef SPI_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
	SPI_I2S_DeInit(SPI1);
	SPI_StructInit(&SPI_InitStruct);
	SPI_InitStruct.SPI_Direction=SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_DataSize=SPI_DataSize_16b;
	SPI_InitStruct.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_2;
	SPI_Init(SPI1, &SPI_InitStruct);
	
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Pin = SPI_SCK_PIN | SPI_MISO_PIN | GPIO_Pin_7 | SPI_NSS_PIN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_PinAFConfig(GPIOA, SPI_SCK_PIN_NUMBER, GPIO_AF_0);	
	GPIO_PinAFConfig(GPIOA, SPI_MISO_PIN_NUMBER, GPIO_AF_0);	
	GPIO_PinAFConfig(GPIOA, SPI_MOSI_PIN_NUMBER, GPIO_AF_0);
	
	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
	//SPI_RxFIFOThresholdConfig(SPI1, SPI_RxFIFOThreshold_QF);
	SPI_Cmd(SPI1, ENABLE);
	
	NVIC_InitStruct.NVIC_IRQChannel=SPI1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPriority=3;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}

void LEDInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;		
	//set PB1 as LED output pins
	
	GPIO_StructInit(&GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin= LED_PIN;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(LED_PORT, &GPIO_InitStruct);

	ChangeLEDState(LED_OK);
	//set LED state to OK
}

void ADCInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	ADC_InitTypeDef ADC_InitStruct;
	//init ADC pins to analog mode
	
	GPIO_StructInit(&GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_Pin = READ_I_PIN | READ_INTEGRAL_PIN | READ_U_PIN | TEMPERATURE_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(AN_PORT, &GPIO_InitStruct);
	
	ADC_StructInit(&ADC_InitStruct);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	ADC_Init(ADC1, &ADC_InitStruct);
	ADC_GetCalibrationFactor(ADC1);
	ADC_Cmd(ADC1, ENABLE);
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN) == RESET);
}

uint16_t ADC_Read(uint32_t channel)
{
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY) == RESET){}
	ADC_ChannelConfig(ADC1, channel, ADC_SampleTime_239_5Cycles);
	ADC_StartOfConversion(ADC1);
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET){}
	ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
	return ADC_GetConversionValue(ADC1);
}

void I2CInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;
	I2C_StructInit(&I2C_InitStruct);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_OD;
	GPIO_InitStruct.GPIO_Pin= SCL_PIN | SDA_PIN;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_10MHz;
	
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_PinAFConfig(GPIOA, SCL_PIN_NUMBER, GPIO_AF_4);
	GPIO_PinAFConfig(GPIOA, SDA_PIN_NUMBER, GPIO_AF_4);	
	
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStruct.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
	I2C_InitStruct.I2C_DigitalFilter = 0;
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_OwnAddress1 = 0xAB;
	I2C_InitStruct.I2C_Timing = 0x10805E89; 
	//100 kHz Mode, Analog Filter, 48 MHz clock, rise 100ns, Fall 10ns, dig coeff 0
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C1, &I2C_InitStruct);
	I2C_Cmd(I2C1, ENABLE);
	
	I2C_TransferHandling(I2C1, 0, 1, I2C_AutoEnd_Mode, I2C_Generate_Start_Write);
	//send general call
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXE) == RESET);
	//wait for completion
	I2C_SendData(I2C1, (uint8_t)GENERAL_CALL_RESET);
	//send general reset request	
	I2C_ClearFlag(I2C1, I2C_FLAG_STOPF);	
	//clear STOP flag

	I2C_SoftwareResetCmd(I2C1);
	//reset I2C
}

uint8_t Find_MCP4725_Address(void)
{
	uint8_t mcp_dev_addr=0, mcp_addr=0;
		
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET);
	//wait for I2C1 to get ready
	
	//detect MCP4725 I2C address, uses MCP4725_ADDR as base and changes bits [3:2]
	for (mcp_dev_addr = 0; (mcp_dev_addr<4) & ((I2C_GetFlagStatus(I2C1, I2C_FLAG_NACKF) == SET) | (mcp_dev_addr==0)); mcp_dev_addr++)
	{
		I2C_ClearFlag(I2C1, I2C_FLAG_NACKF);	//clear NACK flag
		I2C_ClearFlag(I2C1, I2C_FLAG_STOPF);	//clear STOP flag
		mcp_addr = MCP4725_ADDR + (mcp_dev_addr << 2); 
		//calculate MCP4725 address
		I2C_TransferHandling(I2C1, mcp_addr, 0, I2C_AutoEnd_Mode, I2C_Generate_Start_Write);
		ms(2);
		//send address+RW bit, return STOPF+NACKF on error, STOPF on success
	}
	
	I2C_ClearFlag(I2C1, I2C_FLAG_STOPF);	
	//clear STOP flag
	
	I2C_SoftwareResetCmd(I2C1);
	
	return mcp_addr;
}

void DAC_Write_Fast(uint16_t DAC_value)
{
	if (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET)
				I2C_SoftwareResetCmd(I2C1);
	//wait for I2C1 to get ready
	I2C_TransferHandling(I2C1, mcp4725_addr, 2, I2C_AutoEnd_Mode, I2C_Generate_Start_Write);
	//send address+RW bit
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXE) == RESET);
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == RESET);
	//wait for break
			
	I2C_SendData(I2C1, (uint8_t)((DAC_value>>8)&0x0F));
	//send 
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXE) == RESET)
			{if(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == RESET) return;};
	//wait for completion
			
	I2C_SendData(I2C1, (uint8_t)(DAC_value & 0xFF));
	//send
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF) == RESET)
			{if(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == RESET) return;};
	//wait for completion
	
	I2C_ClearFlag(I2C1, I2C_FLAG_STOPF);	//clear STOP flag
}

void OVRInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_StructInit(&GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin= OVR1_PIN | OVR2_PIN;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(OVR1_PORT, &GPIO_InitStruct);

	OverrideMode(COMPLETE_OVERRIDE);
}

void OverrideMode(OverrideState mode)
{
	switch(mode)
	{
		case NO_OVERRIDE: 			{GPIO_WriteBit(OVR1_PORT, OVR1_PIN, Bit_SET);		GPIO_WriteBit(OVR2_PORT, OVR2_PIN, Bit_RESET);	return;};
		case SOFT_OVERRIDE: 		{GPIO_WriteBit(OVR1_PORT, OVR1_PIN, Bit_RESET);	GPIO_WriteBit(OVR2_PORT, OVR2_PIN, Bit_RESET);	return;};
		case HARD_OVERRIDE: 		{GPIO_WriteBit(OVR1_PORT, OVR1_PIN, Bit_SET);		GPIO_WriteBit(OVR2_PORT, OVR2_PIN, Bit_SET);		return;};
		case COMPLETE_OVERRIDE: {GPIO_WriteBit(OVR1_PORT, OVR1_PIN, Bit_RESET);	GPIO_WriteBit(OVR2_PORT, OVR2_PIN, Bit_SET);		return;};
	}
}

void BoardInit(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);
	OVRInit();		//init override function, enable override
	LEDInit();		//init LED, set to OK
	ADCInit();		//init ADC pins
	I2CInit();		//init I2C peripheral and send general reset call
	//mcp4725_addr = Find_MCP4725_Address(); //find address of MCP4725
	mcp4725_addr = MCP4725_ADDR + (1 << 2); //use A1 version of MCP4725
	DAC_Write_Fast(0); //zero the DAC
	SPIInit();
}

void SetCurrent (int32_t miliamperes)
{
	int32_t temp_current= ((miliamperes*calib1)-calib0)>>17;
	
	if (temp_current<=0) 
	{
		temp_current=0;
		OverrideMode(SOFT_OVERRIDE);
		ChangeLEDState(LED_ERROR);
	} else
	{
		OverrideMode(NO_OVERRIDE);
		ChangeLEDState(LED_OK);
	}
	
	DAC_Write_Fast(temp_current);
}

void ChangeLEDState(FunctionalState NewState)
{
	GPIO_WriteBit(LED_PORT, LED_PIN, (NewState==DISABLE) ? Bit_SET : Bit_RESET);
}


void SysTick_CLKSourceConfig(uint32_t SysTick_CLKSource)
{
  assert_param(IS_SYSTICK_CLK_SOURCE(SysTick_CLKSource));
  
  if (SysTick_CLKSource == SysTick_CLKSource_HCLK)
  {
    SysTick->CTRL |= SysTick_CLKSource_HCLK;
  }
  else
  {
    SysTick->CTRL &= SysTick_CLKSource_HCLK_Div8;
  }
}

void SystemCoreClockUpdate (void)
{
  uint32_t tmp = 0, pllmull = 0, pllsource = 0, prediv1factor = 0;

  /* Get SYSCLK source -------------------------------------------------------*/
  tmp = RCC->CFGR & RCC_CFGR_SWS;
  
  switch (tmp)
  {
    case 0x00:  /* HSI used as system clock */
      SystemCoreClock = HSI_VALUE;
      break;
    case 0x04:  /* HSE used as system clock */
      SystemCoreClock = HSE_VALUE;
      break;
    case 0x08:  /* PLL used as system clock */
      /* Get PLL clock source and multiplication factor ----------------------*/
      pllmull = RCC->CFGR & RCC_CFGR_PLLMULL;
      pllsource = RCC->CFGR & RCC_CFGR_PLLSRC;
      pllmull = ( pllmull >> 18) + 2;
      
      if (pllsource == 0x00)
      {
        /* HSI oscillator clock divided by 2 selected as PLL clock entry */
        SystemCoreClock = (HSI_VALUE >> 1) * pllmull;
      }
      else
      {
        prediv1factor = (RCC->CFGR2 & RCC_CFGR2_PREDIV1) + 1;
        /* HSE oscillator clock selected as PREDIV1 clock entry */
        SystemCoreClock = (HSE_VALUE / prediv1factor) * pllmull; 
      }      
      break;
    default: /* HSI used as system clock */
      SystemCoreClock = HSI_VALUE;
      break;
  }
  /* Compute HCLK clock frequency ----------------*/
  /* Get HCLK prescaler */
  tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
  /* HCLK clock frequency */
  SystemCoreClock >>= tmp;  
}
