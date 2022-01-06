#include <stdint.h>
#include "stm32f10x.h"

#define		nA_pin				GPIO_Pin_0
#define		nB_pin				GPIO_Pin_1
#define		nC_pin				GPIO_Pin_2
#define		nD_pin				GPIO_Pin_3
#define		nE_pin				GPIO_Pin_4
#define		nF_pin				GPIO_Pin_5
#define		nG_pin				GPIO_Pin_6
#define		nRDP_pin			GPIO_Pin_7

#define		digit_pins		(nA_pin | nB_pin | nC_pin | nD_pin | nE_pin | nF_pin | nG_pin | nRDP_pin)

#define		SEL1					GPIO_Pin_8
#define		SEL2					GPIO_Pin_9
#define		SEL3					GPIO_Pin_10
#define		SEL4					GPIO_Pin_11
#define		SEL5					GPIO_Pin_12

#define		select_pins		(SEL1 | SEL2 | SEL3 | SEL4 | SEL5)

#define		CURRENT_pin		GPIO_Pin_0
#define		POWER_pin			GPIO_Pin_1
#define		START_pin			GPIO_Pin_2
#define		STOP_pin			GPIO_Pin_3

#define		MODE_pins			(CURRENT_pin | POWER_pin)
#define		STATE_pins		(START_pin 	 | STOP_pin)
#define		LED_pins			(CURRENT_pin | POWER_pin | START_pin | STOP_pin)

#define		COL1					GPIO_Pin_8
#define		COL2					GPIO_Pin_9
#define		COL3					GPIO_Pin_10
#define		COL4					GPIO_Pin_11

#define		COLUMN_pins	  (COL1 | COL2 | COL3 | COL4)

#define		ROW1					GPIO_Pin_12
#define		ROW2					GPIO_Pin_13
#define		ROW3					GPIO_Pin_14
#define		ROW4					GPIO_Pin_15

#define		ROW_pins			(ROW1 | ROW2 | ROW3 | ROW4)

#define		KEYPAD_pins		(COLUMN_pins | ROW_pins)

#define		SPI_SCK_pin		GPIO_Pin_5
#define		SPI_MISO_pin	GPIO_Pin_6
#define		SPI_MOSI_pin	GPIO_Pin_7
#define		SPI_SEL2_pin	GPIO_Pin_13
#define		SPI_SEL3_pin	GPIO_Pin_15

#define		SPI_pins			(SPI_SCK_pin | SPI_MISO_pin | SPI_MOSI_pin)
#define		SPI_SEL_pins	(SPI_SEL2_pin | SPI_SEL3_pin)

__I uint8_t numberLUT[16]={1,2,3,0xA,4,5,6,0xB,7,8,9,0xC,0xE,0,0xF,0xD};

uint32_t SystemCoreClock = 8000000;
__I uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
__I uint8_t digit_segments[11] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0};
volatile uint32_t millis=0;

typedef enum {const_i, const_p} LoadMode;
LoadMode load_mode = const_i;
LoadMode last_load_mode = const_i;
FunctionalState load_state = DISABLE;
FunctionalState last_load_state = DISABLE;
volatile uint8_t current_digits[5] = {0,0,0,0,0};
uint8_t saved_digits[5] = {0,0,0,0,0};
volatile uint16_t req_value=0;
volatile uint16_t last_req_value=0;


void ms(uint32_t time)
{
	uint32_t temp=millis;
	while ((millis-temp)<=time) {}
}

void SystemInit(void)
{
	RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_6);	//set PLL=HSI*3=24 MHz
	RCC_PLLCmd(ENABLE);	//enable PLL
	while (!(RCC->CR & RCC_CR_PLLRDY)) {}	//wait for PLL to start
		
	RCC_HCLKConfig(RCC_SYSCLK_Div1);	//set HCLK=SYSCLK=24 MHz
	RCC_PCLK1Config(RCC_HCLK_Div1);		//SET PCLK1=HCLK=24 MHz
	RCC_PCLK2Config(RCC_HCLK_Div2);		//SET PCLK2=HCLK=24 MHz
		
	RCC_ADCCLKConfig(RCC_PCLK2_Div2);	//set ADCCLK=PCLK2/2=12 MHz
		
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);	//set SYSCLK=PLLCLK
	while (RCC_GetSYSCLKSource() != 0x08) {}		//wait for change to take effect
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);			//enable GPIOAclk	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);			//enable GPIOBclk	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);			//enable GPIOCclk
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);			//enable GPIODclk

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,  ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
}

void DisplayInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = nA_pin | nB_pin | nC_pin | nD_pin | nE_pin | nF_pin | nG_pin | nRDP_pin;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_SetBits(GPIOB, digit_pins);
	
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = SEL1 | SEL2 | SEL3 | SEL4 | SEL5;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);	
	
	GPIO_ResetBits(GPIOA, select_pins);
}

void LEDInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = LED_pins;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_ResetBits(GPIOA, LED_pins);
}

void KeypadInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin = KEYPAD_pins;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_10MHz;
	
	GPIO_Init(GPIOB, &GPIO_InitStruct);	
}

FunctionalState Keypressed(void)
{
	FunctionalState tmp=DISABLE;
	GPIO_InitTypeDef GPIO_InitStruct;	
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = ROW_pins;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);	
	GPIO_SetBits(GPIOB, ROW_pins);
	
	if ((GPIO_ReadInputData(GPIOB) & COLUMN_pins)!=0) tmp=ENABLE;
	
	GPIO_ResetBits(GPIOB, ROW_pins);
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStruct);		
	return tmp;
}

uint8_t ReadKey(void)
{
	uint32_t curr_row=ROW1;
	uint8_t tmp=0;
	uint8_t finished=0;
	
	GPIO_InitTypeDef GPIO_InitStruct;	
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_10MHz;	
	GPIO_SetBits(GPIOB, ROW_pins);
	
	for (uint8_t row=0; row<=3; row++)
	{
		GPIO_InitStruct.GPIO_Pin = curr_row;
		GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;		
		GPIO_Init(GPIOB, &GPIO_InitStruct);
		
		if ((GPIO_ReadInputData(GPIOB) & COLUMN_pins)!=0)
			{
				finished=1;
				tmp=(GPIO_ReadInputData(GPIOB) & COLUMN_pins)>>8;
				for (uint8_t searchbit=0; searchbit<=3; searchbit++)
					{
						if (tmp&0x1)
							{
								tmp=searchbit;
								break;
							}
						tmp>>=1;
					}
				tmp+=4*row;
			}
			
		curr_row<<=1;
		GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOB, &GPIO_InitStruct);
			
		if (finished) break;
	}
	
	if (finished) return numberLUT[tmp];
	
	return 255;
}

void ShowLEDs(void)
{
	GPIO_ResetBits(GPIOA, MODE_pins);
	GPIO_SetBits(GPIOA, (load_mode==const_i) ? CURRENT_pin : POWER_pin);
	
	GPIO_ResetBits(GPIOA, STATE_pins);
	GPIO_SetBits(GPIOA, (load_state==DISABLE) ? STOP_pin : START_pin);
}

void ShowDigit(uint8_t digit, FunctionalState RDP)
{
	GPIO_SetBits(GPIOB, digit_pins & (~(uint32_t) digit_segments[digit]));
	GPIO_ResetBits(GPIOB, (uint32_t) digit_segments[digit]);
	if (RDP==ENABLE) GPIO_ResetBits(GPIOB, nRDP_pin);
}

void SelectDigit(uint8_t digit)
{
	GPIO_ResetBits(GPIOA, select_pins);
	GPIO_SetBits(GPIOA, SEL5 >> digit);
}

void SPIInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;	
	SPI_InitTypeDef	SPI_InitStruct;
	
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = SPI_SCK_pin | SPI_MOSI_pin;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);	
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;	
	GPIO_InitStruct.GPIO_Pin = SPI_SEL_pins;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	GPIO_SetBits(GPIOC, SPI_SEL_pins);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;	
	GPIO_InitStruct.GPIO_Pin = SPI_MISO_pin;
	GPIO_Init(GPIOA, &GPIO_InitStruct);	
	
	SPI_StructInit(&SPI_InitStruct);
	SPI_InitStruct.SPI_Mode=SPI_Mode_Master;
	SPI_InitStruct.SPI_DataSize=SPI_DataSize_16b;
	SPI_InitStruct.SPI_NSS=SPI_NSS_Soft;
	SPI_InitStruct.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_64;
	SPI_Init(SPI1, &SPI_InitStruct);
	
	SPI_Cmd(SPI1, ENABLE);
}

void SendCommand(void)
{
	uint16_t temp_cmd=0;
	temp_cmd= (load_mode==const_i) ? req_value/2 : ((req_value/2) | 0x8000);
	if (load_state==DISABLE) temp_cmd=0;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY)) {};
		
	GPIO_ResetBits(GPIOC, SPI_SEL3_pin);
	for (uint16_t tmp=0; tmp<50; tmp++) {}
	SPI_I2S_SendData(SPI1, temp_cmd);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY)) {};	
	for (uint16_t tmp=0; tmp<50; tmp++) {}
	GPIO_SetBits(GPIOC, SPI_SEL3_pin);
		
	GPIO_ResetBits(GPIOC, SPI_SEL2_pin);
	for (uint16_t tmp=0; tmp<50; tmp++) {}
	SPI_I2S_SendData(SPI1, temp_cmd);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY)) {};	
	for (uint16_t tmp=0; tmp<50; tmp++) {}
	GPIO_SetBits(GPIOC, SPI_SEL2_pin);		
}

int main(void)
{
	SystemCoreClockUpdate();
	
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
	SysTick_Config(SystemCoreClock / 1000);
	
	DisplayInit();
	LEDInit();
	KeypadInit();
	SPIInit();
	
	while(0)
	{
		load_state=ENABLE;
		req_value=0xAAAA;
		SendCommand();
	}
	
	while (1) 
		{
			uint8_t key=0;
			ms(80);
			while(Keypressed()==DISABLE){}
			key=ReadKey();
				
			if ((key==0xE)|(key==0xF)|(key==0xFF)) continue;
				
			if ((key>=0xA)&(key<=0xD))
				{
					switch(key)
					{
						case 0xA:
							load_state=DISABLE;
							load_mode=const_i;
							req_value=0;
							for(uint8_t tmp=0; tmp<=4; tmp++) current_digits[tmp]=0;
							break;
						case 0xB:
							load_state=DISABLE;
							load_mode=const_p;
							req_value=0;
							for(uint8_t tmp=0; tmp<=4; tmp++) current_digits[tmp]=0;
							break;
						case 0xD:
							load_state=DISABLE;
							break;
						case 0xC:
							load_state=ENABLE;
							break;
					}
				}
			
			if (key<=9)
				{
					volatile uint8_t entrymode=1;
					uint16_t temp_number=0;
					for(uint8_t tmp=0; tmp<=4; tmp++) saved_digits[tmp]=current_digits[tmp];
					for(uint8_t tmp=0; tmp<=4; tmp++) current_digits[tmp]=10;
					
					current_digits[0]=key;
					
					while(entrymode)
						{
							current_digits[4]=0;
							ms(200);
							while(Keypressed()==DISABLE){}
							key=ReadKey();	
								
							switch(key)
								{
									case 0:
									case 1:
									case 2:
									case 3:
									case 4:
									case 5:
									case 6:
									case 7:
									case 8:
									case 9:
										for(int8_t tmp=2; tmp>=0; tmp--) current_digits[tmp+1]=current_digits[tmp];
										current_digits[0]=key;
										break;
									case 0xD:
										load_state=DISABLE;
										break;		
									case 0xE:
										for(uint8_t tmp=0; tmp<=3; tmp++) {if(current_digits[tmp]>9) current_digits[tmp]=0;};
										entrymode=0;
										break;
									case 0xF:
										for(uint8_t tmp=0; tmp<=3; tmp++) current_digits[tmp]=saved_digits[tmp];
										entrymode=0;
										break;
									default:
										break;
								}
						}
					
					if (current_digits[3]>3)
						{
							current_digits[3]=4;
							for(uint8_t tmp=0; tmp<=2; tmp++) current_digits[tmp]=0;
						}
						
					for(int8_t tmp=3; tmp>=0; tmp--) temp_number=(10*temp_number)+current_digits[tmp];
					req_value=10*temp_number;
				}
		}
}

void SysTick_Handler(void)
{
	static uint8_t selected_digit=0;
	selected_digit++;
	if (selected_digit>=4) selected_digit=0;
	
	SelectDigit(selected_digit);
	if (current_digits[selected_digit]>11) current_digits[selected_digit]=11;
	ShowDigit(current_digits[selected_digit], (((load_mode == const_i)&(selected_digit==2))|((load_mode == const_p)&(selected_digit==1))) ? ENABLE : DISABLE);
	
  millis++;
	if ((millis & 0x7F)==0) ShowLEDs();
	
	if ((load_state!=last_load_state)|(load_mode!=last_load_mode)|(req_value!=last_req_value))
		{
			last_load_state=load_state;
			last_load_mode=load_mode;
			last_req_value=req_value;
			SendCommand();
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
 //       prediv1factor = (RCC->CFGR2 & RCC_CFGR2_PREDIV1) + 1;
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

/* END OF FILE */
