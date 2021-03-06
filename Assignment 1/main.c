#include "stm32f4xx.h"
#include "main.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_it.h"
#include <misc.h>

GPIO_InitTypeDef GPIO_Initstructure;
TIM_TimeBaseInitTypeDef timer_InitStructure;
EXTI_InitTypeDef EXTI_InitStructure;
NVIC_InitTypeDef nvicStructure;
NVIC_InitTypeDef NVIC_InitStructure;
int brewTimes[4] = {3, 4, 5, 6};
fir_8 filt;
int curr_led = 12;
int button_press_count = 0;
int brewing = 0;
int iterations = 0;
int button_press = -1;
int button_release = -1;

void Delay(__IO uint32_t time);
extern __IO uint32_t TimmingDelay;

void delay(__IO uint32_t nCount){
	while(nCount--){}
}

__IO uint32_t TimmingDelay;
void SysTick_Handler(void)
{
  if(TimmingDelay !=0)
  {
    TimmingDelay --;
  
  }
  
  
}

void InitLEDs()
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
 
 	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Initstructure.GPIO_OType = GPIO_OType_PP;  
	GPIO_Initstructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_Initstructure);
}

void InitTimers()
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
      
  timer_InitStructure.TIM_Prescaler = 839;
  timer_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  timer_InitStructure.TIM_Period = 49999;
  timer_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  timer_InitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2, &timer_InitStructure);
  TIM_Cmd(TIM2, DISABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

void InitTimers2()
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
      
  timer_InitStructure.TIM_Prescaler = 839;
  timer_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  timer_InitStructure.TIM_Period = 49999;
  timer_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  timer_InitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM3, &timer_InitStructure);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
}

void EnableTimerInterrupt()
{
    nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
    nvicStructure.NVIC_IRQChannelSubPriority = 3;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);
}

void EnableTimerInterrupt2()
{
    nvicStructure.NVIC_IRQChannel = TIM3_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
    nvicStructure.NVIC_IRQChannelSubPriority = 2;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);
}

void InitButton() // initialize user button
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 
	
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Initstructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Initstructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_Initstructure);
}

void InitEXTI()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    // Selects the GPIOA pin 0 used as external interrupt source
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
    
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_Init(&EXTI_InitStructure);
}

void EnableEXTIInterrupt()
{
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void TIM2_IRQHandler() {
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
				int i = 0;
				int brewTime = brewTimes[curr_led-12];
				brewTime = brewTime * 2;
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
				
				if(iterations <= brewTime && brewing == 1) {
					if(curr_led == 12)
				{
					GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
				}
				else if(curr_led == 13)
				{
					GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
				}
				else if(curr_led == 14)
				{
					GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
				}
				else if(curr_led == 15)
				{
					GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
				}
				iterations++;
				} else if(iterations > brewTimes [curr_led-12]) {
					TIM_Cmd(TIM2, DISABLE);
					//Finished brewing play sound
					//Just use a crude loop to play the sound for roughly 1 second
					if(brewing == 1){
						while(i < 4800000) {
							if (SPI_I2S_GetFlagStatus(CODEC_I2S, SPI_I2S_FLAG_TXE)){
								SPI_I2S_SendData(CODEC_I2S, sample);

								if (sampleCounter & 0x00000001){
									sawWave += NOTEFREQUENCY;
									if (sawWave > 1.0)
										sawWave -= 2.0;

									filteredSaw = updateFilter(&filt, sawWave);
									sample = (int16_t)(NOTEAMPLITUDE*filteredSaw);
								}
								sampleCounter++;
							}
						i++;
						}
					}
	
					//Reset bits so we're at the starting state again
					GPIO_SetBits(GPIOD, GPIO_Pin_12);
					GPIO_ResetBits(GPIOD, GPIO_Pin_13);
					GPIO_ResetBits(GPIOD, GPIO_Pin_14);
					GPIO_ResetBits(GPIOD, GPIO_Pin_15);
	
					curr_led = 12;
	
					brewing = 0;
					iterations = 0;
				}
    }
}

void TIM3_IRQHandler()
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
				int count = button_press_count;
				button_press_count = 0;
				TIM_Cmd(TIM3, DISABLE);
				TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
				if(count == 1 && brewing == 0) {
					if(curr_led == 15){
						GPIO_ResetBits(GPIOD, GPIO_Pin_15);
						GPIO_SetBits(GPIOD, GPIO_Pin_12);
						curr_led = 12;
					} else if(curr_led == 14) {
						GPIO_ResetBits(GPIOD, GPIO_Pin_14);
						GPIO_SetBits(GPIOD, GPIO_Pin_15);
						curr_led++;
					} else if(curr_led == 13) {
						GPIO_ResetBits(GPIOD, GPIO_Pin_13);
						GPIO_SetBits(GPIOD, GPIO_Pin_14);
						curr_led++;
					} else if(curr_led == 12) {
						GPIO_ResetBits(GPIOD, GPIO_Pin_12);
						GPIO_SetBits(GPIOD, GPIO_Pin_13);
						curr_led++;
					}
				} else if(count == 2 && brewing == 0) {
					brewing = 1;
					TIM_Cmd(TIM2, ENABLE);
				} else if(count == 1 && brewing == 1) {
					//Restart brewing by canceling and then re-brewing
					iterations = 0;
				} else if(count == 2 && brewing == 1) {
					//Cancel brewing
					TIM_Cmd(TIM2, DISABLE);
					//Reset bits so we're at the starting state again
					GPIO_SetBits(GPIOD, GPIO_Pin_12);
					GPIO_ResetBits(GPIOD, GPIO_Pin_13);
					GPIO_ResetBits(GPIOD, GPIO_Pin_14);
					GPIO_ResetBits(GPIOD, GPIO_Pin_15);
	
					curr_led = 12;
	
					brewing = 0;
					iterations = 0;
				}
    }
}

void EXTI0_IRQHandler()
{
    // Checks whether the interrupt from EXTI0 or not
    if (EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
			if(button_press_count == 0) {
				TIM_Cmd(TIM3, ENABLE);
			}
			button_press_count++;
	
			// Clears the EXTI line pending bit		
			EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

float updateFilter(fir_8* filt, float val)
{
	uint16_t valIndex;
	uint16_t paramIndex;
	float outval = 0.0;

	valIndex = filt->currIndex;
	filt->tabs[valIndex] = val;

	for (paramIndex=0; paramIndex<8; paramIndex++)
	{
		outval += (filt->params[paramIndex]) * (filt->tabs[(valIndex+paramIndex)&0x07]);
	}

	valIndex++;
	valIndex &= 0x07;

	filt->currIndex = valIndex;

	return outval;
}

void initFilter(fir_8* theFilter)
{
	uint8_t i;

	theFilter->currIndex = 0;

	for (i=0; i<8; i++)
		theFilter->tabs[i] = 0.0;

	theFilter->params[0] = 0.01;
	theFilter->params[1] = 0.05;
	theFilter->params[2] = 0.12;
	theFilter->params[3] = 0.32;
	theFilter->params[4] = 0.32;
	theFilter->params[5] = 0.12;
	theFilter->params[6] = 0.05;
	theFilter->params[7] = 0.01;
}

int main() {
	InitLEDs();
	InitTimers();
	InitTimers2();
	EnableTimerInterrupt();
	EnableTimerInterrupt2();
	InitButton();
	InitEXTI();
	EnableEXTIInterrupt();
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_9; //I2S SCL and SDA pins
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_SPI3);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	SysTick_Config(SystemCoreClock/1000);

	GPIO_Init(GPIOD, &GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	codec_init();
	codec_ctrl_init();

	I2S_Cmd(CODEC_I2S, ENABLE);

	initFilter(&filt);
	
	GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
	
	while(1){

	}
}

void Delay(__IO uint32_t time)
{
  TimmingDelay = time;
  while(TimmingDelay !=0);
}
