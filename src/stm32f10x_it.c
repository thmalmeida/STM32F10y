/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"

volatile int time_ms;
volatile int time_us;
extern __IO int flag_1s;
//extern __IO int _USART1_cnt;
//extern __IO unsigned char _received_string[MAX_STRLEN+1];
//extern __IO unsigned char SPI_received_string[MAX_STRLEN+1];
//extern __IO int SPI1_cnt;
//extern __IO unsigned int SPI1_flag_rx;
//extern __IO unsigned int SPI1_flag_err;
//extern __IO unsigned char SPI1_rxd;
//extern __IO unsigned char SPI1_txd;
//#define master
//volatile int time_us;
//volatile int time_ms;

void NMI_Handler(void)
{}
void HardFault_Handler(void)
{  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}
void SVC_Handler(void)
{
}
void DebugMon_Handler(void)
{
}
void PendSV_Handler(void)
{
}
int count_1s = 1000000;
void SysTick_Handler(void)
{
	if(time_ms)
		time_ms--;

	if(time_us)
		time_us--;

	if(count_1s)
	{
		count_1s--;
	}
	else
	{
		flag_1s = 1;
		count_1s = 1000000;
	}
}


void _delay_us(int micro)
{
	time_us = micro;
	while(time_us>0);
}
void _delay_ms(int milli)
{
	time_ms = milli*1000;
	while(time_ms>0);
}
void SysTick_Init(void)
{
	// Cortex-M4 and M3
	// 72 MHz PLL clock divided by 8 and by 9;
	SysTick->LOAD  = 9;
	NVIC_SetPriority(SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);
	SysTick->CTRL  = SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}
void TIM_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* Enable the TIM3 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Initialize Leds mounted on STM32F3-Discovery EVAL board */
//	STM_EVAL_LEDInit(LED3);
//	STM_EVAL_LEDInit(LED4);
//	STM_EVAL_LEDInit(LED5);
//	STM_EVAL_LEDInit(LED6);

///*	TIM1_CR1:
//				9:8		CKD[1:0]
//				7		ARPE
//				6:5		CMS[1:0]
//				4		DIR
//				3		OPM
//				2		URS
//				1		UDIS
//				0		CEN
//*/
//	TIM3 -> CR1 |=  TIM_CR1_CEN;		// TIM1 Counter Enable;
	TIM3 -> CR1 &= ~TIM_CR1_DIR;		// Upconting mode DIR = 0;
//	TIM3 -> CR1 |=  TIM_CKD_DIV1;
//
///*	TIM1_SMCR:
//				15		ETP
//				7
//				2:0		SMS[2:0]
//*/
	TIM3 -> SMCR &= ~TIM_SMCR_SMS;		// Internal clock select;
//
///*	TIM1_DIER: DMA/interrupt enable register
//				14		TDE	- Trigger DMA request enable
//				8		UDE
//				6		TIE
//				0		UIE	- Uptade Interrupt enable
//*/
//	TIM1 -> DIER |= TIM_DIER_UIE;
//
	TIM3 -> ARR = 36000;	// Set TIM_Period;

/* Compute the prescaler value */
//	PrescalerValue = (uint16_t) 100*((SystemCoreClock)/72000000);

/* Time base configuration */
//	TIM_TimeBaseStructure.TIM_Period = 65535;
//	TIM_TimeBaseStructure.TIM_Prescaler = 0;
//	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//
//	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

/* Prescaler configuration */
//	TIM_PrescalerConfig(TIM3, PrescalerValue, TIM_PSCReloadMode_Immediate);
	TIM3->PSC = 2000;							// Set TIM_Prescaler;
	TIM3->EGR = TIM_PSCReloadMode_Immediate;	// Set or reset the UG Bit

/* Output Compare Timing Mode configuration: Channel1 */
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
//	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//	TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
//	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);

/* Output Compare Timing Mode configuration: Channel2 */
//	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//	TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
//	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
//	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Disable);

/* Output Compare Timing Mode configuration: Channel3 */
//	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//	TIM_OCInitStructure.TIM_Pulse = CCR3_Val;
//	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
//	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);

	/* Output Compare Timing Mode configuration: Channel 4 */
//	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//	TIM_OCInitStructure.TIM_Pulse = CCR4_Val;
//	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
//	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Disable);

/* TIM Interrupts enable */
//	TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);
	TIM3 -> DIER |= TIM_IT_CC4;

	/* TIM3 enable counter */
//	TIM_Cmd(TIM3, ENABLE);
	TIM3 -> CR1 |=  TIM_CR1_CEN;		// TIM1 Counter Enable;

	/* Turn on LED3, LED4, LED5 and LED6 */
//  STM_EVAL_LEDOn(LED3);
//  STM_EVAL_LEDOn(LED4);
//  STM_EVAL_LEDOn(LED5);
//  STM_EVAL_LEDOn(LED6);
}
void IO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph, ENABLE);

	/* Configure pins:  ---------------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	/* Configure SCK and MOSI pins as Alternate Function Push Pull */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

//	GPIOA -> MODER = (0x02 << GPIO_P)
//	GPIOA -> M

//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_5); //SCK
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_5); //MISO
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_5); //MOSI
}
void RCC_Init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
}
void ADC1_Init()
{
	// Enable the ADC1 clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	ADC_InitTypeDef ADC_InitStructure;
	//ADC1 configuration
	//select independent conversion mode (single)
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	//We will convert single channel only
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	//we will convert one time
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	//select no external triggering
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	//right 12-bit data alignment in ADC data register
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	//single channel conversion
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	//load structure values to control and status registers
	ADC_Init(ADC1, &ADC_InitStructure);
	//wake up temperature sensor
	ADC_TempSensorVrefintCmd(ENABLE);
	//ADC1 channel16 configuration
	//we select 41.5 cycles conversion for channel16
	//and rank=1 which doesn't matter in single mode
	ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_41Cycles5);
	//Enable ADC1
	ADC_Cmd(ADC1, ENABLE);
	//Enable ADC1 reset calibration register
	ADC_ResetCalibration(ADC1);
	//Check the end of ADC1 reset calibration register
	while(ADC_GetResetCalibrationStatus(ADC1));
	//Start ADC1 calibration
	ADC_StartCalibration(ADC1);
	//Check the end of ADC1 calibration
	while(ADC_GetCalibrationStatus(ADC1));
}
int ADC_Read(uint8_t channel)
{
	uint16_t AD_value;

	ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_41Cycles5);

	//Start ADC1 Software Conversion
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	//wait for conversion complete
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)){}
	//read ADC value
	AD_value=ADC_GetConversionValue(ADC1);
	//clear EOC flag
	ADC_ClearFlag(ADC1, ADC_FLAG_EOC);

	return AD_value;
}
void LED_green(uint8_t status)
{
	if(status)
	{
		GPIOC -> BSRR = (1<<(13+16)); // 16 bit shift
//		GPIO_SetBits(GPIOC, 13);
	}
	else
	{
		GPIOC -> BSRR = (1<<13);
//		GPIO_ResetBits(GPIOC, 13);
	}
}
void LED_green_toogle()
{
	GPIOC -> ODR ^= (1<<13);
}
void init(void)
{
	SystemInit(); 		// Setup STM32 system (clock, PLL and Flash configuration)
	ADC1_Init();
	IO_Init();
	SysTick_Init();
}

//void USART1_IRQHandler(void)
//{
//	// check if the USART1 receive interrupt flag was set
//	while(USART_GetITStatus(USART1, USART_IT_RXNE))
//	{
////		LED_green_toogle();
//		char t = USART1->DR; // the character from the USART1 data register is saved in t
//		if(USART1_cnt < MAX_STRLEN)
//		{
//			received_string[USART1_cnt] = t;
//			USART1_cnt++;
//		}
//		else
//		{
//			memset(received_string,0,sizeof(received_string));
//			USART1_cnt = 0;
//		}
//
////		USART1_putc(t);
//
//		// check if the received character is not the LF character (used to determine end of string)
//		// or the if the maximum string length has been been reached
////		if( (t != '\n') && (cnt < MAX_STRLEN) )
////		{
////			received_string[cnt] = t;
////			cnt++;
////		}
////		else{ // otherwise reset the character counter and print the received string
////			cnt = 0;
////			USART1_puts(received_string);
////			memset(received_string,0,sizeof(received_string));
////		}
//	}
//}

//void SPI1_IRQHandler(void)
//{
//#ifdef master
//	//	static unsigned short int count = 0, i = 0 ;
//	//	check if the SPI1 receive interrupt flag was set
//	//	Master interrupt
//	//	TX: 1- empty
//	//	if(SPI1 -> SR & 0x0002)
//		if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_TXE) == SET)
//		{
//			SPI1_flag_rx = 1;
//			SPI1_rxd++;
//			SPI1_txd = SPI1_rxd;
//			SPI1 -> DR = SPI1_txd;
//	//		Wait until the data has been transmitted.
//	//		while (!(SPI1->SR & SPI_I2S_FLAG_TXE));
//	//		USART1_print("Transmitting: ");
//	//		USART1_putc(0x40);
//		}
////	//	if(SPI1 -> SR & SPI_I2S_FLAG_RXNE)
////		//	RX: 1- not empty
////	//	if(SPI1 -> SR & 0x0001)
//		if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) == SET)
//		{
//			SPI1_flag_rx = 1;
//			SPI1_rxd = SPI1 -> DR;
//			if(SPI1_cnt < MAX_STRLEN)
//			{
//				SPI_received_string[SPI1_cnt] = SPI1_rxd;
//				SPI1_cnt++;
//			}
//			else
//			{
//				memset(SPI_received_string, 0, sizeof(SPI_received_string));
//				SPI1_cnt = 0;
//			}
//		}
//
//	//	if(SPI1 -> SR & 0x0100)
//		if(SPI_I2S_GetITStatus(SPI1, SPI_IT_CRCERR))
//		{
//			SPI1_flag_err = 1;
//		}
//#else
////	static unsigned short int count = 0, i = 0 ;
////	check if the SPI1 receive interrupt flag was set
////	Slave interrupt
////	TX: 1- empty
////	if(SPI1 -> SR & 0x0002)
//	if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_TXE) == SET)
//	{
//		SPI1_rxd++;
//		SPI1_txd = SPI1_rxd;
//		SPI1 -> DR = SPI1_txd;
////		Wait until the data has been transmitted.
////		while (!(SPI1->SR & SPI_I2S_FLAG_TXE));
////		USART1_print("Transmitting: ");
////		USART1_putc(0x40);
//	}
////	if(SPI1 -> SR & SPI_I2S_FLAG_RXNE)
//	//	RX: 1- not empty
////	if(SPI1 -> SR & 0x0001)
//	if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) == SET)
//	{
//		SPI1_flag_rx = 1;
//		SPI1_rxd = SPI1 -> DR;
//		if(SPI1_cnt < MAX_STRLEN)
//		{
//			SPI_received_string[SPI1_cnt] = SPI1_rxd;
//			SPI1_cnt++;
//		}
//		else
//		{
//			memset(SPI_received_string, 0, sizeof(SPI_received_string));
//			SPI1_cnt = 0;
//		}
//	}
//
////	if(SPI1 -> SR & 0x0100)
//	if(SPI_I2S_GetITStatus(SPI1, SPI_IT_CRCERR))
//	{
//		SPI1_flag_err = 1;
//	}
//#endif
//}
