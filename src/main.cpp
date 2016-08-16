#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define master
#define debug

#include <stm32f10x.h>
#include <stm32f10x_rtc.h>

#include "stm32f10x_it.h"
#include "eeprom.h"

#include "nRF24L01p.h"

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
__IO int time_us;
__IO int time_ms;
__IO int flag_1s;
__IO int USART1_cnt;
__IO int SPI1_cnt;
__IO char received_string[MAX_STRLEN+1]; // this will hold the recieved string
__IO unsigned char SPI_received_string[MAX_STRLEN+1]; // this will hold the recieved string
__IO unsigned int SPI1_flag_rx;
__IO unsigned char SPI1_rxd;
__IO unsigned char SPI1_txd;
__IO unsigned int SPI1_flag_err;

// Bluetooth str parse variables
uint8_t enableDecode = 0, opcode;
uint8_t k, rLength, j;
uint8_t j2 = 0;
uint8_t flag_frameStartBT = 0;
uint8_t enableTranslate_Bluetooth = 0;
char aux[3], aux2[4], buffer[60], inChar, sInstr[20];
char sInstrBluetooth[30];

// SPI Defines
/*
	SPI1: default
	SPI1_MOSI - PA7
	SPI1_MISO - PA6
	SPI1_SCK  - PA5
	SPI1_NSS  - PA4

	SPI1: remap
	SPI1_MOSI - PB5
	SPI1_MISO - PB4
	SPI1_SCK  - PB3
	SPI1_NSS  - PA15
*/
#define SPIy			SPI1
#define SPI_PORT		GPIOA
#define SPI1_PIN_MOSI	GPIO_Pin_7
#define SPI1_PIN_MISO	GPIO_Pin_6
#define SPI1_PIN_SCK 	GPIO_Pin_5
#define SPI1_PIN_NSS 	GPIO_Pin_4

unsigned char rxc;
unsigned char txcm = 0x4d;	// M
unsigned char txcs = 0x40;	// @
//	unsigned char txcs = 0x53;	// S

// nRF24L01 Defines
#define nRF24_PORT		GPIOB
#define nRF24_Pin_CE	GPIO_Pin_0
#define nRF24_Pin_CSN	GPIO_Pin_1
#define nRF24_Pin_IRQ	GPIO_Pin_10


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ErrorStatus  HSEStartUpStatus;
FLASH_Status FlashStatus;
uint16_t VarValue = 0;
/* Virtual address defined by the user: 0xFFFF value is prohibited */
uint16_t VirtAddVarTab[NumbOfVar] = {0x0000, 0x0004, 0x0008};
uint16_t addr = 0x000C;

void SysTick_Init(void)
{
	// Cortex-M4 and M3
	// 72 MHz PLL clock divided by 8 and by 9;
	SysTick->LOAD  = 9;
	NVIC_SetPriority(SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);
	SysTick->CTRL  = SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}
void delay()
{
	int i;
	for(i=0;i<1000000;i++){};
}
void delay_us(int micro)
{
	time_us = micro;
	while(time_us>0);
}
void delay_ms(int milli)
{
	time_ms = milli*1000;
	while(time_ms>0);
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

void USART1_Init(uint32_t baudrate){

	/* This is a concept that has to do with the libraries provided by ST
	 * to make development easier the have made up something similar to
	 * classes, called TypeDefs, which actually just define the common
	 * parameters that every peripheral needs to work correctly
	 *
	 * They make our life easier because we don't have to mess around with
	 * the low level stuff of setting bits in the correct registers
	 */
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	/* enable APB2 peripheral clock for USART1
	 * note that only USART1 and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* enable the peripheral clock for the pins used by
	 * USART1, PA9 for TX and PA10 for RX
	 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* This sequence sets up the TX and RX pins
	 * so they work correctly with the USART1 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9; 				// Pin 9 (TX)
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP; 		// the pins are configured as alternate function so the USART peripheral has access to them, Push pull output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_Init(GPIOA, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;				// Pin 10 (RX)
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;  // Input mode floating
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* Now the USART_InitStruct is used to define the
	 * properties of USART1
	 */
	USART_InitStruct.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART1, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting


	/* Here the USART1 receive interrupt is enabled
	 * and the interrupt controller is configured
	 * to jump to the USART1_IRQHandler() function
	 * if the USART1 receive interrupt occurs
	 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		// we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;	// this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			// the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							// the properties are passed to the NVIC_Init function which takes care of the low level stuff

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART1, ENABLE);
}
void USART1_print(char const *s)
{
	while(*s)
	{
		// wait until data register is empty. Transmission complete (TC) bit
		while(!(USART1->SR & USART_SR_TC));
		USART1 -> DR = (*s & (uint16_t)0x00FF);
		s++;
	}
}
void USART1_println(char const *s)
{
	while(*s)
	{
		// wait until data register is empty. Transmission complete (TC) bit
		while(!(USART1->SR & USART_SR_TC));
		USART1 -> DR = (*s & (uint16_t)0x00FF);
		s++;
	}

	while(!(USART1->SR & USART_SR_TC));
	USART1 -> DR = (0x0d & (uint16_t)0x00FF);

	while(!(USART1->SR & USART_SR_TC));
	USART1 -> DR = (0x0a & (uint16_t)0x00FF);
}
void USART1_putc(char c)
{
	// wait until transmittion complete
	while( !(USART1-> SR & USART_SR_TC));
	USART1 -> DR = (c & (uint16_t)0x00FF);

	while( !(USART1-> SR & USART_SR_TC));
	USART1 -> DR = (0x0d & (uint16_t)0x00FF);

	while( !(USART1-> SR & USART_SR_TC));
	USART1 -> DR = (0x0a & (uint16_t)0x00FF);
/*
common ascii codes to know
Char  Dec  Oct  Hex   WhatAreThey
---------------------------------------
(nul)   0 0000 0x00   Null
(ht)    9 0011 0x09   Horizontal Tab
(nl)   10 0012 0x0a   New Line
(vt)   11 0013 0x0b   Vertical Tab
(cr)   13 0015 0x0d   Carriage Return
(sp)   32 0040 0x20   Space
0      48 0060 0x30   zero
A      65 0101 0x41   capital A
a      97 0141 0x61   lowercase a


ascii codes and their escape sequences
ASCII Name   Description     C Escape Sequence
----------------------------------------------
nul          null byte       \0 (zero)
bel          bel character   \a
bs           backspace       \b
ht           horizontal tab  \t
np           formfeed        \f
nl           newline         \n
cr           carriage return \r

end of line sequences
Windows end of line sequence:  \r\n
Unix end of line sequence: \n
Mac end of line sequence: \r

ascii table
Char  Dec  Oct  Hex | Char  Dec  Oct  Hex | Char  Dec  Oct  Hex | Char Dec  Oct   Hex
-------------------------------------------------------------------------------------
(nul)   0 0000 0x00 | (sp)   32 0040 0x20 | @      64 0100 0x40 | `      96 0140 0x60
(soh)   1 0001 0x01 | !      33 0041 0x21 | A      65 0101 0x41 | a      97 0141 0x61
(stx)   2 0002 0x02 | "      34 0042 0x22 | B      66 0102 0x42 | b      98 0142 0x62
(etx)   3 0003 0x03 | #      35 0043 0x23 | C      67 0103 0x43 | c      99 0143 0x63
(eot)   4 0004 0x04 | $      36 0044 0x24 | D      68 0104 0x44 | d     100 0144 0x64
(enq)   5 0005 0x05 | %      37 0045 0x25 | E      69 0105 0x45 | e     101 0145 0x65
(ack)   6 0006 0x06 | &      38 0046 0x26 | F      70 0106 0x46 | f     102 0146 0x66
(bel)   7 0007 0x07 | '      39 0047 0x27 | G      71 0107 0x47 | g     103 0147 0x67
(bs)    8 0010 0x08 | (      40 0050 0x28 | H      72 0110 0x48 | h     104 0150 0x68
(ht)    9 0011 0x09 | )      41 0051 0x29 | I      73 0111 0x49 | i     105 0151 0x69
(nl)   10 0012 0x0a | *      42 0052 0x2a | J      74 0112 0x4a | j     106 0152 0x6a
(vt)   11 0013 0x0b | +      43 0053 0x2b | K      75 0113 0x4b | k     107 0153 0x6b
(np)   12 0014 0x0c | ,      44 0054 0x2c | L      76 0114 0x4c | l     108 0154 0x6c
(cr)   13 0015 0x0d | -      45 0055 0x2d | M      77 0115 0x4d | m     109 0155 0x6d
(so)   14 0016 0x0e | .      46 0056 0x2e | N      78 0116 0x4e | n     110 0156 0x6e
(si)   15 0017 0x0f | /      47 0057 0x2f | O      79 0117 0x4f | o     111 0157 0x6f
(dle)  16 0020 0x10 | 0      48 0060 0x30 | P      80 0120 0x50 | p     112 0160 0x70
(dc1)  17 0021 0x11 | 1      49 0061 0x31 | Q      81 0121 0x51 | q     113 0161 0x71
(dc2)  18 0022 0x12 | 2      50 0062 0x32 | R      82 0122 0x52 | r     114 0162 0x72
(dc3)  19 0023 0x13 | 3      51 0063 0x33 | S      83 0123 0x53 | s     115 0163 0x73
(dc4)  20 0024 0x14 | 4      52 0064 0x34 | T      84 0124 0x54 | t     116 0164 0x74
(nak)  21 0025 0x15 | 5      53 0065 0x35 | U      85 0125 0x55 | u     117 0165 0x75
(syn)  22 0026 0x16 | 6      54 0066 0x36 | V      86 0126 0x56 | v     118 0166 0x76
(etb)  23 0027 0x17 | 7      55 0067 0x37 | W      87 0127 0x57 | w     119 0167 0x77
(can)  24 0030 0x18 | 8      56 0070 0x38 | X      88 0130 0x58 | x     120 0170 0x78
(em)   25 0031 0x19 | 9      57 0071 0x39 | Y      89 0131 0x59 | y     121 0171 0x79
(sub)  26 0032 0x1a | :      58 0072 0x3a | Z      90 0132 0x5a | z     122 0172 0x7a
(esc)  27 0033 0x1b | ;      59 0073 0x3b | [      91 0133 0x5b | {     123 0173 0x7b
(fs)   28 0034 0x1c | <      60 0074 0x3c | \      92 0134 0x5c | |     124 0174 0x7c
(gs)   29 0035 0x1d | =      61 0075 0x3d | ]      93 0135 0x5d | }     125 0175 0x7d
(rs)   30 0036 0x1e | >      62 0076 0x3e | ^      94 0136 0x5e | ~     126 0176 0x7e
(us)   31 0037 0x1f | ?      63 0077 0x3f | _      95 0137 0x5f | (del) 127 0177 0x7f


*/

}
int USART1_available()
{
	return USART1_cnt;
}
char USART1_readByte()
{
	int i;
	char data = 0;

	if(USART1_cnt)
	{
		data = received_string[0];

		for(i=0;i<USART1_cnt;i++)
		{
			received_string[i] = received_string[i+1];
		}
		USART1_cnt--;
	}

	return data;
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

void SPI1_Init_Master()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* Configure SPIy pins: SCK, MISO and MOSI ---------------------------------*/
	GPIO_InitStructure.GPIO_Pin = SPI1_PIN_MISO | SPI1_PIN_SCK | SPI1_PIN_MOSI;
	/* Configure SCK and MOSI pins as Alternate Function Push Pull */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_PORT, &GPIO_InitStructure);

//	GPIO_InitStructure.GPIO_Pin = SPI1_PIN_NSS;
//	/* Configure SCK and MOSI pins as Alternate Function Push Pull */
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(SPI_PORT, &GPIO_InitStructure);

//	GPIO_InitStructure.GPIO_Pin = SPI1_PIN_MISO;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(SPI_PORT, &GPIO_InitStructure);

//	SPI1_REMAP
//	GPIO_PinRemapConfig(GPIO_Remap_SPI1, ENABLE);
	// or
//	AFIO -> EVCR |= AFIO_EVCR_PORT_PA | AFIO_EVCR_PIN_PX5;
//	AFIO -> EVCR |= AFIO_EVCR_EVOE;
//	AFIO -> MAPR |= AFIO_MAPR_SPI1_REMAP;

	SPI_InitTypeDef   SPI_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	/* SPIy Config -------------------------------------------------------------*/
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		// Slave mode selected
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;	// byte size
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;			// clock is low when idle
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;		// data sampled at first edge
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;// | SPI_NSSInternalSoft_Set;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);

	SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Set);
//	SPI_SSOutputCmd(SPI1, ENABLE);

//	Interrupt ------
//	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
//	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE);
//	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_OVR, ENABLE);
//	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_ERR, ENABLE);

//	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)
//	NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;				// we want to configure the USART1 interrupts
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;		// this sets the subpriority inside the group
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				// the USART1 interrupts are globally enabled
//	NVIC_Init(&NVIC_InitStructure);								// the properties are passed to the NVIC_Init function which takes care of the low level stuff
//	Interrupt end ---------

	SPI_Cmd(SPI1, ENABLE);
}
void SPI1_Init_Slave()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* Configure SPIy pins: SCK, MISO and MOSI ---------------------------------*/
	GPIO_InitStructure.GPIO_Pin = SPI1_PIN_MOSI | SPI1_PIN_MISO;
	/* Configure SCK and MOSI pins as Alternate Function Push Pull */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SPI1_PIN_SCK;
	/* Configure SCK and MOSI pins as Alternate Function Push Pull */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_PORT, &GPIO_InitStructure);

//	SPI1_REMAP
//	GPIO_PinRemapConfig(GPIO_Remap_SPI1, ENABLE);
	// or
//	AFIO -> EVCR |= AFIO_EVCR_PORT_PA | AFIO_EVCR_PIN_PX5;
//	AFIO -> EVCR |= AFIO_EVCR_EVOE;
//	AFIO -> MAPR |= AFIO_MAPR_SPI1_REMAP;

	SPI_InitTypeDef   SPI_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	/* SPIy Config -------------------------------------------------------------*/
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//SPI_Direction_1Line_Tx;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;		// Slave mode selected
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;	// byte size
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;			// clock is low when idle
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;		// data sampled at second edge
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);
	SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Reset);

//	Interrupt ------
	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE);
//	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_OVR, ENABLE);
	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_ERR, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)
	NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;				// we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;		// this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				// the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);								// the properties are passed to the NVIC_Init function which takes care of the low level stuff
//	Interrupt end ---------

	/* Add IRQ vector to NVIC */
	/* PB12 is connected to EXTI_Line12, which has EXTI15_10_IRQn vector */
//	NVIC_InitTypeDef NVIC_InitStruct;
//
//	NVIC_InitStruct.NVIC_IRQChannel = SPI1_IRQn; //EXTI15_10_IRQn;
//	/* Set priority */
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
//	/* Set sub priority */
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x03;
//	/* Enable interrupt */
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	/* Add to NVIC */
//	NVIC_Init(&NVIC_InitStruct);

	SPI_Cmd(SPI1, ENABLE);
}
void SPI1_print(char const *s)
{
	while(*s)
	{
		// wait until data register is empty. Transmission complete (TC) bit
		while( !(SPI1-> SR & 0x00000002) );
			SPI1 -> DR = (*s & (uint16_t)0x01FF);
		s++;
	}
}
unsigned char SPI1_transfer_Byte(uint16_t Data)
{
	unsigned char Data2 = 0;
	// Verify if transmit buffer is empty TXE bit
	while(!(SPI1-> SR & SPI_I2S_FLAG_TXE) );
	SPI1 -> DR = Data;
//	SPI1 -> DR = (Data & (uint16_t)0x00FF);
	while(!(SPI1-> SR & SPI_I2S_FLAG_RXNE) );
	Data2 = SPI1 -> DR;

	return Data2;
}
unsigned char SPI1_writeByte(uint16_t Data)
{
	unsigned char Data2 = 0;
	// Verify if transmit buffer is empty TXE bit
	while(!(SPI1-> SR & SPI_I2S_FLAG_TXE) );
	SPI1 -> DR = Data;
//	SPI1 -> DR = (Data & (uint16_t)0x00FF);
	while(!(SPI1-> SR & SPI_I2S_FLAG_RXNE) );
	Data2 = SPI1 -> DR;
	return Data2;
}
unsigned char SPI1_readByte()
{
	int i;
	char data = 0;

	if(SPI1_cnt)
	{
		data = received_string[0];

		for(i=0;i<SPI1_cnt;i++)
		{
			SPI_received_string[i] = SPI_received_string[i+1];
		}
		SPI1_cnt--;
	}

	return data;
}
int SPI1_available()
{
	return SPI1_cnt;
}
void SPI1_master_demo01()
{
	// Slave/Master with interruption
	if(SPI1_flag_rx)
	{
		SPI1_flag_rx = 0;
		USART1_print("Received: ");
		USART1_putc(SPI1_rxd);
		USART1_print("Transmitted: ");
		USART1_putc(SPI1_txd);
		LED_green_toogle();
	}
	if(SPI1_flag_err)
	{
		SPI1_flag_err = 0;
		USART1_println("Err");
	}


//		// Master
//		if(SPI1_txd > 0x5a)
//		{
//			SPI1_txd = 0x41;	// A
//		}
//
//		if(SPI1_flag_rx)
//		{
//			SPI1_flag_rx = 0;
//			USART1_print("Receiving: ");
//			USART1_putc(SPI1_rxd);
//		}
//		USART1_print("Transmitting: ");
//		USART1_putc(SPI1_txd);
//		SPI1_writeByte(SPI1_txd);

		// All data transmitted/received but SPI may be busy so wait until done.
		while(SPI1->SR & SPI_I2S_FLAG_BSY);

		// TX: 1- empty
		if(SPI1 -> SR & (uint16_t) SPI_SR_TXE)
		{
			USART1_print("Transmitting: ");
			SPI1 -> DR = txcm;
			//    Wait until the data has been transmitted.
			while (!(SPI1->SR & SPI_I2S_FLAG_TXE));
			USART1_putc(txcm);
		}

		// Wait for any data on MISO pin to be received.
		while (!(SPI1->SR & SPI_I2S_FLAG_RXNE));
		// RX: 1- not empty
		if(SPI1 -> SR & (uint16_t) SPI_SR_RXNE)
		{
			LED_green_toogle();
			rxc = SPI1 -> DR;
			USART1_print("Receiving: ");
			USART1_putc(rxc);
		//			rxc++;
		//			txc = rxc;
		}
		delay_ms(500);
}
void SPI1_slave_demo01()
{
	// Slave/Master with interruption
	if(SPI1_flag_rx)
	{
		SPI1_flag_rx = 0;
		USART1_print("Received: ");
		USART1_putc(SPI1_rxd);
		USART1_print("Transmitted: ");
		USART1_putc(SPI1_txd);
		LED_green_toogle();
	}
	if(SPI1_flag_err)
	{
		SPI1_flag_err = 0;
		USART1_println("Err");
	}


	// Slave Polling
	//	TX: 1- empty
	if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_TXE) == SET)
	{
		rxc = SPI1 -> DR;
//			while (!(SPI1->SR & SPI_I2S_FLAG_RXNE));
		USART1_print("Receiving: ");
		USART1_putc(rxc);
//			rxc++;
//			txcs = rxc;
	}

	// RX: 1- not empty
	if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) == SET)
	{
		SPI1 -> DR = txcs;
//			while (!(SPI1->SR & SPI_I2S_FLAG_TXE));
		USART1_print("Transmitting: ");
		USART1_putc(txcs);
		LED_green_toogle();
	}
//		if(err_cnt > 10)
//		{
//			err_cnt=0;
//			SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Set);
//			delay_ms(100);
//			SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Reset);
//		}
}

uint8_t address[] = { 0xCC,0xCE,0xCC,0xCE,0xCC };
uint8_t flag_nRF24_IRQ = 0;
typedef enum states0 {
	powerDown,
	standByI,
	modeRX,
	modeTX,
	standByII
}stateMachine;
/*
 * 0 = power down
 * 1 = power up
 * 2 = rx mode
 * 3 = tx mode
 */
uint8_t stateMode = 0;
uint8_t nRF24_rx_addr_p0[5];
uint8_t nRF24_rx_addr_p0_d[5] =	{0xB3, 0xB4, 0xB5, 0xB6, 0x05};

//uint8_t nRF24_tx_addr[5];
//uint8_t nRF24_tx_addr_d[5] = 	{0x01, 0x02, 0x03, 0x04, 0x05};
uint8_t vect[2] = {0x41, 0x42};
uint8_t nRF24_flag_send_cont =0;
void nRF24_CE(FunctionalState NewState)
{
	assert_param(IS_FUNCTIONAL_STATE(NewState));
	if (NewState != DISABLE)
	{
		GPIO_SetBits(nRF24_PORT, nRF24_Pin_CE);
		delay_us(130);
	}
	else
	{
		GPIO_ResetBits(nRF24_PORT, nRF24_Pin_CE);
		delay_us(10);
	}
}
void nRF24_CSN(FunctionalState NewState)
{
	assert_param(IS_FUNCTIONAL_STATE(NewState));
	if (NewState != DISABLE)
	{
		delay_us(3);
		GPIO_SetBits(nRF24_PORT, nRF24_Pin_CSN);
	}
	else
	{
		GPIO_ResetBits(nRF24_PORT, nRF24_Pin_CSN);
		delay_us(3);
	}
}
void nRF24_Configure()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph, ENABLE);

	/* Configure pins:  CE and CSN				*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure pins:  IRQ						*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	SPI1_Init_Master();
}
uint8_t nRF24_read_register(uint8_t reg)
{
	uint8_t nRF24_rx;

	nRF24_CSN(DISABLE);
	SPI1_transfer_Byte(R_REGISTER | ( REGISTER_MASK & reg ) );
	nRF24_rx = SPI1_transfer_Byte(NOP);
	nRF24_CSN(ENABLE);

	return nRF24_rx;
}
uint8_t nRF24_read_register_buf(uint8_t reg, uint8_t* buf, uint8_t length)
{
	uint8_t status;
	nRF24_CSN(DISABLE);
	status = SPI1_transfer_Byte(R_REGISTER | ( REGISTER_MASK & reg ) );
	while ( length-- ){
		*buf++ = SPI1_transfer_Byte(NOP);
	}
	nRF24_CSN(ENABLE);

	return status;
}
uint8_t nRF24_write_register(uint8_t reg, uint8_t value)
{
	uint8_t status;
	nRF24_CSN(DISABLE);
	status = SPI1_transfer_Byte(W_REGISTER | ( REGISTER_MASK & reg ) );
	SPI1_transfer_Byte(value);
	nRF24_CSN(ENABLE);

	return status;
}
uint8_t nRF24_write_register_buf(uint8_t reg, uint8_t* buf, uint8_t length)
{
	uint8_t status;
	nRF24_CSN(DISABLE);
	status = SPI1_transfer_Byte(W_REGISTER | ( REGISTER_MASK & reg ) );
	while(length--){
		SPI1_transfer_Byte(*buf++);
	}
	nRF24_CSN(ENABLE);

	return status;
}
void nRF24_PWR(FunctionalState state)
{
	uint8_t regValue;

	regValue = nRF24_read_register(NRF_CONFIG);
	if(state == ENABLE)
	{
		regValue |=  (1 << PWR_UP);
	}
	else
	{
		regValue &= ~(1 << PWR_UP);
	}
	nRF24_write_register(NRF_CONFIG, regValue);

	delay_ms(5);
}
uint8_t nRF24_get_pipe_payload_len(uint8_t pipe)
{
	uint8_t length, addr;
	addr = RX_PW_P0 + pipe;

	// read payload size
	length = nRF24_read_register(addr);

	return length;
}
void nRF24_set_RF_PWR(uint8_t RF_power)
{
	uint8_t regValue;
	regValue = nRF24_read_register(RF_SETUP);
	switch(RF_power)
	{
		case RF_PWR_18dBm:
			regValue &= ~(1 << RF_PWR_high);
			regValue &= ~(1 << RF_PWR_low);
			break;

		case RF_PWR_12dBm:
			regValue &= ~(1 << RF_PWR_high);
			regValue |=  (1 << RF_PWR_low);
			break;

		case RF_PWR_6dBm:
			regValue |=  (1 << RF_PWR_high);
			regValue &= ~(1 << RF_PWR_low);
			break;

		case RF_PWR_0dBm:
			regValue |=  (1 << RF_PWR_high);
			regValue |=  (1 << RF_PWR_low);
			break;
	}
	regValue = nRF24_write_register(RF_SETUP, regValue);
}
void nRF24_set_RX_mode(FunctionalState state)
{
	if(state == ENABLE)
	{
		uint8_t regValue;

		// Set PRIM_RX to high;
		regValue = nRF24_read_register(NRF_CONFIG);

		sprintf(buffer,"aftmode:%d",regValue);
		USART1_println(buffer);

		regValue |= (1 << PRIM_RX);
		nRF24_write_register(NRF_CONFIG, regValue);

		// Just clear these bits
		nRF24_write_register(NRF_STATUS, (1 << RX_DR) | (1<<TX_DS) | (1<<MAX_RT));

		// Set CE to high and wait 130 us;
		nRF24_CE(ENABLE);
//		delay_us(130);

		sprintf(buffer,"rxmodeON:%d",regValue);
		USART1_println(buffer);

		stateMode = 2;
	}
	else
	{
		nRF24_CE(DISABLE);
//		delay_us(130);
		sprintf(buffer,"rxmodeOFF");
		USART1_println(buffer);

		stateMode = 1;
	}
}
void nRF24_set_TX_mode(FunctionalState state)
{
	if(state == ENABLE)
	{
		uint8_t regValue;

		// Set PRIM_RX to low (TX mode);
		regValue = nRF24_read_register(NRF_CONFIG);
		regValue &= ~(1 << PRIM_RX);
		nRF24_write_register(NRF_CONFIG, regValue);

		sprintf(buffer,"CONFIG: %d", regValue);
		USART1_println(buffer);
		USART1_println("sent!");
		// Set CE to high and wait 130 us;
		nRF24_CE(ENABLE);
//		delay_us(130);

		stateMode = 3;
	}
	else
	{
		nRF24_CE(DISABLE);
		delay_us(130);

		stateMode = 1;
	}

}
void nRF24_set_RX_ADDRn(uint8_t pipe, uint8_t *addr_rx, uint8_t addr_length)
{
	uint8_t regAddr = RX_ADDR_P0 + pipe;
	int i;

	nRF24_CSN(DISABLE);
	SPI1_transfer_Byte(W_REGISTER | ( REGISTER_MASK & regAddr ) );
	for(i=0; i<addr_length; i++)
	{
		SPI1_transfer_Byte(addr_rx[i]);
	}
	nRF24_CSN(ENABLE);

#ifdef debug
	for(i=0; i<addr_length; i++)
	{
		sprintf(buffer, "%d ", addr_rx[i]);
		USART1_print(buffer);
	}
	USART1_println("");
#endif
}
void nRF24_get_RX_ADDRn(uint8_t pipe, uint8_t *addr_rx, uint8_t addr_length)
{
	uint8_t regAddr = RX_ADDR_P0 + pipe;
	int i;

	nRF24_CSN(DISABLE);
	SPI1_transfer_Byte(R_REGISTER | ( REGISTER_MASK & regAddr) );
	for(i=0; i<addr_length; i++)
	{
		addr_rx[i] = SPI1_transfer_Byte(NOP);
	}
	nRF24_CSN(ENABLE);

	for(i=0; i<addr_length; i++)
	{
		sprintf(buffer, "%d ", addr_rx[i]);
		USART1_print(buffer);
	}
	USART1_println("");
}
void nRF24_set_TX_ADDR(uint8_t *addr, uint8_t addr_length)
{
	uint8_t regAddr = TX_ADDR;
	int i;

	nRF24_CSN(DISABLE);
	SPI1_transfer_Byte(W_REGISTER | ( REGISTER_MASK & regAddr ) );
	for(i=0; i<addr_length; i++)
	{
		SPI1_transfer_Byte(addr[i]);
	}
	nRF24_CSN(ENABLE);
}
void nRF24_get_TX_ADDR(uint8_t *addr_tx, uint8_t addr_length)
{
	nRF24_CSN(DISABLE);
	SPI1_transfer_Byte(R_REGISTER | ( REGISTER_MASK & TX_ADDR) );
	int i;
	for(i=0; i<addr_length; i++)
	{
//		nRF24_tx_addr[i] = SPI1_transfer_Byte(NOP);
		addr_tx[i] = SPI1_transfer_Byte(NOP);
	}
	nRF24_CSN(ENABLE);

	for(i=0; i<addr_length; i++)
	{
		sprintf(buffer, "%d ", addr_tx[i]);
		USART1_print(buffer);
	}
	USART1_println("");
}
uint8_t nRF24_write_payload(uint8_t *vect, uint8_t length)
{
	uint8_t status;
	int i;
	nRF24_CSN(DISABLE);
	status = SPI1_transfer_Byte(W_TX_PAYLOAD);
	for(i=0; i<length; i++)
	{
		SPI1_transfer_Byte(vect[i]);
	}
	nRF24_CSN(ENABLE);

	return status;
}
uint8_t nRF24_read_payload(uint8_t *vect_rx, uint8_t length)
{
	int i, status;

	nRF24_CSN(DISABLE);
	status = SPI1_transfer_Byte(R_RX_PAYLOAD);

	for(i=0; i<length; i++)
	{
		vect_rx[i] = SPI1_transfer_Byte(NOP);
	}
	nRF24_CSN(ENABLE);

	return status;
}
void nRF24_flush_RX(void)
{
	nRF24_CSN(DISABLE);
	SPI1_transfer_Byte(FLUSH_RX);
	nRF24_CSN(ENABLE);
}
void nRF24_flush_TX(void)
{
	nRF24_CSN(DISABLE);
	SPI1_transfer_Byte(FLUSH_TX);
	nRF24_CSN(ENABLE);
}
void nRF24_set_CRC(FunctionalState state)
{
	uint8_t regValue;
	regValue = nRF24_read_register(NRF_CONFIG);
	if(state == ENABLE)
	{
		regValue |=  (1 << EN_CRC);
	}
	else
	{
		regValue &= ~(1 << EN_CRC);
	}
	regValue = nRF24_write_register(NRF_CONFIG, regValue);
}
void nRF24_set_250kbps()
{
	uint8_t regValue;
	regValue  =  nRF24_read_register(RF_SETUP);
	regValue |=  (1<<RF_DR_LOW);
	regValue &= ~(1<<RF_DR_HIGH);
	regValue &= ~(1<<0);	// Dont care bit
	regValue = nRF24_write_register(RF_SETUP, regValue);
}
void nRF24_set_1Mbps()
{
	uint8_t regValue;
	regValue = nRF24_read_register(RF_SETUP);
	regValue = (regValue & 0b11010111);	// Reset bit 5; Reset bit 3;
	regValue = nRF24_write_register(RF_SETUP, regValue);
}
void nRF24_set_2Mbps()
{
	uint8_t regValue;
	regValue = nRF24_read_register(RF_SETUP);
	regValue = (regValue & 0b11011111) | 0b00001000;	// Reset bit 5; Set bit 3;
	regValue = nRF24_write_register(RF_SETUP, regValue);
}
int nRF24_get_RPD()
{
	return nRF24_read_register(RPD);
}
void nRF24_set_pipe_state(uint8_t pipe, FunctionalState state)
{
	uint8_t regValue;

	// Configure data pipe n
	regValue = nRF24_read_register(EN_RXADDR);
	if(state == ENABLE)
	{
		regValue |=  (1 << pipe);
	}
	else
	{
		regValue &= ~(1 << pipe);
	}
	nRF24_write_register(EN_RXADDR, regValue);
}
void nRF24_set_PowerUp()
{
	uint8_t regValue;
	regValue = nRF24_read_register(NRF_CONFIG);
	regValue |= (1 << PWR_UP);
	regValue = nRF24_write_register(NRF_CONFIG, regValue);

	stateMode = 1;

	delay_ms(5);
}
void nRF24_set_PowerDown()
{
	nRF24_CE(DISABLE);

	uint8_t regValue;
	regValue = nRF24_read_register(NRF_CONFIG);
	regValue &= ~(1 << PWR_UP);
	regValue = nRF24_write_register(NRF_CONFIG, regValue);

	stateMode = 0;
}
void nRF24_set_ACK(uint8_t pipe, FunctionalState state)
{
	uint8_t regValue;
	regValue = nRF24_read_register(EN_AA);

	if(state == ENABLE)
	{
		regValue |=  (1 << pipe);
	}
	else
	{
		regValue &= ~(1 << pipe);
	}
	regValue = nRF24_write_register(EN_AA, regValue);
}
void nRF24_set_Auto_Retransmit(uint8_t wait_time, uint8_t arc)
{
	uint8_t regValue;
	regValue = nRF24_read_register(SETUP_RETR);
	regValue |= ((wait_time << ARD) | (arc << ARC));
	regValue = nRF24_write_register(SETUP_RETR, regValue);
}
void nRF24_set_payload_width(uint8_t pipe, uint8_t length)
{
	uint8_t regAddr = RX_PW_P0 + pipe;
	nRF24_write_register(regAddr, length);
}
void nRF24_set_ACK_disable()
{
	// Disable all auto acknowledge (default 0x3F)
	nRF24_write_register(EN_AA, 0x00);
	// Disable all auto retransmit (default 0x03)
	nRF24_write_register(SETUP_RETR, 0x00);
}
uint8_t nRF24_get_payload_width(uint8_t pipe)
{
	uint8_t regAddr = RX_PW_P0 + pipe, length;
	length = nRF24_read_register(regAddr);

	return length;
}
void nRF24_fill_payload_TX(uint8_t *payload_TX, uint8_t length)
{
	nRF24_write_register_buf(W_TX_PAYLOAD, payload_TX, length);
}
int nRF24_IRQ()
{
	//	if(!flag_nRF24_IRQ)
	if(!GPIO_ReadInputDataBit(nRF24_PORT, nRF24_Pin_IRQ))
	{
		flag_nRF24_IRQ = 1;
		return 1;
	}
	else
	{
		flag_nRF24_IRQ = 0;
		return 0;
	}
}
uint8_t nRF24_poll_RX()
{
	uint8_t regValue;//, pipe = 7;

//	delay_ms(100);

	regValue = nRF24_read_register(NRF_STATUS);
	if(regValue & (1<<RX_DR))
	{
		USART1_println("GOT1!");
		nRF24_write_register(NRF_STATUS, (regValue | (1<<RX_DR)));
	}


	regValue = nRF24_read_register(FIFO_STATUS);
//	pipe = ((regValue >> RX_P_NO) & 0x07);
//	sprintf(buffer,"%d", regValue);
//	USART1_println(buffer);
//	if(regValue & (1 << RX_DR))
	if(!(regValue & (1 << RX_EMPTY)))
	{
		USART1_println("GOT2!");
		return 1;
//		USART1_println("Got!");
//		if(pipe < 6)
//		{
//			return pipe;
//		}
//		else
//			return 0;
	}
	else
	{
		return 0;
	}
}
void nRF24_openReadingPipe(void)//(uint8_t pipe, const uint8_t *addr)
{

}
void nRF24_reset(void)
{
	nRF24_write_register(NRF_CONFIG, 0x08);
//	nRF24_write_register(EN_AA, 0x3F);

	nRF24_write_register(RF_SETUP, 0x0E);
}
void nRF24_Init()
{
	SPI1_Init_Master();
	nRF24_Configure();
	nRF24_CSN(ENABLE);
	nRF24_CE(DISABLE);
	nRF24_set_PowerUp();

	nRF24_set_CRC(DISABLE);
	nRF24_set_ACK_disable();
	nRF24_set_250kbps();
	nRF24_set_RF_PWR(RF_PWR_0dBm);
	nRF24_set_payload_width(0, 2);
	nRF24_set_payload_width(1, 2);
	nRF24_set_RX_ADDRn(0, nRF24_rx_addr_p0_d, 5);	// Set rx address;
	nRF24_set_TX_ADDR(nRF24_rx_addr_p0_d, 5);		// Set destination address;
	nRF24_set_pipe_state(0, ENABLE);
	nRF24_set_pipe_state(1, DISABLE);

//	uint8_t len = nRF24_get_payload_width(0);
//	sprintf(buffer,"len0=%d", len);
//	USART1_println(buffer);

//	nRF24_flush_RX();								// flush rx;
//	nRF24_set_RX_mode(ENABLE);						// Set PRIM bit
}
void nRF24_test()
{
	// Bit 1: PWR_UP =1
	nRF24_write_register(NRF_CONFIG, 0x02);
	// Channel 5
	nRF24_write_register(RF_CH, 0x00);
	// Bit 7: CONT_WAVE = 1; Bit 4: PLL_LOCK = 1
	nRF24_write_register(RF_SETUP, 0x9F);

	nRF24_CE(ENABLE);
}
void nRF24_summary()
{
	uint8_t regValue;

	regValue = nRF24_read_register(NRF_CONFIG);
	sprintf(buffer,"PWR_UP: %d", (regValue & (1<<PWR_UP)));
	USART1_println(buffer);
}
void nRF24_send_2bytes()
{
	/* Transmitter */

	// Write bytes to FIFO and it will define the payload width.
	nRF24_write_payload(vect, 2);
	if(vect[1] < 0x5b)
	{
		vect[0]++;
		vect[1]++;
	}
	else
	{
		vect[0] = 0x41;
		vect[1] = 0x42;
	}

	nRF24_set_RX_mode(DISABLE);
	nRF24_set_TX_mode(ENABLE);

//			while((~nRF24_read_register(NRF_STATUS)) & (1 << TX_DS))
//			{
//				USART1_println("FU");
//			}
//
//			uint8_t regValue = nRF24_read_register(NRF_STATUS);
//			regValue |= (1 << TX_DS);
//			nRF24_write_register(NRF_STATUS, regValue);
//
//			nRF24_flush_TX();								// flush tx;
//			while((~nRF24_read_register(FIFO_STATUS)) & (1 << TX_EMPTY))
//			{
//				USART1_println("WAIT!");
//			}
	sprintf(buffer,"sent:%c %c", vect[0], vect[1]);
	USART1_println(buffer);
	nRF24_set_TX_mode(DISABLE);
//			delay_ms(10);
	nRF24_set_RX_mode(ENABLE);
}

void demo01(void)	// Put this into the while(1) loop function
{
	SystemInit();
//	SystemCoreClockUpdate();

	ADC1_Init();
	IO_Init();
	USART1_Init(9600); // initialize USART1 @ 9600 baud
	SysTick_Init();

	// ADC Variables
	const uint16_t V25 = 1750;// when V25=1.41V at ref 3.3V
	const uint16_t Avg_Slope = 5; //when avg_slope=4.3mV/C at ref 3.3V
	uint16_t TemperatureC;
	int ADC_Value=0;
	int vd = 0, d1, d2;
	double Voltage = 0.0, f1;
	char buffer[50];

	while(1)
	{
		LED_green(1);
		ADC_Value = ADC_Read(ADC_Channel_16);
	//		printf("\r\n ADC value: %d \r\n", AD_value);
		TemperatureC = (uint16_t)((V25-ADC_Value)/Avg_Slope+25);
	//		printf("Temperature: %d%cC\r\n", TemperatureC, 176);

		vd = ADC_Read(ADC_Channel_17);
		Voltage = (vd-1490.0+4096.0)/4096.0*3.3;

	//		 V = 1.205*4095/ADCval

		d1 = (int) Voltage;
		f1 = Voltage - d1;
		d2 = trunc(f1*1000);

		sprintf(buffer,"D: %d, T: %d, V: %d.%04d\r\n",ADC_Value, TemperatureC, d1, d2);
	//		USART_puts(USART1, "Fuck!\r\n"); // just send a message to indicate that it works

		LED_green(0);
		delay_ms(500);
	}
}

void comm_Bluetooth()
{
	// Rx - Always listening
//	uint8_t j2 =0;
	while((USART1_available()>0))	// Reading from serial
	{
		inChar = USART1_readByte();

		if(inChar=='$')
		{
			j2 = 0;
			flag_frameStartBT = 1;
//			Serial.println("Frame Start!");
		}

		if(flag_frameStartBT)
			sInstrBluetooth[j2] = inChar;

//		sprintf(buffer,"J= %d",j2);
//		Serial.println(buffer);

		j2++;

		if(j2>=sizeof(sInstrBluetooth))
		{
			memset(sInstrBluetooth,0,sizeof(sInstrBluetooth));
			j2=0;
//			Serial.println("ZEROU! sIntr BLuetooth Buffer!");
		}

		if(inChar==';')
		{
//			Serial.println("Encontrou ; !");
			if(flag_frameStartBT)
			{
//				Serial.println("Frame Stop!");
				flag_frameStartBT = 0;
				rLength = j2;
				j2 = 0;
				enableTranslate_Bluetooth = 1;
			}
		}
	}
//	flag_frameStart = 0;

	if(enableTranslate_Bluetooth)
	{
//		Serial.println("enableTranslate_Bluetooth");
		enableTranslate_Bluetooth = 0;

		char *pi0, *pf0;
		pi0 = strchr(sInstrBluetooth,'$');
		pf0 = strchr(sInstrBluetooth,';');

		if(pi0!=NULL)
		{
			uint8_t l0=0;
			l0 = pf0 - pi0;

			int i;
			for(i=1;i<=l0;i++)
			{
				sInstr[i-1] = pi0[i];
//				Serial.write(sInstr[i-1]);
			}
			memset(sInstrBluetooth,0,sizeof(sInstrBluetooth));
	//		Serial.println(sInstr);
//			USART1_println(sInstr);

			enableDecode = 1;
		}
		else
		{
//			Serial.println("Err");
			USART1_putc(pi0[0]);
			USART1_putc(pf0[0]);
		}
	}
}
void handleMessage()
{
	if(enableDecode)
	{
		enableDecode = 0;

		// Getting the opcode
//		aux[0] = sInstr[0];
//		aux[1] = sInstr[1];
//		aux[2] = '\0';
//		opcode = (uint8_t) atoi(aux);

		uint8_t reg_value, reg_addr, rxc;

		if(sInstr[2] == ':' && sInstr[6] == ';')
		{
			aux[0] = sInstr[0];
			aux[1] = sInstr[1];
			aux[2] = '\0';
			reg_addr  = (uint8_t) atoi(aux);


			aux2[0] = sInstr[3];
			aux2[1] = sInstr[4];
			aux2[2] = sInstr[5];
			aux2[3] = '\0';
			reg_value  = (uint8_t) atoi(aux2);

			rxc = nRF24_write_register(reg_addr, reg_value);
			sprintf(buffer, "w:%d", rxc);
			USART1_println(buffer);
		}
		else if(sInstr[0] == 'r' && sInstr[1] == ';')
		{
			nRF24_get_RX_ADDRn(0, nRF24_rx_addr_p0, 5);
		}
		else if(sInstr[0] == 'w' && sInstr[1] == ';')
		{
			nRF24_set_RX_ADDRn(0, nRF24_rx_addr_p0_d, 5);
		}
		else if(sInstr[0] == 'c' && sInstr[1] == ':' && sInstr[3] == ';')
		{
			if(sInstr[2] == '1')
			{
				nRF24_test();
				USART1_println("test");
			}
			else
			{
				nRF24_reset();
				USART1_println("RST");
			}
		}
		else if(sInstr[0] == 't' && sInstr[1] == ';')
		{
			nRF24_send_2bytes();
		}
		else if(sInstr[0] == 't' && sInstr[1] == ':' && sInstr[2] == '1' && sInstr[3] == ';')
		{
			nRF24_flag_send_cont = 1;
		}
		else if(sInstr[0] == 't' && sInstr[1] == ':' && sInstr[2] == '0' && sInstr[3] == ';')
		{
			nRF24_flag_send_cont = 0;
		}
		else if(sInstr[0] == 'p' && sInstr[1] == 'u' && sInstr[2] == ';')
		{
			nRF24_set_PowerUp();
		}
		else if(sInstr[0] == 'p' && sInstr[1] == 'd' && sInstr[2] == ';')
		{
			USART1_println("PowerDown");
			nRF24_set_PowerDown();
		}
		else if(sInstr[0] == 'f' && sInstr[1] == 't' && sInstr[2] == ';')
		{
			nRF24_flush_TX();
		}
		else if(sInstr[0] == 'f' && sInstr[1] == 'r' && sInstr[2] == ';')
		{
			nRF24_flush_RX();
		}
		else if(sInstr[0] == 'r' && sInstr[1] == 'x' && sInstr[2] == ':' && sInstr[4] == ';')
		{
			if(sInstr[3] == '1')
			{
				nRF24_set_RX_mode(ENABLE);
			}
			else
			{
				nRF24_set_RX_mode(DISABLE);
			}
		}
		else if(sInstr[2] == ';')
		{
			aux[0] = sInstr[0];
			aux[1] = sInstr[1];
			aux[2] = '\0';

			reg_addr  = (uint8_t) atoi(aux);
			rxc = nRF24_read_register(reg_addr);
			sprintf(buffer, "r:%d", rxc);
			USART1_println(buffer);
		}

		switch (opcode)
		{
//			case 0:
//			{
//				if(sInstr[1] == ';')
//				{
//					rxc = nRF24_read_register(reg_addr);
//					sprintf(buffer, "r:%d", rxc);
//					USART1_println(buffer);
//				}
//			}
//			break;
//			case 1:
//			{
//				if(sInstr[1] == ':' && sInstr[4] == ';')
//				{
//					aux[0] = sInstr[2];
//					aux[1] = sInstr[3];
//					aux[2] = '\0';
//
//					byte_value  = (uint8_t) atoi(aux);
//					rxc = nRF24_write_register(0x00, 0x00);
//					sprintf(buffer, "w1:%d", rxc);
//					USART1_println(buffer);
//				}
//			}
//			break;
//			case 2:
//			{
//				if(sInstr[1] == ':' && sInstr[4] == ';')
//				{
//					aux[0] = sInstr[2];
//					aux[1] = sInstr[3];
//					aux[2] = '\0';
//
//					byte_value  = (uint8_t) atoi(aux);
//					rxc = nRF24_write_register(0x00, byte_value);
//					sprintf(buffer, "w2:%d", rxc);
//					USART1_println(buffer);
//				}
//			}
//			break;
//			default:
//			{
//				USART1_println("fuckdefault!");
//			}
//			break;

// -----------------------------------------------------------------
//			case 0:		// Set motor ON/OFF
//			{
//				if

//			}
			case 30:		// Set motor ON/OFF
			{
				LED_green_toogle();
//				uint8_t motorCommand;
//				aux[0] = '0';
//				aux[1] = sInstr[1];
//				aux[2] = '\0';
//				motorCommand = (uint8_t) atoi(aux);
//
//				if(motorCommand)
//				{
//					LED_green(1);
//					USART1_println("ON");
//				}
//				else
//				{
//					LED_green(0);
//					USART1_println("OFF");
//				}

//				if (motorCommand && (!motorStatus))
//					motor_start();
//				else
//					motor_stop();
//
//				summary_Print(3);
			}
			break;

//			case 4: // EEPROM Write -> $4:h:1234;
//			{
//				if(sInstr[1]==':' && sInstr[3]==':' && sInstr[8]==';')
//				{
//					char aux2[5];
//					aux2[0] = sInstr[4];
//					aux2[1] = sInstr[5];
//					aux2[2] = sInstr[6];
//					aux2[3] = sInstr[7];
//					aux2[4] = '\0';
//					uint16_t valueReceived = (uint16_t) atoi(aux2);
//
//					aux[0] = '0';
//					aux[1] = sInstr[2];
//					aux[2] = '\0';
//
//					uint8_t ind = (uint8_t) atoi(aux);
//
//					EE_WriteVariable(VirtAddVarTab[ind], valueReceived);
//
//					/* Store 800 values of Variable3 in EEPROM */
////					for (VarValue = 0; VarValue < 800; VarValue++)
////					{
////						EE_WriteVariable(VirtAddVarTab[2], VarValue);
////					}
//
//					uint16_t valueStored;
//					EE_ReadVariable(VirtAddVarTab[ind], &valueStored);
//					sprintf(buffer,"%d", valueStored);
//					USART1_println(buffer);
//				}
//				else if(sInstr[1]==';')
//				{
//					uint16_t valueRead;
//					int i;
//					for(i=0; i<3; i++)
//					{
//						EE_ReadVariable(VirtAddVarTab[i], &valueRead);
//						sprintf(buffer,"%d: %d", VirtAddVarTab[i], valueRead);
//						USART1_println(buffer);
//					}
//
//					char name2[21];
//					uint16_t v;
//					for(i=0; i<20; i++)
//					{
//						EE_ReadVariable(addr, &v);
//						name2[i] = (char) v;
//					}
//					USART1_println(name2);
//					USART1_println("Fuck");
//				}
//
//				break;
//			}
// -----------------------------------------------------------------
		}
		memset(sInstr,0,sizeof(sInstr));	// Clear all vector;
	}
}
void comm_SPI1()
{
	char inChar2;
	while((SPI1_available()>0))	// Reading from serial
	{
		USART1_println("i: ");
		inChar2 = SPI1_readByte();
		USART1_putc(inChar2);
	}
	if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) == SET)
	{
		USART1_println("B");
		inChar2 = SPI1_readByte();
		USART1_putc(inChar2);
	}

}
void comm_SPI1_polling()
{
	unsigned char rxc, txc = 0x57;
	if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) == SET)
	{
		rxc = SPI1 -> DR;
		USART1_putc(rxc);
	}
	if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_TXE)  == SET)
	{
		SPI1 -> DR = txc;
	}
}

void refresh_variables()
{
//	nRF24_int();

	if(SPI1_flag_err)
	{
		SPI1_flag_err = 0;
		USART1_println("Err");
		while(1);
	}
}
//int main(int argc, char* argv[])

int main(void)
{
	//	SystemCoreClockUpdate();
	SystemInit(); 		// Setup STM32 system (clock, PLL and Flash configuration)

	ADC1_Init();
	IO_Init();
	SysTick_Init();
	USART1_Init(38400); // initialize USART1 @ 38400 baud

#ifdef master
	nRF24_Init();
	USART1_println("Hi, Master!");
#else
	nRF24_Init();
	USART1_println("Hi, Slave!");
#endif

	while (1)
	{
#ifdef master
		comm_Bluetooth();
		handleMessage();
		refresh_variables();

		if(stateMode == 2)
		{
//				sprintf(buffer,"%2d", nRF24_get_RPD());
//				USART1_println(buffer);
//
//				sprintf(buffer,"%2d", nRF24_read_register(NRF_STATUS));
//				USART1_println(buffer);
//
//				sprintf(buffer,"%2d", nRF24_read_register(FIFO_STATUS));
//				USART1_println(buffer);

			if(nRF24_IRQ())
			{
				USART1_println("IRQ Pin");

				int pipeRX = nRF24_poll_RX();
				if(pipeRX)
				{
		//			uint8_t payload_length = nRF24_get_pipe_payload_len(0);
					uint8_t data[2];
					nRF24_read_payload(data, 2);
//					nRF24_flush_RX();								// flush rx;
					sprintf(buffer,"rx: %c, %c", data[0], data[1]);
					USART1_println(buffer);
				}
			}
		}

		if(flag_1s)
		{
			flag_1s ^= flag_1s;

			if(nRF24_flag_send_cont)
			{
				nRF24_send_2bytes();
			}
		}

#else
//		nRF24_set_RX_mode(ENABLE);


		// Transmitter;
//		nRF24_set_TX_mode(ENABLE);



//		SPI1_slave_demo01();
#endif
	}

	return 0;
}














//
////	__DATE__
//
//	FLASH_Unlock();		// Unlock the Flash Program Erase controller
//	EE_Init();			// EEPROM Init
//
//
//
////	time_t     utcsec;
//	long utcsec;
////	struct tm  ts;
//	char       buf[80];
////	Get current time
////	time(&now);
//	utcsec = 1467296589;
////	Format time, "ddd yyyy-mm-dd hh:mm:ss zzz"
////	ts = *localtime(&now);
//	char *c_time_string;
//    c_time_string = ctime(&utcsec);
//    sprintf(buf, "Current time is %s", c_time_string);
////	sprintf(buf, "utcsec: %d", utcsec);
////    strcpy(buf, "Current time is ");
//	USART1_println(buf);


//	int i;
//	for(i=0;i<20;i++)
//	{
//		EE_WriteVariable(addr, name[i]);
//	}

//	/* --- Store successively many values of the three variables in the EEPROM ---*/
//	/* Store 1000 values of Variable1 in EEPROM */
//	for (VarValue = 0; VarValue < 1000; VarValue++)
//	{
//		EE_WriteVariable(VirtAddVarTab[0], VarValue);
//	}
//
//	/* Store 500 values of Variable2 in EEPROM */
//	for (VarValue = 0; VarValue < 500; VarValue++)
//	{
//		EE_WriteVariable(VirtAddVarTab[1], VarValue);
//	}
//
//	/* Store 800 values of Variable3 in EEPROM */
//	for (VarValue = 0; VarValue < 800; VarValue++)
//	{
//		EE_WriteVariable(VirtAddVarTab[2], VarValue);
//	}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
