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

extern __IO uint32_t time_us;
extern __IO uint32_t time_ms;
extern __IO int USART1_cnt;
extern __IO int flag_1s;
extern __IO char received_string[MAX_STRLEN+1]; // this will hold the recieved string
extern __IO unsigned char SPI_received_string[MAX_STRLEN+1];
extern __IO int SPI1_cnt;
extern __IO unsigned int SPI1_flag_rx;
extern __IO unsigned int SPI1_flag_err;
extern __IO unsigned char SPI1_rxd;
extern __IO unsigned char SPI1_txd;

#define master

//volatile uint8_t cnt = 0; // this counter is used to determine the string length
// this is the interrupt request handler (IRQ) for ALL USART1 interrupts

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
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

void USART1_IRQHandler(void)
{
	// check if the USART1 receive interrupt flag was set
	while(USART_GetITStatus(USART1, USART_IT_RXNE))
	{
//		LED_green_toogle();
		char t = USART1->DR; // the character from the USART1 data register is saved in t
		if(USART1_cnt < MAX_STRLEN)
		{
			received_string[USART1_cnt] = t;
			USART1_cnt++;
		}
		else
		{
			memset(received_string,0,sizeof(received_string));
			USART1_cnt = 0;
		}

//		USART1_putc(t);

		// check if the received character is not the LF character (used to determine end of string)
		// or the if the maximum string length has been been reached
//		if( (t != '\n') && (cnt < MAX_STRLEN) )
//		{
//			received_string[cnt] = t;
//			cnt++;
//		}
//		else{ // otherwise reset the character counter and print the received string
//			cnt = 0;
//			USART1_puts(received_string);
//			memset(received_string,0,sizeof(received_string));
//		}
	}
}

void SPI1_IRQHandler(void)
{
#ifdef master
	//	static unsigned short int count = 0, i = 0 ;
	//	check if the SPI1 receive interrupt flag was set
	//	Master interrupt
	//	TX: 1- empty
	//	if(SPI1 -> SR & 0x0002)
		if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_TXE) == SET)
		{
			SPI1_flag_rx = 1;
			SPI1_rxd++;
			SPI1_txd = SPI1_rxd;
			SPI1 -> DR = SPI1_txd;
	//		Wait until the data has been transmitted.
	//		while (!(SPI1->SR & SPI_I2S_FLAG_TXE));
	//		USART1_print("Transmitting: ");
	//		USART1_putc(0x40);
		}
//	//	if(SPI1 -> SR & SPI_I2S_FLAG_RXNE)
//		//	RX: 1- not empty
//	//	if(SPI1 -> SR & 0x0001)
		if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) == SET)
		{
			SPI1_flag_rx = 1;
			SPI1_rxd = SPI1 -> DR;
			if(SPI1_cnt < MAX_STRLEN)
			{
				SPI_received_string[SPI1_cnt] = SPI1_rxd;
				SPI1_cnt++;
			}
			else
			{
				memset(SPI_received_string, 0, sizeof(SPI_received_string));
				SPI1_cnt = 0;
			}
		}

	//	if(SPI1 -> SR & 0x0100)
		if(SPI_I2S_GetITStatus(SPI1, SPI_IT_CRCERR))
		{
			SPI1_flag_err = 1;
		}
#else
//	static unsigned short int count = 0, i = 0 ;
//	check if the SPI1 receive interrupt flag was set
//	Slave interrupt
//	TX: 1- empty
//	if(SPI1 -> SR & 0x0002)
	if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_TXE) == SET)
	{
		SPI1_rxd++;
		SPI1_txd = SPI1_rxd;
		SPI1 -> DR = SPI1_txd;
//		Wait until the data has been transmitted.
//		while (!(SPI1->SR & SPI_I2S_FLAG_TXE));
//		USART1_print("Transmitting: ");
//		USART1_putc(0x40);
	}
//	if(SPI1 -> SR & SPI_I2S_FLAG_RXNE)
	//	RX: 1- not empty
//	if(SPI1 -> SR & 0x0001)
	if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) == SET)
	{
		SPI1_flag_rx = 1;
		SPI1_rxd = SPI1 -> DR;
		if(SPI1_cnt < MAX_STRLEN)
		{
			SPI_received_string[SPI1_cnt] = SPI1_rxd;
			SPI1_cnt++;
		}
		else
		{
			memset(SPI_received_string, 0, sizeof(SPI_received_string));
			SPI1_cnt = 0;
		}
	}

//	if(SPI1 -> SR & 0x0100)
	if(SPI_I2S_GetITStatus(SPI1, SPI_IT_CRCERR))
	{
		SPI1_flag_err = 1;
	}
#endif
}
/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
