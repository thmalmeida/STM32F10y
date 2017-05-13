/*
 * rtc.h
 *
 *  Created on: 25 de fev de 2017
 *      Author: titi
 */

#include <stm32f10x.h>
#include <stm32f10x_rtc.h>
#include <stdio.h>

typedef struct  {
	uint8_t Second;
	uint8_t Minute;
	uint8_t Hour;
	uint8_t Wday;   // day of week, sunday is day 1
	uint8_t Day;
	uint8_t Month;
	uint8_t Year;   // offset from 1970;
} tmElements_t, TimeElements, *tmElementsPtr_t;

class RTCi {
public:

	struct tm *timeinfo;//, rtc0;
	time_t rawtime;
	tm *rtc0 = new tm;
	time_t uptime32 = 0;

	uint32_t rtc_PRL;
	uint8_t  rtc_clkSource;

	char _buffer[50];

	void begin_rtc(uint8_t clockSource, uint32_t rtc_PRL);
	void setTime(int year, int mon, int mday, int hour, int min, int sec);
	void write(tmElements_t &tm);
	void getTime();
	void getUptime();
	int year();
	int month();
	int day();
	int hour();
	int minute();
	int second();
	void bkpDomainReset(void);
	void calibrateLSI(void);

//	uint32_t GetLSIFrequency(uint16_t arr, uint16_t psc);

	uint32_t getRTC_DIV();
	void setRTC_DIV(uint32_t divValue);

protected:
};


void RTCi::begin_rtc(uint8_t clockSource, uint32_t rtc_PRL)
{
	RCC_APB1PeriphClockCmd(RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN, ENABLE);	// power and backup clocks
	PWR->CR |= PWR_CR_DBP;								// it enable access to the Backup registers and RTC.

	switch(clockSource)
	{
		case 0:
			RCC->BDCR |= RCC_BDCR_RTCSEL_LSI;			// Select internal low speed oscillator clock source;
			RCC->CSR |= RCC_CSR_LSION;					// Internal low speed oscillator enable;
			while((RCC->CSR & RCC_CSR_LSIRDY) == 0);	// Wait until external oscillator is established
//			while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);
		break;

		case 1:
			RCC->BDCR |= RCC_BDCR_RTCSEL_0;				// Select high speed clock source;
			RCC->BDCR |= RCC_BDCR_RTCSEL_1;
			RCC->BDCR |= RCC_BDCR_RTCSEL_HSE;
			RCC->CR |= RCC_CR_HSEON;
			while((RCC->CR & RCC_CR_HSERDY) == 0);
		break;

		case 2:
			RCC->BDCR |= RCC_BDCR_RTCSEL_0;
			RCC->BDCR |= RCC_BDCR_RTCSEL_LSE;			// select external low speed oscillator clock source;
			RCC->BDCR |= RCC_BDCR_LSEON;				// External Low Speed oscillator enable;
			while((RCC->BDCR & RCC_BDCR_LSERDY) == 0);	// Wait until external oscillator is established
		break;

	}

	RCC->BDCR |= RCC_BDCR_RTCEN;						// Enable RTC clock
	RTC_WaitForSynchro();								// Clear registers and wait for sync registers;
	RTC->CRH |= RTC_CRH_SECIE;							//Second Interrupt Enable

	//    NVIC->ISER[0] |= (1 << (RTC_IRQChannel & 0x1F));            // enable interrupt
    NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;			// we want to configure the RTC 1 sec interrupt
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;	// this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			// the RTC interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							// the properties are passed to the NVIC_Init function which takes care of the low level stuff

	if(!clockSource && rtc_PRL)								// set RTC_PRL is LSI is selected
	{
		setRTC_DIV(rtc_PRL);
	}




//	RCC_APB1PeriphClockCmd(RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN, ENABLE);	// power and backup clocks
//	PWR->CR |= PWR_CR_DBP;								// it enable access to the Backup registers and RTC.
//
////	RCC->BDCR |= RCC_BDCR_RTCSEL_0;						// Select high speed clock source;
////	RCC->BDCR |= RCC_BDCR_RTCSEL_1;
//	RCC->BDCR |= RCC_BDCR_RTCSEL_LSE;					// select external low speed oscillator clock source;
//	RCC->BDCR |= RCC_BDCR_LSEON;						// External Low Speed oscillator enable;
//	while((RCC->BDCR & RCC_BDCR_LSERDY) == 0);			// Wait until external oscillator is stabilished
//
//	RCC->BDCR |= RCC_BDCR_RTCEN;						// Enable RTC clock
//
//	RTC->CRL &= ~RTC_CRL_RSF;							// Clear registers Synchronized Flag
//	while(!(RTC->CRL & RTC_CRL_RSF));					// Wait for the RSF bit in RTC_CRL to be set by hardware
//
//	RTC->CRH |= RTC_CRH_SECIE;							//Second Interrupt Enable
//
//	//    NVIC->ISER[0] |= (1 << (RTC_IRQChannel & 0x1F));            // enable interrupt
//    NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)
//
//	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;			// we want to configure the RTC 1 sec interrupt
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;	// this sets the subpriority inside the group
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			// the RTC interrupts are globally enabled
//	NVIC_Init(&NVIC_InitStructure);							// the properties are passed to the NVIC_Init function which takes care of the low level stuff

//    while((RTC->CRL & RTC_CRL_RTOFF) == 0);    //Wait for RTOFF It is not possible to write to the RTC_CR register while the peripheral is completing a previous write operation
//    RTC->CRL |= RTC_CRL_CNF;                   //Set the CNF bit to enter configuration mode
    /* Set RTC COUNTER MSB word */
//    RTC->CNTH = (12*3600 + 40*60 + 00) >> 16;  //Random time
    /* Set RTC COUNTER LSB word */
//    RTC->CNTL = ((12*3600 + 40*60 + 00)& 0x0000FFFF);
//    RTC->CRH |= RTC_CRH_SECIE;                 //Second Interrupt Enable
//    RTC->CRL &= ~RTC_CRL_CNF;                  //Exit configuration mode
//    while((RTC->CRL & RTC_CRL_RTOFF) == 0);    //Wait for RTOFF
}
void RTCi::setTime(int year, int mon, int mday, int hour, int min, int sec)
{
	timeinfo = localtime(&rawtime);
	timeinfo->tm_year = year - 1900;
	timeinfo->tm_mon = mon - 1;
	timeinfo->tm_mday = mday;
	timeinfo->tm_hour = hour;
	timeinfo->tm_min = min;
	timeinfo->tm_sec = sec;
	timeinfo->tm_isdst = 1;
//	timeinfo->tm_yday = 55;
//	timeinfo->tm_wday = 5;

	rawtime = mktime(timeinfo);

	RTC_WaitForLastTask();
	RTC_SetCounter(rawtime);
	RTC_WaitForLastTask();

//	sprintf(_buffer, "%s", ctime(&rawtime));
//	Serial.println(_buffer);
}
void RTCi::write(tmElements_t &tm)
{
	timeinfo = localtime(&rawtime);
	timeinfo->tm_year = tm.Year;
	timeinfo->tm_mon = tm.Month - 1;
	timeinfo->tm_mday = tm.Day;
	timeinfo->tm_hour = tm.Hour;
	timeinfo->tm_min = tm.Minute;
	timeinfo->tm_sec = tm.Second;
	timeinfo->tm_isdst = 1;
//	timeinfo->tm_yday = 55;
//	timeinfo->tm_wday = 5;

	rawtime = mktime(timeinfo);

	RTC_WaitForLastTask();
	RTC_SetCounter(rawtime);
	RTC_WaitForLastTask();
}
void RTCi::getTime()
{
	rawtime = RTC_GetCounter();
	timeinfo = localtime(&rawtime);
}
void RTCi::getUptime()
{
	localtime_r(&uptime32, rtc0);
}
int RTCi::year()
{
	getTime();
	return timeinfo->tm_year+1970;
}
int RTCi::month()
{
	getTime();
	return timeinfo->tm_mon+1;
}
int RTCi::day()
{
	getTime();
	return timeinfo->tm_mday;
}
int RTCi::hour()
{
	getTime();
	return timeinfo->tm_hour;
}
int RTCi::minute()
{
	getTime();
	return timeinfo->tm_min;
}
int RTCi::second()
{
	getTime();
	return timeinfo->tm_sec;
}
void RTCi::bkpDomainReset(void)
{
	RCC->BDCR |= RCC_BDCR_BDRST;						// Backup domain reset by software;
}
void RTCi::calibrateLSI(void)
{
//	/* RTC Configuration -------------------------------------------------------*/
//	NVIC_InitTypeDef NVIC_InitStructure;
//	EXTI_InitTypeDef EXTI_InitStructure;
//
//	RTC_TypeDef RTC_InitStructure;
////	RTC_InitTypeDef   RTC_InitStructure;
//	__IO uint32_t LsiFreq = 0;
//	__IO uint32_t CaptureNumber = 0, PeriodValue = 0;
//
//	/* Enable the PWR clock */
//	//	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
//
//	/* Allow access to RTC */
//	//	  PWR_RTCAccessCmd(ENABLE);
//
//	/* LSI used as RTC source clock */
//	/* The RTC Clock may varies due to LSI frequency dispersion. */
//	/* Enable the LSI OSC */
////	RCC_LSICmd(ENABLE);
//
//	/* Wait till LSI is ready */
////	while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
////	{
////	}
//
//	/* Select the RTC Clock Source */
////	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
//
//	/* Enable the RTC Clock */
////	RCC_RTCCLKCmd(ENABLE);
//
//	/* Wait for RTC APB registers synchronisation */
////	RTC_WaitForSynchro();
//
//	/* Calendar Configuration */
//	RTC_InitStructure.
//	RTC_InitStructure.RTC_AsynchPrediv = 0x7F;
//	RTC_InitStructure.RTC_SynchPrediv	=  0x120; /* (37KHz / 128) - 1 = 0x120*/
//	RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
//	RTC_Init(&RTC_InitStructure);
//
//	/* EXTI configuration *******************************************************/
//	EXTI_ClearITPendingBit(EXTI_Line20);
//	EXTI_InitStructure.EXTI_Line = EXTI_Line20;
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//	EXTI_Init(&EXTI_InitStructure);
//
//	/* Enable the RTC Wakeup Interrupt */
//	NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//
//	/* Configure the RTC WakeUp Clock source: CK_SPRE (1Hz) */
//	RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);
//	RTC_SetWakeUpCounter(0x0);
//
//	/* Enable the RTC Wakeup Interrupt */
//	RTC_ITConfig(RTC_IT_WUT, ENABLE);
//
//	/* Enable Wakeup Counter */
//	RTC_WakeUpCmd(ENABLE);
//
//	// -------------------
//
//	/* Wait Until KEY BUTTON is pressed */
//	while(STM_EVAL_PBGetState(BUTTON_KEY) == RESET)
//	{
//	}
//
//	/* Get the LSI frequency:  TIM10 is used to measure the LSI frequency */
//	LsiFreq = GetLSIFrequency();
//
//	/* Turn on LED2 */
//	STM_EVAL_LEDOn(LED2);
//
//	/* Calendar Configuration */
//	RTC_InitStructure.RTC_AsynchPrediv = 0x7F;
//	RTC_InitStructure.RTC_SynchPrediv	=  (LsiFreq/128) - 1;
//	RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
//	RTC_Init(&RTC_InitStructure);
}
void RTCi::setRTC_DIV(uint32_t divValue)
{
	RTC_WaitForLastTask();
	RTC_SetPrescaler(divValue);
	RTC_WaitForLastTask();
}
uint32_t RTCi::getRTC_DIV()
{
	return RTC_GetDivider();
}
void NVIC_Configuration(void)
{
//	NVIC_InitTypeDef NVIC_InitStructure;
//
//	/* Configure one bit for preemption priority */
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
//
//	/* Enable the RTC Interrupt */
//	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//
//	/* Enable the TIM5 Interrupt */
//	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
}

//uint32_t RTCi::GetLSIFrequency(uint16_t arr, uint16_t psc)
//{
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	TIM_ICInitTypeDef  TIM_ICInitStructure;
//	RCC_ClocksTypeDef RCC_Clocks;
//	__IO uint32_t PeriodValue = 0,  LsiFreq = 0;
//	__IO uint32_t OperationComplete = 0;
//
//	RCC_GetClocksFreq(&RCC_Clocks);
//
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
//	GPIO_PinRemapConfig(GPIO_Remap_TIM5CH4_LSI, ENABLE);
//
//	/* TIM5 Time base configuration */
//	TIM_TimeBaseStructure.TIM_Prescaler = 0;
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
//	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
//
//	  /* TIM5 Channel4 Input capture Mode configuration */
//	  TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
//	  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
//	  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
//	  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
//	  TIM_ICInitStructure.TIM_ICFilter = 0;
//	  TIM_ICInit(TIM5, &TIM_ICInitStructure);
//
//	  /* Reinitialize the index for the interrupt */
//	  OperationComplete = 0;
//
//	  /* Enable the TIM5 Input Capture counter */
//	  TIM_Cmd(TIM5, ENABLE);
//	  /* Reset all TIM5 flags */
//	  TIM5->SR = 0;
//	  /* Enable the TIM5 channel 4 */
//	  TIM_ITConfig(TIM5, TIM_IT_CC4, ENABLE);
//
//	  /* NVIC configuration */
//	  NVIC_Configuration();
//	// configure channel 4 in input capture mode;
////	TIM5->CCR1 |= TIM1_
//
//
//	AFIO->MAPR |= AFIO_MAPR_TIM5CH4_IREMAP;		// LSI internal clock is connected to TIM5_CH4 input for calibration;
//
//	NVIC_InitTypeDef   NVIC_InitStructure;
//	TIM_ICInitTypeDef  TIM_ICInitStructure;
//	RCC_ClocksTypeDef  RCC_ClockFreq;
//
//	/* Enable the LSI oscillator ************************************************/
//	RCC_LSICmd(ENABLE);
//
//
//	/* Wait till LSI is ready */
//	while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
//	{}
//
//
//	/* TIM10 configuration *******************************************************/
//	/* Enable TIM10 clock */
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
//
//	/* Reset TIM10 registers */
//	TIM_DeInit(TIM10);
//
//	/* Configure TIM10 prescaler */
//	TIM_PrescalerConfig(TIM10, 0, TIM_PSCReloadMode_Immediate);
//
//	/* Connect LSI clock to TIM10 Input Capture 1 */
//	TIM_RemapConfig(TIM10, TIM10_LSI);
//
//	/* TIM10 configuration: Input Capture mode ---------------------
//	The reference clock(LSE or external) is connected to TIM10 CH1
//	The Rising edge is used as active edge,
//	The TIM10 CCR1 is used to compute the frequency value
//	------------------------------------------------------------ */
//	TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;
//	TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
//	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
//	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;
//	TIM_ICInitStructure.TIM_ICFilter = 0x0;
//	TIM_ICInit(TIM10, &TIM_ICInitStructure);
//
//	/* Enable the TIM10 global Interrupt */
//	NVIC_InitStructure.NVIC_IRQChannel = TIM10_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//
//	/* Enable TIM10 counter */
//	TIM_Cmd(TIM10, ENABLE);
//
//	/* Reset the flags */
//	TIM10->SR = 0;
//
//	/* Enable the CC4 Interrupt Request */
//	TIM_ITConfig(TIM10, TIM_IT_CC1, ENABLE);
//
//
//	/* Wait until the TIM10 get 2 LSI edges (refer to TIM10_IRQHandler() in
//	stm32l1xx_it.c file) ******************************************************/
//	while(CaptureNumber != 2)
//	{
//	}
//	/* Deinitialize the TIM10 peripheral registers to their default reset values */
//	TIM_DeInit(TIM10);
//
//
//	/* Compute the LSI frequency, depending on TIM10 input clock frequency (PCLK1)*/
//	/* Get SYSCLK, HCLK and PCLKx frequency */
//	RCC_GetClocksFreq(&RCC_ClockFreq);
//
//	/* Get PCLK1 prescaler */
//	if ((RCC->CFGR & RCC_CFGR_PPRE1) == 0)
//	{
//	/* PCLK1 prescaler equal to 1 => TIMCLK = PCLK1 */
//	return ((RCC_ClockFreq.PCLK1_Frequency / PeriodValue) * 8);
//	}
//	else
//	{ /* PCLK1 prescaler different from 1 => TIMCLK = 2 * PCLK1 */
//	return (((2 * RCC_ClockFreq.PCLK1_Frequency) / PeriodValue) * 8) ;
//	}
//}





//void RTCi::putTime()
//{
	//	sprintf(buffer, "rawtime: %ld", rawtime);
	//	Serial.println(buffer);
//	sprintf(_buffer, "%s", ctime(&rawtime));
//	Serial.println(_buffer);
//}
