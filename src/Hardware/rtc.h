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
	time_t uptime32;

	char _buffer[50];

	void begin_rtc(void);
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
};

void RTCi::begin_rtc()
{
	RCC_APB1PeriphClockCmd(RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN, ENABLE);	// power and backup clocks
	PWR->CR |= PWR_CR_DBP;								// it enable access to the Backup registers and RTC.

//	RCC->BDCR |= RCC_BDCR_RTCSEL_0;						// Select high speed clock source;
//	RCC->BDCR |= RCC_BDCR_RTCSEL_1;
	RCC->BDCR |= RCC_BDCR_RTCSEL_LSE;					// select external low speed oscillator clock source;
	RCC->BDCR |= RCC_BDCR_LSEON;						// External Low Speed oscillator enable;
	while((RCC->BDCR & RCC_BDCR_LSERDY) == 0);			// Wait until external oscillator is stabilished

	RCC->BDCR |= RCC_BDCR_RTCEN;						// Enable RTC clock

	RTC->CRL &= ~RTC_CRL_RSF;							// Clear registers Synchronized Flag
	while(!(RTC->CRL & RTC_CRL_RSF));					// Wait for the RSF bit in RTC_CRL to be set by hardware

	RTC->CRH |= RTC_CRH_SECIE;							//Second Interrupt Enable

	//    NVIC->ISER[0] |= (1 << (RTC_IRQChannel & 0x1F));            // enable interrupt
    NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;			// we want to configure the RTC 1 sec interrupt
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;	// this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			// the RTC interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							// the properties are passed to the NVIC_Init function which takes care of the low level stuff

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

RTCi rtc;






//void RTCi::putTime()
//{
	//	sprintf(buffer, "rawtime: %ld", rawtime);
	//	Serial.println(buffer);
//	sprintf(_buffer, "%s", ctime(&rawtime));
//	Serial.println(_buffer);
//}
