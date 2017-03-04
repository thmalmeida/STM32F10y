///*
// * RTCi.h
// *
// *  Created on: 24 de fev de 2017
// *      Author: titi
// */
//
//#ifndef HARDWARE_RTCI_H_
//#define HARDWARE_RTCI_H_
//
//#include <stm32f10x.h>
//#include <stm32f10x_rtc.h>
//#include <time.h>
//
//class RTCi {
// public:
// 	RTCi();
////    RTCi(rtc_clk_src src );
////	RTCi(rtc_clk_src src, uint16 prescaler );
//	//~RTCi(); //to implement
//
//
//	void setTime (time_t time_stamp);
//	void setTime (struct tm * tm_ptr);
//
//	struct tm* getTime(struct tm* tm_ptr);
//	time_t getTime();
//
////	void createAlarm(voidFuncPtr function, time_t alarm_time_t);
////	void createAlarm(voidFuncPtr function, struct tm* alarm_tm);
////
////	void attachSecondsInterrupt(voidFuncPtr function);
//	void detachSecondsInterrupt();
//
//	void setAlarmTime (tm * tm_ptr);
//	void setAlarmTime (time_t alarm_time);
// //private:
//
//} ;
//
//
//RTCi::RTCi()
//{
//	RTCi(RTCSEL_HSE, 0xf423);
//}
//RTCi::RTCi(rtc_clk_src src)
//{
//	RTCi(src, 0);
//}//end RTC
//RTCi::RTCi(rtc_clk_src src, uint16 prescaler )
//{
//	switch (src)
//	{
//		case RTCSEL_LSE : {
//		rtc_init(RTCSEL_LSE);//LSE should be 32768 Hz.
//		if (prescaler != 0) rtc_set_prescaler_load(prescaler); //according to sheet clock/(prescaler + 1) = Hz
//		else rtc_set_prescaler_load(0x7fff);
//		break;
//		}
//		case RTCSEL_LSI : {
//		rtc_init(RTCSEL_LSI);//LSI is around 40000 Hz (between 30000 and 60000).
//		if (prescaler != 0) rtc_set_prescaler_load(prescaler); //according to sheet clock/(prescaler + 1) = Hz 39999Hz = 0x9C3F
//		else rtc_set_prescaler_load(0x9C3F);
//		break;
//		}
//		case RTCSEL_HSE : {
//		rtc_init(RTCSEL_HSE);//HSE = 8/128MHz = 62500 Hz
//		if (prescaler != 0) rtc_set_prescaler_load(prescaler); //according to sheet clock/(prescaler + 1) = Hz 0xF423 = 62499
//		else rtc_set_prescaler_load(0xF423);
//		break;
//		}
//		case RTCSEL_DEFAULT: {
//		//do nothing. Have a look at the clocks to see the diff between NONE and DEFAULT
//		break;
//		}
//		case RTCSEL_NONE: {
//		//do nothing. Have a look at the clocks to see the diff between NONE and DEFAULT
//		break;
//		}
//	}//end switch
//}
///*
//RTCi::~RTCi() {
////to implement
//}
//*/
//void RTCi::setTime (time_t time_stamp)
//{
//
////	rtc_set_count(time_stamp);
//}
//void RTCi::setTime (struct tm* tm_ptr)
//{
//	rtc_set_count(mktime (tm_ptr));
//}
//time_t RTCi::getTime()
//{
//	return rtc_get_count();
//}
//struct tm*  RTCi::getTime(struct tm* tm_ptr)
//{
//	time_t res = rtc_get_count();
//	tm_ptr = gmtime(&res); //why not gmtime?
//	return tm_ptr;
//}
//void RTCi::createAlarm(voidFuncPtr function, time_t alarm_time_t)
//{
//	rtc_set_alarm(alarm_time_t); //must be int... for standardization sake.
//	rtc_attach_interrupt(RTC_ALARM_SPECIFIC_INTERRUPT, function);
//}
//void RTCi::attachSecondsInterrupt(voidFuncPtr function)
//{
//	rtc_attach_interrupt(RTC_SECONDS_INTERRUPT, function);
//}
//void RTCi::detachSecondsInterrupt()
//{
//	rtc_detach_interrupt(RTC_SECONDS_INTERRUPT);
//}
//void RTCi::createAlarm(voidFuncPtr function, tm* alarm_tm)
//{
//	time_t alarm = mktime(alarm_tm);//convert to time_t
//	createAlarm(function, alarm);
//}
////change alarm time
//void RTCi::setAlarmTime (tm * tm_ptr)
//{
//	time_t time = mktime(tm_ptr);//convert to time_t
//	rtc_set_alarm(time); //must be int... for standardization sake.
//}
//
//void RTCi::setAlarmTime (time_t alarm_time)
//{
//	rtc_set_alarm(alarm_time);
//}
//
//
//
//#endif /* HARDWARE_RTCI_H_ */
