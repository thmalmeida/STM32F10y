/*
 * acionna.h
 *
 *  Created on: 14 de fev de 2017
 *      Author: titi
 */

#ifndef ACIONNA_H_
#define ACIONNA_H_

#define stm32f1

#ifdef atmega

#define k1_on()				PORTD |=  (1<<2);
#define k1_off()			PORTD &= ~(1<<2);
#define k2_on()				PORTD |=  (1<<3);
#define k2_off()			PORTD &= ~(1<<3);
#define k3_on()				PORTD |=  (1<<4);
#define k3_off()			PORTD &= ~(1<<4);
#define drive_led_on()		PORTB |=  (1<<5);
#define drive_led_off()		PORTB &= ~(1<<5);

#define DefOut_k1()			DDRD |=  (1<<2);
#define DefOut_k2()			DDRD |=  (1<<3);
#define DefOut_k3()			DDRD |=  (1<<4);
#define DefOut_led()		DDRB |=  (1<<5);

#define DefIn__Rth()		DDRD &= ~(1<<6);
#define DefIn__read_k1()	DDRD &= ~(1<<5);
#define DefIn__read_k3()	DDRD &= ~(1<<7);


//#define readPin_Rth			(~PIND & 0b10000000)
#define readPin_k1_PIN		bit_is_set(PIND, 2)

#define readPin_k1			bit_is_clear(PIND, 5)
#define readPin_Rth			bit_is_clear(PIND, 6)
#define readPin_k3			bit_is_clear(PIND, 7)

#else

#include <stm32f10x.h>

#include "Hardware/adc.h"
#include "Hardware/usart.h"
#include <ctime>

class GPIO {
public:

	void gateConfig(uint8_t pin, uint8_t dir);
	void gateSet(uint8_t pin, uint8_t status);
	void gateToggle(uint8_t pin);
	bool gateRead(uint8_t pin);
};

void GPIO::gateConfig(uint8_t pin, uint8_t dir)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	switch (pin)
	{
		case 1:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;

			if(dir)
			{
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
				GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			}
			else
			{
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
			}
			GPIO_Init(GPIOC, &GPIO_InitStructure);
			break;

		case 2:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;

			if(dir)
			{
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
				GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			}
			else
			{
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
			}
			GPIO_Init(GPIOC, &GPIO_InitStructure);
			break;

		case 3:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;

			if(dir)
			{
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
				GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			}
			else
			{
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
			}
			GPIO_Init(GPIOC, &GPIO_InitStructure);
			break;
	}
}
void GPIO::gateSet(uint8_t pin, uint8_t status)
{
	switch (pin)
	{
		case 1:
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
			break;

		case 2:
			if(status)
			{
				GPIOC -> BSRR = (1<<(14+16)); // 16 bit shift
		//		GPIO_SetBits(GPIOC, 14);
			}
			else
			{
				GPIOC -> BSRR = (1<<14);
		//		GPIO_ResetBits(GPIOC, 13);
			}
			break;

		case 3:
			if(status)
			{
				GPIOC -> BSRR = (1<<(15+16)); // 16 bit shift
		//		GPIO_SetBits(GPIOC, 15);
			}
			else
			{
				GPIOC -> BSRR = (1<<15);
		//		GPIO_ResetBits(GPIOC, 15);
			}
			break;
	}
}
void GPIO::gateToggle(uint8_t pin)
{
	switch (pin)
	{
		case 1:
			GPIOC -> ODR ^= (1<<13);
			break;

		case 2:
			GPIOC -> ODR ^= (1<<14);
			break;

		case 3:
			GPIOC -> ODR ^= (1<<15);
			break;
	}

}
bool GPIO::gateRead(uint8_t pin)
{
	uint8_t status = 0;

	switch (pin)
	{
		case 1:
			status = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13);
			break;

		case 2:
			status = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14);
			break;

		case 3:
			status = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15);
			break;

		case 4:
			status = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
			break;
	}

	if(status)
	{
		return true;
	}
	else
		return false;
}

#define pin_out_led 1
#define pin_out_k1	2
#define pin_out_k2	3
#define pin_out_k3	4
#define pin_in_Rth	5
#define pin_in_k1	6
#define pin_in_k3	7

#define startTypeK			1		// Partida direta: monofásico
//#define startTypeK			2		// Partida direta: trifásico
//#define startTypeK			3		// Partida estrela/triangulo

enum states01 {
	redTime,
	greenTime
};
enum states01 periodo = redTime;

typedef struct  {
	uint8_t Second;
	uint8_t Minute;
	uint8_t Hour;
	uint8_t Wday;   // day of week, sunday is day 1
	uint8_t Day;
	uint8_t Month;
	uint8_t Year;   // offset from 1970;
} tmElements_t, TimeElements, *tmElementsPtr_t;

tmElements_t tm;

USART Serial;

class ACIONNA : GPIO , ADC, USART {
public:

	uint8_t enableDecode = 0, opcode;
	static const uint8_t sInstr_SIZE = 17;
	uint8_t k, rLength, j;
	uint8_t j2 = 0;
	uint8_t flag_frameStartBT = 0;
	uint8_t enableTranslate_Bluetooth = 0;
	char aux[3], aux2[5], buffer[60], inChar, sInstr[sInstr_SIZE];
	char sInstrBluetooth[30];

	uint16_t motorTimerE = 0;
	uint8_t PRessureRef = 0;
	uint8_t PRessureRef_Valve = 0;
	uint8_t PRessureMax_Sensor = 100; // units in psi
	uint8_t PRessurePer = 85;

	uint8_t flag_AwaysON = 0;
	uint8_t flag_timeMatch = 0;
	uint8_t flag_PressureDown = 1;
	uint8_t flag_01 = 0;
	uint8_t flag_02 = 0;
	uint8_t flag_03 = 0;

	uint8_t flag_Th = 0;

	//uint16_t motor_timerON = 0;
	//uint16_t motor_timerOFF = 0;

	uint8_t enableSend = 0;
	uint8_t enableTranslate = 0;
	uint8_t flagSync = 0;
	uint8_t countSync = 0;
	uint8_t flag_debug = 0;

	uint8_t motorStatus = 0;
	uint8_t flag_waitPowerOn = 1;	// Minutes before start motor after power line ocasionally down
	uint8_t waitPowerOn_min_standBy=0;
	uint8_t waitPowerOn_min = 0;
	uint8_t waitPowerOn_sec = 0;
	//uint8_t powerOff_min = 0;
	//uint8_t powerOff_sec = 0;
	// Motor timers in milliseconds
	uint8_t motorTimerStart1 = 35;
	uint16_t motorTimerStart2 = 200;

	uint8_t HourOn  = 21;
	uint8_t MinOn   = 30;
	uint8_t HourOff = 6;
	uint8_t MinOff  = 0;

	uint8_t nTM;
	uint8_t HourOnTM[9];
	uint8_t MinOnTM[9];

	uint16_t levelRef_10bit = 0;
	uint8_t stateMode = 0;


	// Logs
	static const int nLog = 7;
	uint8_t reasonV[nLog];
	uint8_t hourLog_ON[nLog], minuteLog_ON[nLog];
	uint8_t hourLog_OFF[nLog], minuteLog_OFF[nLog];
	uint8_t dayLog_ON[nLog], monthLog_ON[nLog];
	uint8_t dayLog_OFF[nLog], monthLog_OFF[nLog];

	uint16_t levelSensorLL, levelSensorML, levelSensorHL;
	uint16_t levelSensorLL_d, levelSensorML_d, levelSensorHL_d;

	uint16_t timeOn_min = 0;
	uint8_t  timeOn_sec = 0;
	uint16_t timeOff_min = 0;
	uint8_t  timeOff_sec = 0;

	volatile int flag_1s;

	int PRess;
	int PRessHold;
	int Pd = 0;

	void drive_led_on();
	void drive_led_off();

	void DefOut_k1();
	void DefOut_k2();
	void DefOut_k3();
	void DefOut_led();

	void DefIn__Rth();
	void DefIn__read_k1();
	void DefIn__read_k3();

	bool readPin_k1_PIN();
	bool readPin_k1();
	bool readPin_Rth();
	bool readPin_k3();

	void motor_start();
	void motor_stop(uint8_t reason);

	void driveMotor_ON(uint8_t startType);
	void driveMotor_OFF();

	void begin_acn();

	void get_levelSensors();
	double get_Pressure();
	void check_pressure();
	void check_levelSensors();
	void check_period();
	void check_timeMatch();
	void check_thermalSafe();
	void check_gpio();
	void check_TimerVar();
	void check_pressureDown();
	void process_valveControl();
	void process_waterPumpControl();
	void process_motorPeriodDecision();
	void process_Mode();
	void summary_Print(uint8_t opt);
	void RTC_update();
	void refreshVariables();
	void refreshStoredData();
	void handleMessage();
	void comm_Bluetooth();

private:
	void k1_on();
	void k1_off();
	void k2_on();
	void k2_off();
	void k3_on();
	void k3_off();
};
bool ACIONNA::readPin_Rth()
{
	return gateRead(pin_in_Rth);
}
bool ACIONNA::readPin_k1()
{
	return 	gateRead(pin_in_k1);
}
bool ACIONNA::readPin_k1_PIN()
{
	return gateRead(pin_in_k1);
}
bool ACIONNA::readPin_k3()
{
	return gateRead(pin_in_k3);
}
void ACIONNA::begin_acn()
{
	gateConfig(pin_out_k1, 1);
	gateConfig(pin_out_k2, 1);
	gateConfig(pin_out_k3, 1);

	gateConfig(pin_in_k1, 0);
	gateConfig(pin_in_k3, 0);
	gateConfig(pin_in_Rth, 0);
}
void ACIONNA::drive_led_on()
{
	gateSet(pin_out_led, 1);
}
void ACIONNA::drive_led_off()
{
	gateSet(pin_out_led, 0);
}
void ACIONNA::k1_on()
{
	gateSet(pin_out_k1, 1);
}
void ACIONNA::k1_off()
{
	gateSet(pin_out_k1, 0);
}
void ACIONNA::k2_on()
{
	gateSet(pin_out_k2, 1);
}
void ACIONNA::k2_off()
{
	gateSet(pin_out_k2, 0);
}
void ACIONNA::k3_on()
{

}
void ACIONNA::k3_off()
{

}
void ACIONNA::driveMotor_OFF()
{
	k1_off();
	k2_off();
	k3_off();
}
void ACIONNA::motor_stop(uint8_t reason)
{
	int i;
	for(i=(nLog-1);i>0;i--)
	{
		hourLog_OFF[i] = hourLog_OFF[i-1];
		minuteLog_OFF[i] = minuteLog_OFF[i-1];

		dayLog_OFF[i] = dayLog_OFF[i-1];
		monthLog_OFF[i] = monthLog_OFF[i-1];
	}

	reasonV[1] = reasonV[0];
	reasonV[0] = reason;

	hourLog_OFF[0] = tm.Hour;
	minuteLog_OFF[0] = tm.Minute;

	dayLog_OFF[0] = tm.Day;
	monthLog_OFF[0] = tm.Month;

	timeOff_min = 0;
	timeOff_sec = 0;

	flag_waitPowerOn = 1;
	waitPowerOn_min = waitPowerOn_min_standBy;

	driveMotor_OFF();
	drive_led_off();
	_delay_ms(1);

	motorStatus = readPin_k1_PIN();
}
void ACIONNA::motor_start()
{
	if(!flag_waitPowerOn)
	{
		int i;
		for(i=(nLog-1);i>0;i--)
		{
			hourLog_ON[i] = hourLog_ON[i-1];
			minuteLog_ON[i] = minuteLog_ON[i-1];

			dayLog_ON[i] = dayLog_ON[i-1];
			monthLog_ON[i] = monthLog_ON[i-1];
		}

		hourLog_ON[0] = tm.Hour;
		minuteLog_ON[0] = tm.Minute;

		dayLog_ON[0] = tm.Day;
		monthLog_ON[0] = tm.Month;

		timeOn_min = 0;
		timeOn_sec = 0;

		PRessHold = PRess;

		driveMotor_ON(startTypeK);
		drive_led_on();

		_delay_ms(1);

		motorStatus = readPin_k1_PIN();
	}
}
void ACIONNA::driveMotor_ON(uint8_t startType)
{
	switch (startType)
	{
		case 1:	// Partida direta: monofásico
			k1_on();
			break;

		case 2: // Partida direta: trifásico
			k1_on();
			k2_on();
			break;

		case 3:	// Partida estrela/triangulo
			k1_on();
			k3_on();
			//wdt_reset();
			_delay_ms(((double) 100.0*motorTimerStart1));
			//wdt_reset();

			k3_off();
			uint32_t countK = 0;
			while(readPin_k3())
			{
				countK++;
				if(countK>=120000)
				{
					k1_off();
					k2_off();
					k3_off();
					return;
				}
			}
		//			Serial.print("FuCK: ");
		//			Serial.println(count);
			_delay_ms(motorTimerStart2);
			k2_on();
			break;
	}
}
void ACIONNA::get_levelSensors()
{
	// Select ADC0 - LL sensor
	levelSensorLL_d = adc_readChannel(0);

	if(levelSensorLL_d < levelRef_10bit)
		levelSensorLL = 1;
	else
		levelSensorLL = 0;


	// Select ADC1 - ML sensor
	levelSensorML_d = adc_readChannel(1);

	if(levelSensorML_d < levelRef_10bit)
		levelSensorML = 1;
	else
		levelSensorML = 0;


	// Select ADC2 - HL sensor
	levelSensorHL_d = adc_readChannel(2);

	if(levelSensorHL_d < levelRef_10bit)
		levelSensorHL = 1;
	else
		levelSensorHL = 0;
}
double ACIONNA::get_Pressure()
{
	/*
	Sensor details

    Thread size : G 1/4" (BSP)
    Sensor material:  Carbon steel alloy
    Working voltage: 5 VDC
    Output voltage: 0.5 to 4.5 VDC
    Working Current: <= 10 mA
    Working pressure range: 0 to  1.2 MPa
    Maxi pressure: 2.4 MPa
    Working temperature range: 0 to 100 graus C
    Accuracy: ± 1.0%
    Response time: <= 2.0 ms
    Package include: 1 pc pressure sensor
    Wires : Red---Power (+5V)  Black---Power (0V) - blue ---Pulse singal output


    4.5 V___	   922___	1.2 MPa___	 12 Bar___	 120 m.c.a.___
	  	  |				|			|			|				|
	 	  |				|			|			|				|
	 	  |				|			|			|				|
	  out_|			Pd__|		  __|			|			Pa__|
	 	  |				|			|			|				|
	 	  |				|			|			|				|
	 	  |				|			|			|				|
		 _|_		   _|_		   _|_		   _|_			   _|_
	0.5 V			103			0 MPa		0 Bar		0 m.c.a.

	(out-0.5)/(4.5-0.5) = 1024

	(out-0.0)/(5-0) = (x-0)/(1024-0)

	(Pd - 103)/(922-103) = (Pa - 0)/(120 - 0)
	Pa = 120.0*Pd/(1024.0);

	(xs - 0) = temp - (0)
	(255 - 0)  +50 - (0)

	Direct Conversion
	xs = 255*(temp+0)/51
	tempNow_XS = (uint8_t) 255.0*(tempNow+0.0)/51.0;

	Inverse Conversion
	temp = (TempMax*xs/255) - TempMin
	tempNow = (uint8_t) ((sTempMax*tempNow_XS)/255.0 - sTempMin);
    */

	const double Kpsi = 0.7030768118;
//	const double PRessMax = 68.9475729;	// Sensor max pressure [m.c.a.] with 100 psi;
//	const double PRessMax = 103.4212;	// Sensor max pressure [m.c.a.] with 150 psi;
//	const double PRessMax = 120.658253;	// Sensor max pressure [m.c.a.] with 174.045 psi;

	double PRessMax = Kpsi*PRessureMax_Sensor;

	Pd = adc_readChannel(7);
	return (PRessMax)*(Pd-102.4)/(921.6-102.4);
//	(Pd - 103)/(922-103) = (Pa - 0)/(120 - 0);
}
void ACIONNA::check_pressure()
{
	PRess = get_Pressure();	// Get current pressure

//	switch (stateMode)
//	{
//		case 1:
//			break;
//
//		case 2:
//			break;
//
//		case 3:	// Valve mode case. Do not turn off.
//			break;
//
//		default:
//			break;
//	}
}
void ACIONNA::check_levelSensors()
{
	get_levelSensors();
}
void ACIONNA::check_period()
{
	// Season time verify
	if(((tm.Hour == HourOn) && (tm.Minute == MinOn)) || (tm.Hour > HourOn) || (tm.Hour < HourOff) || ((tm.Hour == HourOff) && (tm.Minute < MinOff)))
	{
		periodo = greenTime;

		if(flag_01)
		{
			flag_01 = 0;
//			flag_timeMatch = 1;
		}
	}

	if (((tm.Hour == HourOff) && (tm.Minute >= MinOff))	|| ((tm.Hour > HourOff) && (tm.Hour < HourOn))	|| ((tm.Hour == HourOn) && (tm.Minute < MinOn)))
	{
		periodo = redTime;

//		flag_timeMatch = 0;
		flag_01 = 1;
	}
}
void ACIONNA::check_timeMatch()
{
	uint8_t i, nTM_var=1;

	// matching time verify
	if(!motorStatus)
	{
		if(stateMode)
		{
			switch (stateMode)
			{
			case 1:
				nTM_var = 1;
				break;

			case 2:
				nTM_var = nTM;
				break;

			case 3:
				nTM_var = nTM;
				break;
			}

			for(i=0;i<nTM_var;i++)
			{
				if((tm.Hour == HourOnTM[i]) && (tm.Minute == MinOnTM[i]))
				{
					flag_timeMatch = 1;	// IF conditions are OK, SET (flag_timeMatch) variable;
				}
			}
		}
	}
}
void ACIONNA::check_thermalSafe()
{
	if(motorStatus)
	{
		if(readPin_Rth())
		{
			uint16_t countThermal = 50000;
			Serial.println("Th0");
			while(readPin_Rth() && countThermal)
			{
				countThermal--;
			}
			Serial.println("Th1");
			if(!countThermal)
			{
				flag_Th = 1;
			}
			else
				flag_Th = 0;
		}
		else
		{
			flag_Th = 0;
		}
	}
}
void ACIONNA::check_gpio()
{
	if(startTypeK == 1)
	{
		motorStatus = readPin_k1_PIN();

	}
	else
	{
		motorStatus = readPin_k1();
	}
}
void ACIONNA::check_TimerVar()
{
	if(motorStatus)	// Load is ON;
	{
		if(timeOn_sec > 59)
		{
			timeOn_sec = 0;
			timeOn_min++;
		}
		else
		{
			timeOn_sec++;
		}
	}
	else			// Load is OFF;
	{
		if(timeOff_sec > 59)
		{
			timeOff_sec = 0;
			timeOff_min++;
		}
		else
		{
			timeOff_sec++;
		}
	}

	if(flag_waitPowerOn)
	{
		if(waitPowerOn_sec == 0)
		{
			if(waitPowerOn_min == 0)
			{
				flag_waitPowerOn = 0;
			}
			else
			{
				waitPowerOn_sec = 59;
				waitPowerOn_min--;
			}
		}
		else
		{
			waitPowerOn_sec--;
		}
	}
}
void ACIONNA::check_pressureDown()
{
	if(stateMode == 5)
	{
		if(motorStatus)
		{
			if(PRess > PRessHold)
			{
				PRessHold = PRess;
			}
			else if(PRess < ((PRessurePer/100.0)*PRessHold))
			{
				flag_PressureDown = 1;
			}
		}
	}
}
void ACIONNA::process_valveControl()
{
	if(flag_timeMatch && (PRess >= PRessureRef_Valve))
	{
		flag_timeMatch &= 0;

		if(!motorStatus)
		{
			motor_start();
		}
	}
}
void ACIONNA::process_waterPumpControl()
{
	/*
	 * 0x01 - pressure
	 * 0x02 - level
	 * 0x03 - thermal
	 * 0x04 - time out
	 * 0x05 - red time
	 * 0x06 - command line
	 * 0x07 - broke pressure
	 * */

	if(!levelSensorLL)
	{
		if(motorStatus)
		{
			motor_stop(0x02);
		}
	}

	if(levelSensorHL && (stateMode == 4) && levelSensorLL)
	{
		if(!motorStatus)
		{
			motor_start();
		}
	}

	if(flag_timeMatch && (stateMode != 4))
	{
		flag_timeMatch = 0;

		if(!motorStatus)
		{
			motor_start();
		}
	}

	if(PRessureRef)					// Has a valid number mean this function is activated
	{
		if(PRess >= PRessureRef)
		{
			if(motorStatus)
			{
				motor_stop(0x01);
			}
		}
	}

	if(stateMode == 5)
	{
		if(motorStatus && flag_PressureDown)
		{
			flag_PressureDown = 0;
			motor_stop(0x07);
		}
	}

	if(flag_Th)
	{
		motor_stop(0x03);
		stateMode = 0;
		//eeprom_write_byte(( uint8_t *)(addr_stateMode), stateMode);
	}
}
void ACIONNA::process_motorPeriodDecision()
{
	switch (periodo)
	{
	case redTime:
		if(motorStatus)
		{
			motor_stop(0x05);
		}
		break;

	case greenTime:
		process_waterPumpControl();
		break;
	}
}
void ACIONNA::process_Mode()
{
	switch (stateMode)
	{
		case 0:	// System Down!
			break;

		case 1:	// Night Working;
			process_motorPeriodDecision();
			break;

		case 2:	// For irrigation mode. Start in a programmed time.
			process_waterPumpControl();
			break;

		case 3:	// For reservoir only. Works in a inverted pressured! Caution!
			process_valveControl();
			break;

		case 4:	// Is that for a water pump controlled by water sensors. Do not use programmed time.
			process_waterPumpControl();
			break;

		case 5:	// For irrigation mode and instantly low pressure turn motor off.
			process_waterPumpControl();
			break;

		default:
			stateMode = 0;
			Serial.println("Standby");
			break;
	}

	// maximum time drive keeps turned ON
	if(motorTimerE)
	{
		if(motorStatus)
		{
			if(timeOn_min >= motorTimerE)
			{
				motor_stop(0x04);
			}
		}
	}
}
void ACIONNA::summary_Print(uint8_t opt)
{
	switch (opt)
	{
		case 0:
			sprintf(buffer,"Time:%.2d:%.2d:%.2d %.2d/%.2d",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month);//, tmYearToCalendar(tm.Year));
			Serial.print(buffer);

//			sprintf(buffer," UP:%.2d:%.2d:%.2d, d:%d m:%d", hour(), minute(), second(), day()-1, month()-1);
			Serial.println(buffer);

			sprintf(buffer," P:%d k1:%d",periodo, motorStatus);
			Serial.println(buffer);

			switch (stateMode)
			{
				case 0:
					strcpy(buffer," Modo:Desligado");
					break;

				case 1:
					sprintf(buffer," Modo:Liga Noite");
					break;

				case 2:
					if(nTM == 1)
					{
						sprintf(buffer," Irrig: Liga %dx as %2.d:%.2d", nTM, HourOnTM[0], MinOnTM[0]);
					}
					else
					{
						sprintf(buffer," Irrig: Liga %dx/dia",nTM);
					}
					break;

				case 3:
					if(nTM == 1)
					{
						sprintf(buffer," Valve: Liga %dx as %2.d:%.2d", nTM, HourOnTM[0], MinOnTM[0]);
					}
					else
					{
						sprintf(buffer," Valve: Liga %dx/dia",nTM);
					}
					break;

				case 4:
					sprintf(buffer," Modo: Auto HL");
					break;

				case 5:
					if(nTM == 1)
					{
						sprintf(buffer," IrrigLow: Liga %dx as %2.d:%.2d", nTM, HourOnTM[0], MinOnTM[0]);
					}
					else
					{
						sprintf(buffer," IrrigLow: Liga %dx/dia",nTM);
					}
					break;
					break;


				default:
					strcpy(buffer,"sMode Err");
					break;
			}
			Serial.println(buffer);
			break;

		case 1:
			int i;
			if(motorStatus)
			{
				for(i=(nLog-1);i>=0;i--)
				{
					memset(buffer,0,sizeof(buffer));
					sprintf(buffer,"OFF_%.2d: %.2d:%.2d, %.2d/%.2d ",(i+1),hourLog_OFF[i], minuteLog_OFF[i], dayLog_OFF[i], monthLog_OFF[i]);
					Serial.println(buffer);
					_delay_ms(20);

					memset(buffer,0,sizeof(buffer));
					sprintf(buffer,"ON__%.2d: %.2d:%.2d, %.2d/%.2d ",(i+1),hourLog_ON[i], minuteLog_ON[i], dayLog_ON[i], monthLog_ON[i]);
					Serial.println(buffer);
					_delay_ms(20);
				}
			}
			else
			{
				for(i=(nLog-1);i>=0;i--) //	for(i=0;i<nLog;i++)
				{
					memset(buffer,0,sizeof(buffer));
					sprintf(buffer,"ON__%.2d: %.2d:%.2d, %.2d/%.2d " ,(i+1),hourLog_ON[i], minuteLog_ON[i], dayLog_ON[i], monthLog_ON[i]);
					Serial.println(buffer);
					_delay_ms(20);

					memset(buffer,0,sizeof(buffer));
					sprintf(buffer,"OFF_%.2d: %.2d:%.2d, %.2d/%.2d ",(i+1),hourLog_OFF[i], minuteLog_OFF[i], dayLog_OFF[i], monthLog_OFF[i]);
					Serial.println(buffer);
					_delay_ms(20);
				}
			}
			sprintf(buffer,"r0:%d, r1:%d ",reasonV[0], reasonV[1]);
			Serial.println(buffer);
			break;

		case 2:
			sprintf(buffer,"f:%d t1:%.2d:%.2d c%dmin t2:%.2d:%.2d s:%dmin ", flag_waitPowerOn, waitPowerOn_min, waitPowerOn_sec, waitPowerOn_min_standBy, timeOn_min, timeOn_sec, motorTimerE);
			Serial.println(buffer);
			break;

		case 3:
			sprintf(buffer,"Motor:%d Fth:%d Rth:%d Pr:%d ", motorStatus, flag_Th, readPin_Rth(), PRess);
			Serial.print(buffer);
			switch (stateMode)
			{
				case 3:
					sprintf(buffer,"Pref:%d ", PRessureRef_Valve);
					Serial.println(buffer);
					break;

				case 5:
					sprintf(buffer,"Pref:%d, Pper:%d ", PRessureRef, PRessurePer);
					Serial.println(buffer);
					break;

				default:
					sprintf(buffer,"Pref:%d ", PRessureRef);
					Serial.println(buffer);
					break;

			}
			break;

		case 4:
			sprintf(buffer,"LL:%d ML:%d HL:%d ",levelSensorLL, levelSensorML, levelSensorHL);
			Serial.println(buffer);

			sprintf(buffer,"LL:%d ML:%d HL:%d",levelSensorLL_d, levelSensorML_d, levelSensorHL_d);
			Serial.println(buffer);
			break;

		case 5:
			for(i=0;i<nTM;i++)
			{
				sprintf(buffer,"h%d: %.2d:%.2d",i+1, HourOnTM[i], MinOnTM[i]);
				Serial.println(buffer);
			}
			break;

		case 6:
			sprintf(buffer,"tON:%.2d:%.2d ",timeOn_min, timeOn_sec);
			Serial.println(buffer);
			sprintf(buffer,"tOFF:%.2d:%.2d",timeOff_min, timeOff_sec);
			Serial.println(buffer);
			break;

		case 7:
			sprintf(buffer,"P:%d Fth:%d Rth:%d Ftm:%d k1:%d k3:%d", PRess, flag_Th, readPin_Rth(), flag_timeMatch, motorStatus, readPin_k3());
			Serial.println(buffer);

			sprintf(buffer,"LL:%d ML:%d HL:%d  ",levelSensorLL, levelSensorML, levelSensorHL);
			Serial.println(buffer);

			sprintf(buffer,"LL:%d ML:%d HL:%d  ",levelSensorLL_d, levelSensorML_d, levelSensorHL_d);
			Serial.println(buffer);
			break;

		case 8:
//			sprintf(buffer,"WDRF:%d BORF:%d EXTRF:%d PORF:%d", flag_WDRF, flag_BORF, flag_EXTRF, flag_PORF);
//			Serial.println(buffer);
			break;

		case 9:
			sprintf(buffer,"Err");
			Serial.println(buffer);
			break;

		default:
			sprintf(buffer,"not implemented");
			Serial.println(buffer);
			break;
	}
}
void ACIONNA::RTC_update()
{
	if(tm.Second == 59)
	{
		tm.Second = 0;
		if(tm.Minute == 59)
		{
			tm.Minute = 0;
			if(tm.Hour == 23)
			{
				tm.Hour = 0;
				uint8_t month31, month30;
				if((tm.Month == 1) || (tm.Month == 3) || (tm.Month == 5) || (tm.Month == 7) || (tm.Month == 8) || (tm.Month == 10) || (tm.Month == 12))
				{
					month31 = 1;
				}
				else
				{
					month30 = 1;
				}

				if((tm.Day == 30 && month30) || (tm.Day == 31 && month31))
				{
					tm.Day = 1;
					if(tm.Month == 12)
					{
						tm.Month = 1;
						tm.Year++;
					}
					else
					{
						tm.Month++;
					}
				}
				else
				{
					tm.Day++;
				}
			}
			else
			{
				tm.Hour++;
			}
		}
		else
		{
			tm.Minute++;
		}
	}
	else
	{
		tm.Second++;
	}
}
void ACIONNA::refreshVariables()
{
	if (flag_1s)
	{
		flag_1s = 0;
		RTC_update();
//		RTC.read(tm);

		check_gpio();			// Check drive status pin;
		check_period();			// Period verify;
		check_timeMatch();		// time matches flag;
		check_TimerVar();		// drive timers

		check_pressure();		// get and check pressure system;
		check_thermalSafe();	// thermal relay check;
		check_levelSensors();	// level sensors;

		check_pressureDown();

		if(flag_debug)
		{
			summary_Print(7);
		}
	}
}
void ACIONNA::refreshStoredData()
{
	stateMode = //eeprom_read_byte((uint8_t *)(addr_stateMode));

	waitPowerOn_min_standBy = //eeprom_read_byte((uint8_t *)(addr_standBy_min));
	waitPowerOn_min = waitPowerOn_min_standBy;

	uint8_t lbyte, hbyte;
	hbyte = //eeprom_read_byte((uint8_t *)(addr_LevelRef+1));
	lbyte = //eeprom_read_byte((uint8_t *)(addr_LevelRef));
	levelRef_10bit = ((hbyte << 8) | lbyte);

	hbyte = //eeprom_read_byte((uint8_t *)(addr_motorTimerE+1));
	lbyte = //eeprom_read_byte((uint8_t *)(addr_motorTimerE));
	motorTimerE = ((hbyte << 8) | lbyte);

//	PRessureRef = //eeprom_read_byte((uint8_t *)(addr_PRessureRef));
//	PRessureRef_Valve = //eeprom_read_byte((uint8_t *)(addr_PRessureRef_Valve));
//	PRessurePer = //eeprom_read_byte((uint8_t *)(addr_PREssurePer));
//	PRessureMax_Sensor = //eeprom_read_byte((uint8_t *)(addr_PRessureMax_Sensor));
//
//	nTM = //eeprom_read_byte((uint8_t *)(addr_nTM));
//	int i;
//	for(i=0;i<9;i++)
//	{
////		HourOnTM[i] = //eeprom_read_byte((uint8_t *)(addr_HourOnTM+i));
////		MinOnTM[i] = //eeprom_read_byte((uint8_t *)(addr_MinOnTM+i));
//	}

}
void ACIONNA::handleMessage()
{
/*
$0X;				Verificar detalhes - Detalhes simples (tempo).
	$00;			- Detalhes simples (tempo).
	$01;			- Verifica histórico de quando ligou e desligou;
	$02;			- Mostra tempo que falta para ligar;
		$02:c;		- Zera o tempo;
		$02:c:30;	- Ajusta novo tempo para 30 min;
		$02:s:090;	- Tempo máximo ligado para 90 min. Para não utilizar, colocar zero;
	$03;			- Verifica detalhes do motor, pressão e sensor termico;
		$03:s:72;	- Set pressure ref [m.c.a.];
		$03:v:32;	- Set pressure for valve load turn on and fill reservoir;
		$03:p:150;	- Set sensor max pressure ref to change the scale [psi];
		$03:b:85;	- Set to 85% the pressure min bellow the current pressure to avoid pipe broken;
	$04;			- Verifica detalhes do nível de água no poço e referência 10 bits;
		$04:0;		- Interrompe o envio continuo das variáveis de pressão e nível;
		$04:1;		- Envia continuamente valores de pressão e nível;
		$04:0900;	- Adiciona nova referência para os sensores de nível. Valor de 0 a 1023;
	$05;			- Mostra os horários que liga no modo $62;
	$06;			- Tempo ligado e tempo desligado;
	$07:x;			- ADC reference change.
		$07:0;		- AREF
		$07:1;		- AVCC with external cap at AREF pin
		$07:2;		- Internal 1.1 Voltage reference.
	$08;			- Motivo do reboot.
	$09;			- Reinicia o sistema.

$1:h:HHMMSS;		- Ajustes do calendário;
	$1:h:HHMMSS;	- Ajusta o horário do sistema;
	$1:h:123040;	- E.g. ajusta a hora para 12:30:40
	$1:d:DDMMAAAA;	- Ajusta a data do sistema no formato dia/mês/ano(4 dígitos);
	$1:d:04091986;	- E.g. Altera a data para 04/09/1986;

$2:DevName;			- Change bluetooth name;
	$2:Vassalo;		- Altera o nome do bluetooth para "Vassalo";

$3X;				- Acionamento do motor;
	$31;			- liga o motor;
	$30;			- desliga o motor;

$4:a1:0;
$4:a2:0;
$4:a3:0;
$4:a4:0;

$5:n:X; ou $5:hX:HHMM;
	$5:n:9;			- Configura para acionar 9 vezes. Necessário configurar 9 horários;
	$5:n:9;			- Configura o sistema para acionar uma única vez às 21:30 horas;
	$5:h1:2130;		- Configura o primeiro horário como 21:30 horas;
	$5:h8:0437;		- Configura o oitavo horário como 04:37 horas;

$6X;				- Modos de funcionamento;
	$60; 			- Sistema Desligado (nunca ligará);
	$61;			- Liga somente à noite. Sensor superior;
	$62;			- Liga nos determinados horários estipulados;
	$63;			- Função para válvula do reservatório;
	$64;			- Função para motobomba do reservatório;
*/
	// Tx - Transmitter
	if(enableDecode)
	{
		enableDecode = 0;

//		int i;
//		for(i=0;i<rLength;i++)
//		{
//			Serial1.println(sInstr[i]);
//		}
//		for(i=0;i<rLength;i++)
//		{
//			Serial1.println(sInstr[i],HEX);
//		}

		// Getting the opcode
		aux[0] = '0';
		aux[1] = sInstr[0];
		aux[2] = '\0';
		opcode = (uint8_t) atoi(aux);
//		Serial1.println("Got!");
		uint8_t statusCommand = 0;

		switch (opcode)
		{
// -----------------------------------------------------------------
			case 0:		// Check status
			{
				aux[0] = '0';
				aux[1] = sInstr[1];
				aux[2] = '\0';
				statusCommand = (uint8_t) atoi(aux);

				switch (statusCommand)
				{
					// ----------
					// $02:c;  -> clear time counter;
					// $02:c:mm;  -> set time counter ref;
					case 2:
					{
						if(sInstr[2]==':' && sInstr[3]=='c')
						{
							if(sInstr[4] == ';')
							{
								flag_waitPowerOn = 0;
								waitPowerOn_min = 0;
								waitPowerOn_sec = 0;
							}
							else if(sInstr[4] ==':' && sInstr[7] == ';')
							{
								aux[0] = sInstr[5];
								aux[1] = sInstr[6];
								aux[2] = '\0';
								waitPowerOn_min_standBy = (uint8_t) atoi(aux);

								//eeprom_write_byte((uint8_t *)(addr_standBy_min), waitPowerOn_min_standBy);

//								Serial.print("powerOn min:");
//								Serial.println(powerOn_min_Standy);
							}
						}//$02:s:129;
						else if(sInstr[2] == ':' && sInstr[3] == 's' && sInstr[4] == ':' && sInstr[8] == ';')
						{
							aux2[0] = '0';
							aux2[1] = sInstr[5];
							aux2[2] = sInstr[6];
							aux2[3] = sInstr[7];
							aux2[4] = '\0';
							motorTimerE = (uint16_t) atoi(aux2);

							uint8_t lbyteRef = 0, hbyteRef = 0;
							lbyteRef = motorTimerE;
							hbyteRef = (motorTimerE >> 8);

							//eeprom_write_byte((uint8_t *)(addr_motorTimerE+1), hbyteRef);
							//eeprom_write_byte((uint8_t *)(addr_motorTimerE), lbyteRef);
						}
						summary_Print(statusCommand);
					}
					break;
					// ------------------------------
					case 3:// $03:s:68;
						if(sInstr[2]==':' && sInstr[3] == 's' && sInstr[4] == ':' && sInstr[7] == ';')
						{
							aux[0] = sInstr[5];
							aux[1] = sInstr[6];
							aux[2] = '\0';
							PRessureRef = (uint8_t) atoi(aux);
							//eeprom_write_byte((uint8_t *)(addr_PRessureRef), PRessureRef);

							sprintf(buffer,"PRessRef: %d", PRessureRef);
							Serial.println(buffer);
						}
						else if(sInstr[2]==':' && sInstr[3] == 'v' && sInstr[4] == ':' && sInstr[7] == ';')
						{
							aux[0] = sInstr[5];
							aux[1] = sInstr[6];
							aux[2] = '\0';
							PRessureRef_Valve = (uint8_t) atoi(aux);
							//eeprom_write_byte((uint8_t *)(addr_PRessureRef_Valve), PRessureRef_Valve);

							sprintf(buffer,"PRessRef_Valve: %d", PRessureRef_Valve);
							Serial.println(buffer);
						}
						else if(sInstr[2]==':' && sInstr[3] == 'p' && sInstr[4] == ':' && sInstr[8] == ';')
						{// $03:p:150;
							aux2[0] = '0';
							aux2[1] = sInstr[5];
							aux2[2] = sInstr[6];
							aux2[3] = sInstr[7];
							aux2[4] = '\0';
							PRessureMax_Sensor = (uint8_t) atoi(aux2);

							//eeprom_write_byte((uint8_t *)(addr_PRessureMax_Sensor), PRessureMax_Sensor);
						}
						else if(sInstr[2]==':' && sInstr[3] == 'b' && sInstr[4] == ':' && sInstr[7] == ';')
						{
							aux[0] = sInstr[5];
							aux[1] = sInstr[6];
							aux[2] = '\0';
							PRessurePer = (uint8_t) atoi(aux);
							//eeprom_write_byte((uint8_t *)(addr_PREssurePer), PRessurePer);
						}
						else
							summary_Print(statusCommand);
						break;
					// ------------------------------
					case 4: // $04:1;
						if(sInstr[2]==':' && sInstr[4]==';')
						{
							aux[0] = '0';
							aux[1] = sInstr[3];
							aux[2] = '\0';
							uint8_t debugCommand;
							debugCommand = (uint8_t) atoi(aux);
							flag_debug = debugCommand;
						}//$04:s:0900;
						else if(sInstr[2]==':' && sInstr[3]=='s' && sInstr[4]==':' && sInstr[9]==';')
						{
							aux2[0] = sInstr[5];
							aux2[1] = sInstr[6];
							aux2[2] = sInstr[7];
							aux2[3] = sInstr[8];
							aux2[4] = '\0';

							levelRef_10bit = (uint16_t) atoi(aux2);

							uint8_t lbyteRef = 0, hbyteRef = 0;
							lbyteRef = levelRef_10bit;
							hbyteRef = (levelRef_10bit >> 8);

							//eeprom_write_byte((uint8_t *)(addr_LevelRef+1), hbyteRef);
							//eeprom_write_byte((uint8_t *)(addr_LevelRef), lbyteRef);
						}
						summary_Print(statusCommand);
						sprintf(buffer,"Ref: %d", levelRef_10bit);
						Serial.println(buffer);
						break;
					// ------------------------------
					case 7:
						if(sInstr[2]==':' && sInstr[4]==';')
						{
							aux[0] = '0';
							aux[1] = sInstr[3];
							aux[2] = '\0';
							uint8_t adcCommand = (uint8_t) atoi(aux);

							switch (adcCommand)
							{
//								case 0:
//									ADMUX &=  ~(1<<REFS1);		// AREF, Internal Vref turned off
//									ADMUX &=  ~(1<<REFS0);
//									Serial.println("AREF");
//									break;
//
//								case 1:
//									ADMUX &=  ~(1<<REFS1);		// AVCC with external capacitor at AREF pin
//									ADMUX |=   (1<<REFS0);
//									Serial.println("AVCC");
//									break;
//
//								case 2:
//									ADMUX |=   (1<<REFS1);		// Internal 1.1V Voltage Reference with external capacitor at AREF pin
//									ADMUX |=   (1<<REFS0);
//									Serial.println("1.1V");
//									break;
							}
						}
						break;
					// ------------------------------
					case 9:
						Serial.println("Rebooting...");
						//wdt_enable(WDTO_15MS);
						_delay_ms(100);
//						flag_reset = 1;
						break;

					default:
						summary_Print(statusCommand);
						break;
				}
			}
			break;
// -----------------------------------------------------------------
			case 1: // Setup calendar
			{
				// Set-up clock -> $1:h:HHMMSS;
				if(sInstr[1]==':' && sInstr[2]=='h' && sInstr[3]==':' && sInstr[10]==';')
				{
					aux[0] = sInstr[4];
					aux[1] = sInstr[5];
					aux[2] = '\0';
					tm.Hour = (uint8_t) atoi(aux);

					aux[0] = sInstr[6];
					aux[1] = sInstr[7];
					aux[2] = '\0';
					tm.Minute = (uint8_t) atoi(aux);

					aux[0] = sInstr[8];
					aux[1] = sInstr[9];
					aux[2] = '\0';
					tm.Second = (uint8_t) atoi(aux);

//					RTC.write(tm);
					summary_Print(0);
				}
				// 	Set-up date -> $1:d:DDMMAAAA;
				else if(sInstr[1]==':' && sInstr[2]=='d' && sInstr[3]==':' && sInstr[12]==';')
				{
					// Getting the parameters
					aux[0] = sInstr[4];
					aux[1] = sInstr[5];
					aux[2] = '\0';
					tm.Day = (uint8_t) atoi(aux);

					aux[0] = sInstr[6];
					aux[1] = sInstr[7];
					aux[2] = '\0';
					tm.Month = (uint8_t) atoi(aux);

					char aux2[5];
					aux2[0] = sInstr[8];
					aux2[1] = sInstr[9];
					aux2[2] = sInstr[10];
					aux2[3] = sInstr[11];
					aux2[4] = '\0';
					tm.Year = (uint8_t) (atoi(aux2)-1970);

//					RTC.write(tm);

					summary_Print(0);

				}
			}
			break;
// -----------------------------------------------------------------
			case 2:		// Setup Bluetooth device name
			{
				// Setup clock -> $2:BluetoothName;
				char aux_str[sInstr_SIZE], aux_str2[sInstr_SIZE+7];
				uint8_t k1=0;
				if(sInstr[1]==':')
				{
					// 3 because 2 and : count 2 characters and one more for '\0'
					while((k1<sInstr_SIZE-3) && sInstr[k1] != ';')
					{
						aux_str[k1] = sInstr[k1+2];
						k1++;
//						Serial.println(k1);
					}

					aux_str[k1-2] = '\0';
					Serial.println("Disconnect!");
//					wdt_reset();
//					_delay_ms(3000);
//					wdt_reset();
//					_delay_ms(3000);
//					wdt_reset();
//					_delay_ms(3000);
//					strcpy(aux_str2,"AT");
//					Serial.print(aux_str2);
					//wdt_reset();
					_delay_ms(3000);
					strcpy(aux_str2,"AT+NAME");
					strcat(aux_str2,aux_str);
					Serial.print(aux_str2);
					//wdt_reset();
					_delay_ms(1000);
				}
			}
			break;
// -----------------------------------------------------------------
			case 3:		// Set motor ON/OFF
			{
				uint8_t motorCommand;
				aux[0] = '0';
				aux[1] = sInstr[1];
				aux[2] = '\0';
				motorCommand = (uint8_t) atoi(aux);

				if (motorCommand && (!motorStatus))
				{
					motor_start();
				}
				else
				{
					motor_stop(0x06);
				}

				summary_Print(3);
			}
			break;
// -----------------------------------------------------------------

// -----------------------------------------------------------------
			case 5: // Command is $5:h1:2130;
			{
				if(sInstr[1]==':' && sInstr[2]=='h' && sInstr[4]==':' && sInstr[9]==';')
				{
					aux[0] = '0';
					aux[1] = sInstr[3];
					aux[2] = '\0';
					uint8_t indexV = (uint8_t) atoi(aux);

					aux[0] = sInstr[5];
					aux[1] = sInstr[6];
					aux[2] = '\0';
					HourOnTM[indexV-1] = (uint8_t) atoi(aux);
					//eeprom_write_byte(( uint8_t *)(addr_HourOnTM+indexV-1), HourOnTM[indexV-1]);

					aux[0] = sInstr[7];
					aux[1] = sInstr[8];
					aux[2] = '\0';
					MinOnTM[indexV-1] = (uint8_t) atoi(aux);
					//eeprom_write_byte(( uint8_t *)(addr_MinOnTM+indexV-1), MinOnTM[indexV-1]);

					summary_Print(5);
				}
				else if(sInstr[1]==':' && sInstr[2]=='n' && sInstr[3]==':' && sInstr[5]==';')
				{
					aux[0] = '0';
					aux[1] = sInstr[4];
					aux[2] = '\0';

					nTM = (uint8_t) atoi(aux);
					//eeprom_write_byte(( uint8_t *)(addr_nTM), nTM);

					summary_Print(5);
				}
				else if(sInstr[1]==';')
				{
					summary_Print(5);
				}
			}
			break;
// ----------------------------------------------------------------
			case 6:		// Set working mode
			{
				aux[0] = '0';
				aux[1] = sInstr[1];
				aux[2] = '\0';
				stateMode = (uint8_t) atoi(aux);
//				//eeprom_write_byte(( uint8_t *)(addr_stateMode), stateMode);

				summary_Print(0);
			}
			break;
// -----------------------------------------------------------------
			default:
				summary_Print(10);
				break;
		}
		memset(sInstr,0,sizeof(sInstr));	// Clear all vector;
	}
}
void ACIONNA::comm_Bluetooth()
{
	// Rx - Always listening
//	uint8_t j2 =0;
	while((Serial.available()>0))	// Reading from serial
	{
		inChar = Serial.read();

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

			enableDecode = 1;
		}
		else
		{
//			Serial.println("Err");
//			Serial.write(pi0[0]);
//			Serial.write(pf0[0]);
		}
	}
}


#endif /* HARDWARE_H_ */




#endif /* ACIONNA_H_ */
