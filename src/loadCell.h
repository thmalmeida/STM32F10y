/*
 * loadCell.h
 *
 *  Created on: 23 de mai de 2017
 *      Author: titi
 */

#ifndef LOADCELL_H_
#define LOADCELL_H_

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
#include "Hardware/spi.h"
#include "Hardware/gpio.h"

#define pin_data_HX711	32
#define pin_sck_HX711	31
#define pin_tare_HX711	30
#define pin_led			1

class LOADCELL : public GPIO , ADC {
public:

	int timeD = 1;
	int offset = 0;					// offset after tare;
	int WeightX10;					// currently weight x10;

	static const int nWeight = 200;

	int WeightVect[nWeight];
	int signalVect[nWeight];

	static const int Waccu = 10000;
	static const int Werror = Waccu*0.10;// 1000;
	double Kc = 1.3299;				// proportional constant;
	double Vrange = 20.0;			// Small signal scale range [mV];
	double scaleHalf = 8388607.0;	// ((2^24)/2)-1;
	double Wmax = 1000.0;			// Sensor max weight [g];
	double Vref = 4.97;				// Voltage reference [V];

	void drive_led(uint8_t status);
	void begin_loadcell();
	int readInput();
	void pin_sck_set(uint8_t status);
	void pin_data_set(uint8_t status);
	uint32_t pin_data_get();
	int get_weight();

	void tareSystem();
	void tareSystem2();
	void tareSystem3();
	int readTareButton();

private:
};

void LOADCELL::begin_loadcell()
{
	gateConfig(pin_data_HX711, 0);	// data pin
	gateConfig(pin_sck_HX711, 1);	// sck pin
	gateConfig(pin_tare_HX711, 0);	// data pin
	gateConfig(pin_led, 1);			// led

//	tareSystem3();
}
void LOADCELL::tareSystem()
{
	int i, n = 100;

	for(i=0;i<n;i++)
	{
		offset += readInput();
	}
	offset = offset/n;
}
void LOADCELL::tareSystem2()
{
	while(get_weight() != 0)
	{
		get_weight();

		int Ssum = 0;
		for(int i=0; i< nWeight ;i++)
		{
			Ssum+= signalVect[i];
		}
		offset = Ssum/nWeight;
	}
}
void LOADCELL::tareSystem3()
{
	int Ssum = 0;
	for(int i=0; i<nWeight; i++)
	{
		Ssum+= signalVect[i];
	}
	offset = Ssum/nWeight;
}
void LOADCELL::pin_sck_set(uint8_t status)
{
	gateSet(pin_sck_HX711, status);
}
uint32_t LOADCELL::pin_data_get()
{
	return (uint32_t) gateRead(pin_data_HX711, 0);
}
void LOADCELL::pin_data_set(uint8_t status)
{
	gateSet(pin_data_HX711, status);
}
int LOADCELL::readInput()
{
	int i, cycles = 24;
	int Count = 0;

	pin_data_set(1);
	pin_sck_set(0);
	while(pin_data_get());

	for(i=0;i<cycles;i++)
	{
		pin_sck_set(1);
		_delay_us(timeD);

		Count = Count << 1;

		pin_sck_set(0);
		_delay_us(timeD);

		if(pin_data_get())
		{
//			Count++;
			Count |= 1;
		}

//		weigth = ((weigth << 1) | (pin_data_get()));
	}

	// 25 clk
	pin_sck_set(1);
	_delay_us(timeD);
	pin_sck_set(0);
	_delay_us(timeD);

//	// 26 clk
//	pin_sck_set(1);
//	_delay_us(timeD);
//	pin_sck_set(0);
//	_delay_us(timeD);
//
//	// 27 clk
//	pin_sck_set(1);
//	_delay_us(timeD);
//	pin_sck_set(0);
//	_delay_us(timeD);

//	Count ^= 0xFF800000;
//	Count = ~Count + 1 ;
//	Count &= 0x007FFFFF;

	return Count;
}
int LOADCELL::readTareButton()
{
	uint8_t status = 0;
	status = !gateRead(pin_tare_HX711, 0);

	if(status)
	{
		_delay_ms(50);
	}

	return status;
}
int LOADCELL::get_weight(void)
{
	int signal = readInput();
	double Vdig = (signal - offset);
	double a = (Kc*Vrange*Vdig)/scaleHalf;
	int WeightTemp = (int) Waccu*(a*Wmax/Vref);

	int error = WeightTemp - WeightX10;
	if(abs(error) > Werror)
	{
		for(int i=1; i<nWeight;i++)
		{
			signalVect[i] = signal;
			WeightVect[i] = WeightTemp;
		}
		WeightX10 = WeightTemp;
	}
	else
	{
		int Wsum = 0;
		for(int i=(nWeight-1);i>0;i--)
		{
			signalVect[i] = signalVect[i-1];
			WeightVect[i] = WeightVect[i-1];
			Wsum+= WeightVect[i];
		}
		signalVect[0] = signal;
		WeightVect[0] = WeightTemp;
		Wsum+= WeightVect[0];
		WeightX10 = Wsum/nWeight;
	}

	return WeightX10;
}
void LOADCELL::drive_led(uint8_t status)
{
	gateSet(pin_led, status);
}
#endif /* HARDWARE_H_ */

#endif /* ACIONNA_H_ */
