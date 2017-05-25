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

class LOADCELL : public GPIO , ADC {
public:

	int timeD = 1;

	void drive_led(uint8_t status);
	void begin_loadcell();
	uint32_t readInput();
	void pin_sck_set(uint8_t status);
	void pin_data_set(uint8_t status);
	uint32_t pin_data_get();
	int get_weight();

private:
};

void LOADCELL::drive_led(uint8_t status)
{
	gateSet(1, status);
}
void LOADCELL::begin_loadcell()
{
	gateConfig(31, 1);	// sck pin
	gateConfig(32, 0);	// data pin
	gateConfig(1, 1);	// led
}
void LOADCELL::pin_sck_set(uint8_t status)
{
	gateSet(31, status);
}
uint32_t LOADCELL::pin_data_get()
{
	return (uint32_t) gateRead(32, 0);
}
void LOADCELL::pin_data_set(uint8_t status)
{
	gateSet(32, status);
}
uint32_t LOADCELL::readInput()
{
	int i, cycles = 24;
	uint32_t Count = 0;

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
int LOADCELL::get_weight()
{
	int d = readInput();
	int a = (int) (20.0*d)/16777216.0;
	int P = (int) a*1000.0/4.09*10;

	return P;
}
#endif /* HARDWARE_H_ */

#endif /* ACIONNA_H_ */
