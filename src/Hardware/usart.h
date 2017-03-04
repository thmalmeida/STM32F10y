///*
// * usart.h
// *
// *  Created on: 25 de jun de 2016
// *      Author: titi
// */
#ifndef USART_H_
#define USART_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stm32f10x.h>

class USART {
public:

	char buffer[30];
	volatile int _USART1_cnt;
	unsigned char _received_string[50];
//	extern __IO int USART1_cnt;
//	extern __IO char received_string[MAX_STRLEN+1]; // this will hold the recieved string


	void begin(uint32_t baudRate);

	void print(char const *s);
	void println(char const *s);
	void println(int value);
	void write(char c);
	int available();
	char read();

private:
	static void USART1_IRQHandler(void);

};
#endif
