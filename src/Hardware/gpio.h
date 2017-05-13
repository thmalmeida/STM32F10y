/*
 * gpio.h
 *
 *  Created on: 22 de fev de 2017
 *      Author: titi
 */

#ifndef HARDWARE_GPIO_H_
#define HARDWARE_GPIO_H_

class GPIO {
public:

	void gateConfig(uint8_t pin, uint8_t dir);
	void gateSet(uint8_t pin, uint8_t status);
	void gateToggle(uint8_t pin);
	uint8_t gateRead(uint8_t pin, uint8_t reg);
};

void GPIO::gateConfig(uint8_t pin, uint8_t dir)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	if(dir)
	{
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	}
	else
	{
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	}

	switch (pin)
	{
		case 1:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
			GPIO_Init(GPIOC, &GPIO_InitStructure);
			break;

		case 2:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
			GPIO_Init(GPIOC, &GPIO_InitStructure);
			break;

		case 3:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
			GPIO_Init(GPIOC, &GPIO_InitStructure);
			break;

		case 4:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			break;

		case 5:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			break;

		case 25:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			break;

		case 26:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
			GPIO_Init(GPIOB, &GPIO_InitStructure);
			break;

		case 27:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
			GPIO_Init(GPIOB, &GPIO_InitStructure);
			break;

		case 28:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
			GPIO_Init(GPIOB, &GPIO_InitStructure);
			break;

		case 29:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
			GPIO_Init(GPIOB, &GPIO_InitStructure);
			break;

		case 30:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
			GPIO_Init(GPIOB, &GPIO_InitStructure);
			break;

		case 31:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
			GPIO_Init(GPIOB, &GPIO_InitStructure);
			break;

		case 32:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
			GPIO_Init(GPIOB, &GPIO_InitStructure);
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
//				GPIOC -> BSRR = (1<<(13+16)); // 16 bit shift
				GPIOC -> BSRR = (1<<13);
//				GPIO_SetBits(GPIOC, 13);
			}
			else
			{
				GPIOC -> BRR  = (1<<13);
			}
			break;

		case 2:
			if(status)
			{
				GPIOC -> BSRR = (1<<14);
			}
			else
			{
				GPIOC -> BRR  = (1<<14);
			}
			break;

		case 3:
			if(status)
			{
				GPIOC -> BSRR = (1<<15);
			}
			else
			{
				GPIOC -> BRR  = (1<<15);
			}
			break;

		case 4:
			if(status)
			{
				GPIOA -> BSRR = (1<<0);
			}
			else
			{
				GPIOA -> BRR  = (1<<0);
			}
			break;

		case 5:
			if(status)
			{
				GPIOA -> BSRR = (1<<1);
			}
			else
			{
				GPIOA -> BRR  = (1<<1);
			}
			break;

		case 26:	// PB3
			if(status)
			{
				GPIOB -> BSRR = (1<<3);
			}
			else
			{
				GPIOB -> BRR  = (1<<3);
			}
			break;

		case 27:	// PB4
			if(status)
			{
				GPIOB -> BSRR = (1<<4);
			}
			else
			{
				GPIOB -> BRR  = (1<<4);
			}
			break;

		case 28:	// PB5
			if(status)
			{
				GPIOB -> BSRR = (1<<5);
			}
			else
			{
				GPIOB -> BRR  = (1<<5);
			}
			break;

		case 29:	// PB6
			if(status)
			{
				GPIOB -> BSRR = (1<<6);
			}
			else
			{
				GPIOB -> BRR  = (1<<6);
			}
			break;

		case 30:	// PB7
			if(status)
			{
				GPIOB -> BSRR = (1<<7);
			}
			else
			{
				GPIOB -> BRR  = (1<<7);
			}
			break;

		case 31:	// PB8
			if(status)
			{
				GPIOB -> BSRR = (1<<8);
			}
			else
			{
				GPIOB -> BRR  = (1<<8);
			}
			break;

		case 32:	// PB9
			if(status)
			{
				GPIOB -> BSRR = (1<<9);
			}
			else
			{
				GPIOB -> BRR  = (1<<9);
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
uint8_t GPIO::gateRead(uint8_t pin, uint8_t reg)	// reg: read register input IDR (0) or output ODR (1)
{
	uint8_t status = 0;

	switch (pin)
	{
		case 1:
			if(reg)
				status = GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13);
			else
				status = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13);

//			status =  (uint8_t) (((GPIOC -> ODR) & (1 << 13)) >> 13);
			break;

		case 2:
			if(reg)
				status = GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_14);
			else
				status = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14);
			break;

		case 3:
			if(reg)
				status = GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_15);
			else
				status = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15);
			break;

		case 4:
			if(reg)
				status = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_0);
			else
				status = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
			break;
	}

	return status;
}

#endif /* HARDWARE_GPIO_H_ */
