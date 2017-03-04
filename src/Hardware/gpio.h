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

#endif /* HARDWARE_GPIO_H_ */
