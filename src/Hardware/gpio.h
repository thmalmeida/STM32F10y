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
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	// Push pull
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	// open drain
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	}
	else
	{
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	// Pull-Up mode
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;	// Pull-Down mode
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

		case 6:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			break;

		case 7:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			break;

		case 8:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			break;

		case 9:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			break;

		case 10:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			break;

		case 11:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			break;

		case 12:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
			GPIO_Init(GPIOB, &GPIO_InitStructure);
			break;

		case 13:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
			GPIO_Init(GPIOB, &GPIO_InitStructure);
			break;

		case 14:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
			GPIO_Init(GPIOB, &GPIO_InitStructure);
			break;

		case 15:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
			GPIO_Init(GPIOB, &GPIO_InitStructure);
			break;

		case 16:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
			GPIO_Init(GPIOB, &GPIO_InitStructure);
			break;

		case 17:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
			GPIO_Init(GPIOB, &GPIO_InitStructure);
			break;

		case 18:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
			GPIO_Init(GPIOB, &GPIO_InitStructure);
			break;

		case 19:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
			GPIO_Init(GPIOB, &GPIO_InitStructure);
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

		case 6:
			if(status)
			{
				GPIOA -> BSRR = (1<<2);
			}
			else
			{
				GPIOA -> BRR  = (1<<2);
			}
			break;

		case 7:
			if(status)
			{
				GPIOA -> BSRR = (1<<3);
			}
			else
			{
				GPIOA -> BRR  = (1<<3);
			}
			break;

		case 8:
			if(status)
			{
				GPIOA -> BSRR = (1<<4);
			}
			else
			{
				GPIOA -> BRR  = (1<<4);
			}
			break;

		case 9:		// PA5 (SPI1_SCK)
			if(status)
			{
				GPIOA -> BSRR = (1<<5);
			}
			else
			{
				GPIOA -> BRR  = (1<<5);
			}
			break;

		case 10:	// PA6
			if(status)
			{
				GPIOA -> BSRR = (1<<6);
			}
			else
			{
				GPIOA -> BRR  = (1<<6);
			}
			break;

		case 11:	// PA7
			if(status)
			{
				GPIOA -> BSRR = (1<<7);
			}
			else
			{
				GPIOA -> BRR  = (1<<7);
			}
			break;

		case 12:	// PB0
			if(status)
			{
				GPIOB -> BSRR = (1<<0);
			}
			else
			{
				GPIOB -> BRR  = (1<<0);
			}
			break;

		case 13:	// PB1
			if(status)
			{
				GPIOB -> BSRR = (1<<1);
			}
			else
			{
				GPIOB -> BRR  = (1<<1);
			}
			break;

		case 14:	// PB10
			if(status)
			{
				GPIOB -> BSRR = (1<<10);
			}
			else
			{
				GPIOB -> BRR  = (1<<10);
			}
			break;

		case 15:	// PB11
			if(status)
			{
				GPIOB -> BSRR = (1<<11);
			}
			else
			{
				GPIOB -> BRR  = (1<<11);
			}
			break;

		case 16:	// PB12
			if(status)
			{
				GPIOB -> BSRR = (1<<12);
			}
			else
			{
				GPIOB -> BRR  = (1<<12);
			}
			break;

		case 17:	// PB13
			if(status)
			{
				GPIOB -> BSRR = (1<<13);
			}
			else
			{
				GPIOB -> BRR  = (1<<13);
			}
			break;

		case 18:	// PB14
			if(status)
			{
				GPIOB -> BSRR = (1<<14);
			}
			else
			{
				GPIOB -> BRR  = (1<<14);
			}
			break;

		case 19:	// PB15
			if(status)
			{
				GPIOB -> BSRR = (1<<15);
			}
			else
			{
				GPIOB -> BRR  = (1<<15);
			}
			break;

		case 25:	// PA15
			if(status)
			{
				GPIOA -> BSRR = (1<<15);
			}
			else
			{
				GPIOA -> BRR  = (1<<15);
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

		case 4:
			GPIOA -> ODR ^= (1<<0);
			break;

		case 5:
			GPIOA -> ODR ^= (1<<1);
			break;

		case 6:
			GPIOA -> ODR ^= (1<<2);
			break;

		case 7:
			GPIOA -> ODR ^= (1<<3);
			break;

		case 8:
			GPIOA -> ODR ^= (1<<4);
			break;

		case 9:
			GPIOA -> ODR ^= (1<<5);
			break;

		case 10:
			GPIOA -> ODR ^= (1<<6);
			break;

		case 11:
			GPIOA -> ODR ^= (1<<7);
			break;

		case 30:
			GPIOB -> ODR ^= (1<<7);
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

		case 5:
			if(reg)
				status = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_1);
			else
				status = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1);
			break;

		case 6:
			if(reg)
				status = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_2);
			else
				status = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2);
			break;

		case 7:
			if(reg)
				status = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_3);
			else
				status = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3);
			break;

		case 16:
			if(reg)
				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_12);
			else
				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12);
			break;

		case 17:
			if(reg)
				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_13);
			else
				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13);
			break;

		case 18:
			if(reg)
				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_14);
			else
				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14);
			break;

		case 19:
			if(reg)
				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_15);
			else
				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15);
			break;

		case 24:
			if(reg)
				status = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_12);
			else
				status = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12);
			break;

		case 25:
			if(reg)
				status = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_15);
			else
				status = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15);
			break;

		case 26:
			if(reg)
				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_3);
			else
				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3);
			break;

		case 27:
			if(reg)
				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_4);
			else
				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4);
			break;

		case 28:
			if(reg)
				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_5);
			else
				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5);
			break;

		case 29:
			if(reg)
				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_6);
			else
				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6);
			break;

		case 30:
			if(reg)
				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_7);
			else
				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7);
			break;

		case 31:
			if(reg)
				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_8);
			else
				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8);
			break;

		case 32:
			if(reg)
				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_9);
			else
				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9);
			break;
	}

	return status;
}

#endif /* HARDWARE_GPIO_H_ */
