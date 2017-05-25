/*
 * nokia5110.h
 *
 *  Created on: 25 de mai de 2017
 *      Author: titi
 */

#ifndef NOKIA5110_NOKIA5110_H_
#define NOKIA5110_NOKIA5110_H_

static const uint8_t FontNew[] = {
	0x00, 0x00, 0x00, 0x00, 0x00,// (space)
	0x00, 0x00, 0x5F, 0x00, 0x00,// !
	0x00, 0x07, 0x00, 0x07, 0x00,// "
	0x14, 0x7F, 0x14, 0x7F, 0x14,// #
	0x24, 0x2A, 0x7F, 0x2A, 0x12,// $
	0x23, 0x13, 0x08, 0x64, 0x62,// %
	0x36, 0x49, 0x55, 0x22, 0x50,// &
	0x00, 0x05, 0x03, 0x00, 0x00,// '
	0x00, 0x1C, 0x22, 0x41, 0x00,// (
	0x00, 0x41, 0x22, 0x1C, 0x00,// )
	0x08, 0x2A, 0x1C, 0x2A, 0x08,// *
	0x08, 0x08, 0x3E, 0x08, 0x08,// +
	0x00, 0x50, 0x30, 0x00, 0x00,// ,
	0x08, 0x08, 0x08, 0x08, 0x08,// -
	0x00, 0x60, 0x60, 0x00, 0x00,// .
	0x20, 0x10, 0x08, 0x04, 0x02,// /
	0x3E, 0x51, 0x49, 0x45, 0x3E,// 0
	0x00, 0x42, 0x7F, 0x40, 0x00,// 1
	0x42, 0x61, 0x51, 0x49, 0x46,// 2
	0x21, 0x41, 0x45, 0x4B, 0x31,// 3
	0x18, 0x14, 0x12, 0x7F, 0x10,// 4
	0x27, 0x45, 0x45, 0x45, 0x39,// 5
	0x3C, 0x4A, 0x49, 0x49, 0x30,// 6
	0x01, 0x71, 0x09, 0x05, 0x03,// 7
	0x36, 0x49, 0x49, 0x49, 0x36,// 8
	0x06, 0x49, 0x49, 0x29, 0x1E,// 9
	0x00, 0x36, 0x36, 0x00, 0x00,// :
	0x00, 0x56, 0x36, 0x00, 0x00,// ;
	0x00, 0x08, 0x14, 0x22, 0x41,// <
	0x14, 0x14, 0x14, 0x14, 0x14,// =
	0x41, 0x22, 0x14, 0x08, 0x00,// >
	0x02, 0x01, 0x51, 0x09, 0x06,// ?
	0x32, 0x49, 0x79, 0x41, 0x3E,// @
	0x7E, 0x11, 0x11, 0x11, 0x7E,// A
	0x7F, 0x49, 0x49, 0x49, 0x36,// B
	0x3E, 0x41, 0x41, 0x41, 0x22,// C
	0x7F, 0x41, 0x41, 0x22, 0x1C,// D
	0x7F, 0x49, 0x49, 0x49, 0x41,// E
	0x7F, 0x09, 0x09, 0x01, 0x01,// F
	0x3E, 0x41, 0x41, 0x51, 0x32,// G
	0x7F, 0x08, 0x08, 0x08, 0x7F,// H
	0x00, 0x41, 0x7F, 0x41, 0x00,// I
	0x20, 0x40, 0x41, 0x3F, 0x01,// J
	0x7F, 0x08, 0x14, 0x22, 0x41,// K
	0x7F, 0x40, 0x40, 0x40, 0x40,// L
	0x7F, 0x02, 0x04, 0x02, 0x7F,// M
	0x7F, 0x04, 0x08, 0x10, 0x7F,// N
	0x3E, 0x41, 0x41, 0x41, 0x3E,// O
	0x7F, 0x09, 0x09, 0x09, 0x06,// P
	0x3E, 0x41, 0x51, 0x21, 0x5E,// Q
	0x7F, 0x09, 0x19, 0x29, 0x46,// R
	0x46, 0x49, 0x49, 0x49, 0x31,// S
	0x01, 0x01, 0x7F, 0x01, 0x01,// T
	0x3F, 0x40, 0x40, 0x40, 0x3F,// U
	0x1F, 0x20, 0x40, 0x20, 0x1F,// V
	0x7F, 0x20, 0x18, 0x20, 0x7F,// W
	0x63, 0x14, 0x08, 0x14, 0x63,// X
	0x03, 0x04, 0x78, 0x04, 0x03,// Y
	0x61, 0x51, 0x49, 0x45, 0x43,// Z
	0x00, 0x00, 0x7F, 0x41, 0x41,// [
	0x02, 0x04, 0x08, 0x10, 0x20,// "\"
	0x41, 0x41, 0x7F, 0x00, 0x00,// ]
	0x04, 0x02, 0x01, 0x02, 0x04,// ^
	0x40, 0x40, 0x40, 0x40, 0x40,// _
	0x00, 0x01, 0x02, 0x04, 0x00,// `
	0x20, 0x54, 0x54, 0x54, 0x78,// a
	0x7F, 0x48, 0x44, 0x44, 0x38,// b
	0x38, 0x44, 0x44, 0x44, 0x20,// c
	0x38, 0x44, 0x44, 0x48, 0x7F,// d
	0x38, 0x54, 0x54, 0x54, 0x18,// e
	0x08, 0x7E, 0x09, 0x01, 0x02,// f
	0x08, 0x14, 0x54, 0x54, 0x3C,// g
	0x7F, 0x08, 0x04, 0x04, 0x78,// h
	0x00, 0x44, 0x7D, 0x40, 0x00,// i
	0x20, 0x40, 0x44, 0x3D, 0x00,// j
	0x00, 0x7F, 0x10, 0x28, 0x44,// k
	0x00, 0x41, 0x7F, 0x40, 0x00,// l
	0x7C, 0x04, 0x18, 0x04, 0x78,// m
	0x7C, 0x08, 0x04, 0x04, 0x78,// n
	0x38, 0x44, 0x44, 0x44, 0x38,// o
	0x7C, 0x14, 0x14, 0x14, 0x08,// p
	0x08, 0x14, 0x14, 0x18, 0x7C,// q
	0x7C, 0x08, 0x04, 0x04, 0x08,// r
	0x48, 0x54, 0x54, 0x54, 0x20,// s
	0x04, 0x3F, 0x44, 0x40, 0x20,// t
	0x3C, 0x40, 0x40, 0x20, 0x7C,// u
	0x1C, 0x20, 0x40, 0x20, 0x1C,// v
	0x3C, 0x40, 0x30, 0x40, 0x3C,// w
	0x44, 0x28, 0x10, 0x28, 0x44,// x
	0x0C, 0x50, 0x50, 0x50, 0x3C,// y
	0x44, 0x64, 0x54, 0x4C, 0x44,// z
	0x00, 0x08, 0x36, 0x41, 0x00,// {
	0x00, 0x00, 0x7F, 0x00, 0x00,// |
	0x00, 0x41, 0x36, 0x08, 0x00,// }
	0x08, 0x08, 0x2A, 0x1C, 0x08,// ->
	0x08, 0x1C, 0x2A, 0x08, 0x08 // <-
};

#define LCD_PORT 						GPIOA

#define	LCD_PIN_LED						12	//GPIO_Pin_3
#define LCD_PIN_SCE						15	//GPIO_Pin_0
#define LCD_PIN_RESET					14	//GPIO_Pin_1
#define LCD_PIN_COMMAND					13	//GPIO_Pin_2
#define LCD_PIN_CLOCK					9	//GPIO_Pin_5
#define LCD_PIN_DATA					11	//GPIO_Pin_7

#define PCD8544_TIME_DELAY				20

#define BLACK 							1
#define WHITE 							0

#define LCDWIDTH						84
#define LCDHEIGHT						48

#define PCD8544_POWERDOWN				0x04
//#define PCD8544_POWER_DOWN            (1<<2)
#define PCD8544_ENTRYMODE				0x02
#define PCD8544_EXTENDEDINSTRUCTION		0x01

#define PCD8544_DISPLAYBLANK			0x0
#define PCD8544_DISPLAYNORMAL			0x4
#define PCD8544_DISPLAYALLON			0x1
#define PCD8544_DISPLAYINVERTED			0x5

// H = 0
#define PCD8544_FUNCTION_SET			0x20
#define PCD8544_POWER_DOWN				(1<<2)
//#define PCD8544_FUNCTION_SET			(1<<5)
#define PCD8544_DISPLAYCONTROL			0x08
#define PCD8544_SET_YADDR				0x40
#define PCD8544_SET_XADDR				0x80

// H = 1
#define PCD8544_SET_TEMP				0x04
#define PCD8544_SET_BIAS				0x10
#define PCD8544_SET_VOP					0x80

#define PCD8544_NOP						0

#define PCD8544_HORIZONTAL_ADDRESSING	0
#define PCD8544_VERTICAL_ADDRESSING  	(1<<1)
#define PCD8544_EXTENDED_INSTRUCTION	(1<<0)

#define PCD8544_SET_Y_ADDRESS			0x40
#define PCD8544_SET_X_ADDRESS			0x80

#define PCD8544_MAX_BANKS				6
#define PCD8544_MAX_COLS				84

// name Basic instruction set (H=0)
#define PCD8544_DISPLAY_CONTROL			(1<<3)
#define PCD8544_DISPLAY_BLANK			0x0
#define PCD8544_DISPLAY_NORMAL			(1<<2)
#define PCD8544_DISPLAY_ALL_ON			(1<<0)
#define PCD8544_DISPLAY_INVERTED		(1<<2|1<<0)
// name Extended instruction set (H=1)

//#define PCD8544_SET_TEMP (1<<2)
#define PCD8544_TEMPCO_0				0b00
#define PCD8544_TEMPCO_1				0b01
#define PCD8544_TEMPCO_2				0b10
#define PCD8544_TEMPCO_3				0b11

//#define PCD8544_SET_BIAS (1<<4)
//#define PCD8544_SET_VOP  (1<<7)

#define SPIy                   SPI1
#define SPIy_CLK               RCC_APB2Periph_SPI1
#define SPIy_GPIO              GPIOA
//#define SPIy_GPIO_CLK          RCC_APB2Periph_GPIOA
#define SPIy_PIN_SCK           GPIO_Pin_5
#define SPIy_PIN_MISO          GPIO_Pin_6
#define SPIy_PIN_MOSI          GPIO_Pin_7

/* Private define ------------------------------------------------------------*/
#define BufferSize 32
#define SEND_LIMIT 3


#include "../Hardware/spi.h"
#include "../Hardware/gpio.h"
#include "glcd.h"

class NOKIA5110 : SPI, GPIO {
public:
	void RCC_Configuration();
	void GPIO_Configuration();
	void SPI1_Configuration();

	void glcd_init();
	void glcd_init_pins();
	void glcd_data(uint8_t c);
	void glcd_write();
	void glcd_clear2();
	void glcd_set_x_address(uint8_t x);
	void glcd_set_y_address(uint8_t y);

	void glcd_command(uint8_t c);

	void glcd_put_char(char c);
	void glcd_put_string(uint8_t x, uint8_t y, char const *str);
	void glcd_reset();
	void glcd_print(void);
	void glcd_set_contrast(uint8_t val);
	void glcd_power_down(void);
	void glcd_power_up(void);
	void glcd_PCD8544_init(void);
	void glcd_example();

	void glcd_SPI1_debug();

protected:
	void LCD_ENABLE();
	void LCD_DISABLE();
	void LCD_RESET_ON();
	void LCD_RESET_OFF();
	void LCD_DC_COMMAND();
	void LCD_DC_DATA();
	void LCD_LED_OFF();
	void LCD_LED_ON();
};

void NOKIA5110::LCD_ENABLE()
{
//	GPIO_ResetBits(LCD_PORT, LCD_PIN_SCE);
	gateSet(LCD_PIN_SCE, 0);
}
void NOKIA5110::LCD_DISABLE()
{
//	GPIO_SetBits(LCD_PORT, LCD_PIN_SCE);
	gateSet(LCD_PIN_SCE, 1);
}
void NOKIA5110::LCD_RESET_ON()
{
//	GPIO_ResetBits(LCD_PORT, LCD_PIN_RESET);
	gateSet(LCD_PIN_RESET, 0);
}
void NOKIA5110::LCD_RESET_OFF()
{
//	GPIO_SetBits(LCD_PORT, LCD_PIN_RESET);
	gateSet(LCD_PIN_RESET, 1);
}
void NOKIA5110::LCD_DC_COMMAND()
{
//	GPIO_ResetBits(LCD_PORT, LCD_PIN_COMMAND);
	gateSet(LCD_PIN_COMMAND, 0);
}
void NOKIA5110::LCD_DC_DATA()
{
//	GPIO_SetBits(LCD_PORT, LCD_PIN_COMMAND);
	gateSet(LCD_PIN_COMMAND, 1);
}
void NOKIA5110::LCD_LED_OFF()
{
//	GPIO_ResetBits(LCD_PORT, LCD_PIN_LED);
	gateSet(LCD_PIN_LED, 0);
}
void NOKIA5110::LCD_LED_ON()
{
//	GPIO_SetBits(LCD_PORT, LCD_PIN_LED);
	gateSet(LCD_PIN_LED, 1);
}
void NOKIA5110::RCC_Configuration(void)
{
	/* PCLK2 = HCLK/2 */
	RCC_PCLK2Config(RCC_HCLK_Div2);
}
//void GPIO_Configuration()
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
//
//	/* Configure SPIy pins: SCK, MISO and MOSI ---------------------------------*/
//	GPIO_InitStructure.GPIO_Pin = SPIy_PIN_SCK | SPIy_PIN_MOSI | SPIy_PIN_MISO;
////	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5;
//	/* Configure SCK and MOSI pins as Alternate Function Push Pull */
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(LCD_PORT, &GPIO_InitStructure);
//
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_5); //SCK
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_5); //MISO
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_5); //MOSI
//
//	/*Configure GPIO pins*/
//	GPIO_InitStructure.GPIO_Pin = LCD_PIN_SCE | LCD_PIN_RESET | LCD_PIN_COMMAND | LCD_PIN_LED;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(LCD_PORT, &GPIO_InitStructure);
//
//
////	GPIOA ->
//}
//void writeByte(uint8_t B)
//{
//	SPI_SendData8(SPI1, B);
//}
void NOKIA5110::glcd_reset()
{
//	LCD_DISABLE();
	LCD_RESET_ON();
	_delay_us(PCD8544_TIME_DELAY);
	LCD_RESET_OFF();
	_delay_us(PCD8544_TIME_DELAY);
	LCD_ENABLE();
	_delay_us(PCD8544_TIME_DELAY);
}
void NOKIA5110::glcd_command(uint8_t c)
{
	LCD_RESET_OFF();
	LCD_ENABLE();
	LCD_DC_COMMAND();
	writeByte(c);
//	SPI_write(c);
	_delay_us(PCD8544_TIME_DELAY);
}
void NOKIA5110::glcd_data(uint8_t c)
{
	LCD_RESET_OFF();
	LCD_ENABLE();
	LCD_DC_DATA();
	_delay_us(PCD8544_TIME_DELAY);
	writeByte(c);
	_delay_us(PCD8544_TIME_DELAY);
}
void NOKIA5110::glcd_SPI1_debug()
{
	char c = 'A';
	while(1)
	{
		writeByte(c);
	}
}
void NOKIA5110::glcd_init()
{
//	SPI1_Configuration();
//	GPIO_Configuration();
	gateConfig(LCD_PIN_SCE, 1);
	gateConfig(LCD_PIN_RESET, 1);
	gateConfig(LCD_PIN_COMMAND, 1);
	gateConfig(LCD_PIN_DATA, 1);
	gateConfig(LCD_PIN_CLOCK, 1);
	gateConfig(LCD_PIN_LED, 1);

	set_SPI_to_Master(1);

	// --- Reset and Enable
	glcd_reset();
	LCD_LED_ON();

	// --- glcd intialization
	// get into the EXTENDED mode!
	glcd_command(PCD8544_FUNCTION_SET | PCD8544_EXTENDEDINSTRUCTION);

	// LCD bias select (4 is optimal?)
	glcd_command(PCD8544_SET_BIAS | 0x04);

	// set VOP
//	if (contrast > 0x7f)
//	contrast = 0x7f;

	glcd_command(PCD8544_SET_VOP | 40); // Experimentally determined

	// normal mode
	glcd_command(PCD8544_FUNCTION_SET);

	// Set display to Normal
	glcd_command(PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYNORMAL);

	glcd_clear2();

//	/* Initialization sequence of controller */
//	glcd_reset();
//
//	/* Get into the EXTENDED mode! */
//	glcd_command(PCD8544_FUNCTION_SET | PCD8544_EXTENDED_INSTRUCTION);
//
//	/* LCD bias select (4 is optimal?) */
//	glcd_command(PCD8544_SET_BIAS | 0x2);
//
//	/* Set VOP */
//	glcd_command(PCD8544_SET_VOP | 50); // Experimentally determined
//
//	/* Back to standard instructions */
//	glcd_command(PCD8544_FUNCTION_SET);
//
//	/* Normal mode */
//	glcd_command(PCD8544_DISPLAY_CONTROL | PCD8544_DISPLAY_NORMAL);
//
////	glcd_select_screen((uint8_t *)&glcd_buffer,&glcd_bbox);
//
////	glcd_set_contrast(50);
//
////	glcd_clear();
}
void NOKIA5110::glcd_print(void)
{
	glcd_data(0x5A);
	glcd_data(0x5A);
	glcd_data(0x5A);
	glcd_data(0x5A);
	glcd_data(0x5A);
}
void NOKIA5110::glcd_set_contrast(uint8_t val)
{
	glcd_command(PCD8544_FUNCTION_SET | PCD8544_EXTENDED_INSTRUCTION);
	glcd_command(PCD8544_SET_VOP | (val&0x7f));
	glcd_command(PCD8544_FUNCTION_SET);
	glcd_command(PCD8544_DISPLAY_CONTROL | PCD8544_DISPLAY_NORMAL);
}
void NOKIA5110::glcd_power_down(void)
{
	/* First, fill RAM with zeroes to ensure minimum specified current consumption */
	glcd_clear2();

	/* Power down */
	glcd_command(PCD8544_FUNCTION_SET|PCD8544_POWER_DOWN);
}
void NOKIA5110::glcd_power_up(void)
{
	glcd_command(PCD8544_FUNCTION_SET);
}
void NOKIA5110::glcd_set_y_address(uint8_t y)
{
	glcd_command(PCD8544_SET_Y_ADDRESS | (y > 5 ? 5 : y));
	_delay_us(PCD8544_TIME_DELAY);
}
void NOKIA5110::glcd_set_x_address(uint8_t x)
{
//	glcd_command(PCD8544_SET_X_ADDRESS | (x & 0x7f));
	glcd_command(0b10000000 | (x & 0b01111111));
	_delay_us(PCD8544_TIME_DELAY);
}
void NOKIA5110::glcd_clear2()
{
	int i;
	for(i=0; i<PCD8544_MAX_BANKS*PCD8544_MAX_COLS; i++)
	{
//		glcd_set_x_address(i);
//		delay_ms(1);
//		for(j=0; i<PCD8544_MAX_COLS; j++)
		{
//			glcd_set_y_address(j);
			glcd_data(0);
		}
	}
//	for (bank = 0; bank < PCD8544_MAX_BANKS; bank++) {
//		/* Each bank is a single row 8 bits tall */
//		uint8_t column;
//
//		if (glcd_bbox_selected->y_min >= (bank+1)*8) {
//			continue; /* Skip the entire bank */
//		}
//
//		if (glcd_bbox_selected->y_max < bank*8) {
//			break;    /* No more banks need updating */
//		}
//
//		glcd_command(PCD8544_SET_Y_ADDRESS | bank);
//		delay_ms(10);
//		glcd_command(PCD8544_SET_X_ADDRESS | glcd_bbox_selected->x_min);
//		delay_ms(10);
//		for (column = glcd_bbox_selected->x_min; column <= glcd_bbox_selected->x_max; column++)
//		{
//			glcd_data( glcd_buffer_selected[PCD8544_MAX_COLS * bank + column] );
//			delay_ms(10);
//		}
//	}
}
void NOKIA5110::glcd_write()
{
	uint8_t bank;

	for (bank = 0; bank < PCD8544_MAX_BANKS; bank++) {
		/* Each bank is a single row 8 bits tall */
		uint8_t column;

		if (glcd_bbox_selected->y_min >= (bank+1)*8) {
			continue; /* Skip the entire bank */
		}

		if (glcd_bbox_selected->y_max < bank*8) {
			break;    /* No more banks need updating */
		}

		glcd_command(PCD8544_SET_Y_ADDRESS | bank);
		glcd_command(PCD8544_SET_X_ADDRESS | glcd_bbox_selected->x_min);
		for (column = glcd_bbox_selected->x_min; column <= glcd_bbox_selected->x_max; column++)
		{
			glcd_data( glcd_buffer_selected[PCD8544_MAX_COLS * bank + column] );
			_delay_ms(10);
		}
	}

	glcd_reset_bbox();

}
void NOKIA5110::glcd_PCD8544_init(void) {

	glcd_reset();

	/* Get into the EXTENDED mode! */
	glcd_command(PCD8544_FUNCTION_SET | PCD8544_EXTENDED_INSTRUCTION);

	/* LCD bias select (4 is optimal?) */
	glcd_command(PCD8544_SET_BIAS | 0x2);

	/* Set VOP (affects contrast) */
	glcd_command(PCD8544_SET_VOP | 80); /* Experimentally determined, play with this figure until contrast looks nice */

	/* Back to standard instructions */
	glcd_command(PCD8544_FUNCTION_SET);

	/* Normal mode */
	glcd_command(PCD8544_DISPLAY_CONTROL | PCD8544_DISPLAY_NORMAL);
}
void NOKIA5110::glcd_put_char(char c)
{
	int i;
	glcd_data(0x00);
	for(i=0;i<5;i++)
	{
		glcd_data(FontNew[(c-32)*5+i]);
	}
}
void NOKIA5110::glcd_put_string(uint8_t x, uint8_t y, char const *str)
{
	glcd_set_x_address(x);
	glcd_set_y_address(y);
	int i, length = strlen(str);

	for(i=0;i<length;i++)
		glcd_put_char(str[i]);
}
void NOKIA5110::glcd_example()
{
	char *str1 = "EXAMPLE";
//	char *str1 = {};
//	strcpy(str1, "EXAMPLE");
//	char str2[9];
//	sprintf(str2,"EXAMPLE%d",0);
	glcd_command(PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYNORMAL);
	glcd_put_string(13, 3, str1);
}





#endif /* NOKIA5110_NOKIA5110_H_ */
