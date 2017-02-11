#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define master
#define debug

#include <time.h>
#include <stm32f10x.h>
#include <stm32f10x_rtc.h>
#include "stm32f10x_it.h"
#include "eeprom.h"

#include "nRF24L01p.h"

#include "usart.h"

USART Serial;


TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;

__IO int flag_1s;

// Bluetooth str parse variables
uint8_t enableDecode = 0, opcode;
uint8_t k, rLength, j;
uint8_t j2 = 0;
uint8_t flag_frameStartBT = 0;
uint8_t enableTranslate_Bluetooth = 0;
char aux[3], aux2[4], buffer[60], inChar, sInstr[20];
char sInstrBluetooth[30];

unsigned char rxc;
unsigned char txcm = 0x4d;	// M
unsigned char txcs = 0x40;	// @
//	unsigned char txcs = 0x53;	// S


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//ErrorStatus  HSEStartUpStatus;
//FLASH_Status FlashStatus;
//uint16_t VarValue = 0;
///* Virtual address defined by the user: 0xFFFF value is prohibited */
//uint16_t VirtAddVarTab[NumbOfVar] = {0x0000, 0x0004, 0x0008};
//uint16_t addr = 0x000C;
//void demo01(void)	// Put this into the while(1) loop function
//{
//	SystemInit();
//	SystemCoreClockUpdate();
//
//	ADC1_Init();
//	IO_Init();
//	USART1_Init(9600); // initialize USART1 @ 9600 baud
//	SysTick_Init();
//
//	// ADC Variables
//	const uint16_t V25 = 1750;// when V25=1.41V at ref 3.3V
//	const uint16_t Avg_Slope = 5; //when avg_slope=4.3mV/C at ref 3.3V
//	uint16_t TemperatureC;
//	int ADC_Value=0;
//	int vd = 0, d1, d2;
//	double Voltage = 0.0, f1;
//	char buffer[50];
//
//	while(1)
//	{
//		LED_green(1);
//		ADC_Value = ADC_Read(ADC_Channel_16);
//	//		printf("\r\n ADC value: %d \r\n", AD_value);
//		TemperatureC = (uint16_t)((V25-ADC_Value)/Avg_Slope+25);
//	//		printf("Temperature: %d%cC\r\n", TemperatureC, 176);
//
//		vd = ADC_Read(ADC_Channel_17);
//		Voltage = (vd-1490.0+4096.0)/4096.0*3.3;
//
//	//		 V = 1.205*4095/ADCval
//
//		d1 = (int) Voltage;
//		f1 = Voltage - d1;
//		d2 = trunc(f1*1000);
//
//		sprintf(buffer,"D: %d, T: %d, V: %d.%04d\r\n",ADC_Value, TemperatureC, d1, d2);
//	//		USART_puts(USART1, "Fuck!\r\n"); // just send a message to indicate that it works
//
//		LED_green(0);
//		delay_ms(500);
//	}
//}
void comm_Bluetooth()
{
	// Rx - Always listening
//	uint8_t j2 =0;
	while((Serial.available()>0))	// Reading from serial
	{
//		Serial.sendByte('B');
		inChar = Serial.readByte();

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
//				Serial.sendByte(sInstr[i-1]);
//				Serial.write(sInstr[i-1]);
			}
			memset(sInstrBluetooth,0,sizeof(sInstrBluetooth));
	//		Serial.println(sInstr);
//			USART1_println(sInstr);

			enableDecode = 1;
		}
		else
		{
//			Serial.println("Err");
//			USART1_putc(pi0[0]);
//			USART1_putc(pf0[0]);
		}
	}
}
void handleMessage()
{
	if(enableDecode)
	{
		enableDecode = 0;

		// Getting the opcode
		aux[0] = sInstr[0];
		aux[1] = sInstr[1];
		aux[2] = '\0';
		opcode = (uint8_t) atoi(aux);

//		uint8_t reg_value, reg_addr, rxc;

		switch (opcode)
		{
			case 0:		// Set motor ON/OFF
			{
				if(sInstr[2] == ';')
				{
					LED_green_toogle();
					Serial.println("Fuck!");
				}
			}
		}
		memset(sInstr,0,sizeof(sInstr));	// Clear all vector;
	}
}
//void handleMessage()
//{
//	if(enableDecode)
//	{
//		enableDecode = 0;
//
//		// Getting the opcode
//		aux[0] = sInstr[0];
//		aux[1] = sInstr[1];
//		aux[2] = '\0';
//		opcode = (uint8_t) atoi(aux);
//
//		uint8_t reg_value, reg_addr, rxc;
//
//		if(sInstr[2] == ';')
//		{
//			aux[0] = sInstr[0];
//			aux[1] = sInstr[1];
//			aux[2] = '\0';
//			reg_addr  = (uint8_t) atoi(aux);
//
//			sprintf(buffer,"%d",reg_addr);
//			Serial.print(buffer);
//		}
//
//
////		if(sInstr[2] == ':' && sInstr[6] == ';')
////		{
////			aux[0] = sInstr[0];
////			aux[1] = sInstr[1];
////			aux[2] = '\0';
////			reg_addr  = (uint8_t) atoi(aux);
////
////
////			aux2[0] = sInstr[3];
////			aux2[1] = sInstr[4];
////			aux2[2] = sInstr[5];
////			aux2[3] = '\0';
////			reg_value  = (uint8_t) atoi(aux2);
////
////			rxc = nRF24_write_register(reg_addr, reg_value);
////			sprintf(buffer, "w:%d", rxc);
////			USART1_println(buffer);
////		}
////		else if(sInstr[0] == 'r' && sInstr[1] == ';')
////		{
////			nRF24_get_RX_ADDRn(0, nRF24_rx_addr_p0, 5);
////		}
////		else if(sInstr[0] == 'w' && sInstr[1] == ';')
////		{
////			nRF24_set_RX_ADDRn(0, nRF24_rx_addr_p0_d, 5);
////		}
////		else if(sInstr[0] == 'c' && sInstr[1] == ':' && sInstr[3] == ';')
////		{
////			if(sInstr[2] == '1')
////			{
////				nRF24_test();
////				USART1_println("test");
////			}
////			else
////			{
////				nRF24_reset();
////				USART1_println("RST");
////			}
////		}
////		else if(sInstr[0] == 't' && sInstr[1] == ';')
////		{
////			nRF24_send_2bytes();
////		}
////		else if(sInstr[0] == 't' && sInstr[1] == ':' && sInstr[2] == '1' && sInstr[3] == ';')
////		{
////			nRF24_flag_send_cont = 1;
////		}
////		else if(sInstr[0] == 't' && sInstr[1] == ':' && sInstr[2] == '0' && sInstr[3] == ';')
////		{
////			nRF24_flag_send_cont = 0;
////		}
////		else if(sInstr[0] == 'p' && sInstr[1] == 'u' && sInstr[2] == ';')
////		{
////			nRF24_set_PowerUp();
////		}
////		else if(sInstr[0] == 'p' && sInstr[1] == 'd' && sInstr[2] == ';')
////		{
////			USART1_println("PowerDown");
////			nRF24_set_PowerDown();
////		}
////		else if(sInstr[0] == 'f' && sInstr[1] == 't' && sInstr[2] == ';')
////		{
////			nRF24_flush_TX();
////		}
////		else if(sInstr[0] == 'f' && sInstr[1] == 'r' && sInstr[2] == ';')
////		{
////			nRF24_flush_RX();
////		}
////		else if(sInstr[0] == 'r' && sInstr[1] == 'x' && sInstr[2] == ':' && sInstr[4] == ';')
////		{
////			if(sInstr[3] == '1')
////			{
////				nRF24_set_RX_mode(ENABLE);
////			}
////			else
////			{
////				nRF24_set_RX_mode(DISABLE);
////			}
////		}
////		else if(sInstr[2] == ';')
////		{
////			aux[0] = sInstr[0];
////			aux[1] = sInstr[1];
////			aux[2] = '\0';
////
////			reg_addr  = (uint8_t) atoi(aux);
////			rxc = nRF24_read_register(reg_addr);
////			sprintf(buffer, "r:%d", rxc);
////			USART1_println(buffer);
////		}
//
//		switch (opcode)
//		{
////			case 0:
////			{
////				if(sInstr[1] == ';')
////				{
////					rxc = nRF24_read_register(reg_addr);
////					sprintf(buffer, "r:%d", rxc);
////					USART1_println(buffer);
////				}
////			}
////			break;
////			case 1:
////			{
////				if(sInstr[1] == ':' && sInstr[4] == ';')
////				{
////					aux[0] = sInstr[2];
////					aux[1] = sInstr[3];
////					aux[2] = '\0';
////
////					byte_value  = (uint8_t) atoi(aux);
////					rxc = nRF24_write_register(0x00, 0x00);
////					sprintf(buffer, "w1:%d", rxc);
////					USART1_println(buffer);
////				}
////			}
////			break;
////			case 2:
////			{
////				if(sInstr[1] == ':' && sInstr[4] == ';')
////				{
////					aux[0] = sInstr[2];
////					aux[1] = sInstr[3];
////					aux[2] = '\0';
////
////					byte_value  = (uint8_t) atoi(aux);
////					rxc = nRF24_write_register(0x00, byte_value);
////					sprintf(buffer, "w2:%d", rxc);
////					USART1_println(buffer);
////				}
////			}
////			break;
////			default:
////			{
////				USART1_println("fuckdefault!");
////			}
////			break;
//
//// -----------------------------------------------------------------
////			case 0:		// Set motor ON/OFF
////			{
////				if
//
////			}
//			case 30:		// Set motor ON/OFF
//			{
//				LED_green_toogle();
////				uint8_t motorCommand;
////				aux[0] = '0';
////				aux[1] = sInstr[1];
////				aux[2] = '\0';
////				motorCommand = (uint8_t) atoi(aux);
////
////				if(motorCommand)
////				{
////					LED_green(1);
////					USART1_println("ON");
////				}
////				else
////				{
////					LED_green(0);
////					USART1_println("OFF");
////				}
//
////				if (motorCommand && (!motorStatus))
////					motor_start();
////				else
////					motor_stop();
////
////				summary_Print(3);
//			}
//			break;
//
////			case 4: // EEPROM Write -> $4:h:1234;
////			{
////				if(sInstr[1]==':' && sInstr[3]==':' && sInstr[8]==';')
////				{
////					char aux2[5];
////					aux2[0] = sInstr[4];
////					aux2[1] = sInstr[5];
////					aux2[2] = sInstr[6];
////					aux2[3] = sInstr[7];
////					aux2[4] = '\0';
////					uint16_t valueReceived = (uint16_t) atoi(aux2);
////
////					aux[0] = '0';
////					aux[1] = sInstr[2];
////					aux[2] = '\0';
////
////					uint8_t ind = (uint8_t) atoi(aux);
////
////					EE_WriteVariable(VirtAddVarTab[ind], valueReceived);
////
////					/* Store 800 values of Variable3 in EEPROM */
//////					for (VarValue = 0; VarValue < 800; VarValue++)
//////					{
//////						EE_WriteVariable(VirtAddVarTab[2], VarValue);
//////					}
////
////					uint16_t valueStored;
////					EE_ReadVariable(VirtAddVarTab[ind], &valueStored);
////					sprintf(buffer,"%d", valueStored);
////					USART1_println(buffer);
////				}
////				else if(sInstr[1]==';')
////				{
////					uint16_t valueRead;
////					int i;
////					for(i=0; i<3; i++)
////					{
////						EE_ReadVariable(VirtAddVarTab[i], &valueRead);
////						sprintf(buffer,"%d: %d", VirtAddVarTab[i], valueRead);
////						USART1_println(buffer);
////					}
////
////					char name2[21];
////					uint16_t v;
////					for(i=0; i<20; i++)
////					{
////						EE_ReadVariable(addr, &v);
////						name2[i] = (char) v;
////					}
////					USART1_println(name2);
////					USART1_println("Fuck");
////				}
////
////				break;
////			}
//// -----------------------------------------------------------------
//		}
//		memset(sInstr,0,sizeof(sInstr));	// Clear all vector;
//	}
//}
//void comm_SPI1()
//{
//	char inChar2;
//	while((SPI1_available()>0))	// Reading from serial
//	{
//		USART1_println("i: ");
//		inChar2 = SPI1_readByte();
//		USART1_putc(inChar2);
//	}
//	if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) == SET)
//	{
//		USART1_println("B");
//		inChar2 = SPI1_readByte();
//		USART1_putc(inChar2);
//	}
//
//}
//void comm_SPI1_polling()
//{
//	unsigned char rxc, txc = 0x57;
//	if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) == SET)
//	{
//		rxc = SPI1 -> DR;
//		USART1_putc(rxc);
//	}
//	if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_TXE)  == SET)
//	{
//		SPI1 -> DR = txc;
//	}
//}
//void refresh_variables()
//{
////	nRF24_int();
//
//	if(SPI1_flag_err)
//	{
//		SPI1_flag_err = 0;
//		USART1_println("Err");
//		while(1);
//	}
//}
//int main(int argc, char* argv[])

int main(void)
{
	init();

	Serial.begin(9600);	// initialize USART1 @ 9600 baud

	while(1)
	{
		comm_Bluetooth();

		handleMessage();
	}

}


/*
 * Interruptions sequence
 */
extern "C" {
void USART1_IRQHandler(void)
{
	// check if the USART1 receive interrupt flag was set
	while(USART_GetITStatus(USART1, USART_IT_RXNE))
	{
//		GPIOC -> ODR ^= (1<<13);
		char t = (USART1->DR); // the character from the USART1 data register is saved in t

		if(Serial._USART1_cnt < MAX_STRLEN)
		{
			Serial._received_string[Serial._USART1_cnt] = t;
			Serial._USART1_cnt++;
		}
		else
		{
			memset(Serial._received_string,0,sizeof(Serial._received_string));
			Serial._USART1_cnt = 0;
		}
	}
}
}
/*
 * Interruptions sequence END
 */

void uart_demo1(void)
{
	char c = 'a';
	uint8_t flag01 =0;

	if(!flag01)
	{
		Serial.sendByte(c++);
		if(c == 'f')
		{
			flag01 = 1;
		}
	}
	else
	{
		Serial.sendByte(c--);
		if(c == 'a')
		{
			flag01 = 0;
		}
	}
	_delay_ms(500);
}


//void USART1_IRQHandler(void)
//{
//	// check if the USART1 receive interrupt flag was set
//	while(USART_GetITStatus(USART1, USART_IT_RXNE))
//	{
//		LED_green_toogle();
//		char t = USART1->DR; // the character from the USART1 data register is saved in t
//		if(Serial._USART1_cnt < MAX_STRLEN)
//		{
//			Serial._received_string[Serial._USART1_cnt] = t;
//			Serial._USART1_cnt++;
//		}
//		else
//		{
////			memset(Serial._received_string,0,sizeof(Serial._received_string));
//			Serial._USART1_cnt = 0;
//		}
//	}
//}
//int main(void)
//{
//	//	SystemCoreClockUpdate();
//	SystemInit(); 		// Setup STM32 system (clock, PLL and Flash configuration)
//
//	ADC1_Init();
//	IO_Init();
//	SysTick_Init();
//	USART1_Init(38400); // initialize USART1 @ 38400 baud
//
//#ifdef master
//	nRF24_Init();
//	USART1_println("Hi, Master!");
//#else
//	nRF24_Init();
//	USART1_println("Hi, Slave!");
//#endif
//
//	while (1)
//	{
//#ifdef master
//		comm_Bluetooth();
//		handleMessage();
//		refresh_variables();
//
//		if(stateMode == 2)
//		{
////				sprintf(buffer,"%2d", nRF24_get_RPD());
////				USART1_println(buffer);
////
////				sprintf(buffer,"%2d", nRF24_read_register(NRF_STATUS));
////				USART1_println(buffer);
////
////				sprintf(buffer,"%2d", nRF24_read_register(FIFO_STATUS));
////				USART1_println(buffer);
//
//			if(nRF24_IRQ())
//			{
//				USART1_println("IRQ Pin");
//
//				int pipeRX = nRF24_poll_RX();
//				if(pipeRX)
//				{
//		//			uint8_t payload_length = nRF24_get_pipe_payload_len(0);
//					uint8_t data[2];
//					nRF24_read_payload(data, 2);
////					nRF24_flush_RX();								// flush rx;
//					sprintf(buffer,"rx: %c, %c", data[0], data[1]);
//					USART1_println(buffer);
//				}
//			}
//		}
//
//		if(flag_1s)
//		{
//			flag_1s ^= flag_1s;
//
//			if(nRF24_flag_send_cont)
//			{
//				nRF24_send_2bytes();
//			}
//		}
//
//#else
////		nRF24_set_RX_mode(ENABLE);
//
//
//		// Transmitter;
////		nRF24_set_TX_mode(ENABLE);
//
//
//
////		SPI1_slave_demo01();
//#endif
//	}
//
//	return 0;
//
//
////	__DATE__
//
//	FLASH_Unlock();		// Unlock the Flash Program Erase controller
//	EE_Init();			// EEPROM Init
//
//
//
////	time_t     utcsec;
//	long utcsec;
////	struct tm  ts;
//	char       buf[80];
////	Get current time
////	time(&now);
//	utcsec = 1467296589;
////	Format time, "ddd yyyy-mm-dd hh:mm:ss zzz"
////	ts = *localtime(&now);
//	char *c_time_string;
//    c_time_string = ctime(&utcsec);
//    sprintf(buf, "Current time is %s", c_time_string);
////	sprintf(buf, "utcsec: %d", utcsec);
////    strcpy(buf, "Current time is ");
//	USART1_println(buf);
//	int i;
//	for(i=0;i<20;i++)
//	{
//		EE_WriteVariable(addr, name[i]);
//	}
//	/* --- Store successively many values of the three variables in the EEPROM ---*/
//	/* Store 1000 values of Variable1 in EEPROM */
//	for (VarValue = 0; VarValue < 1000; VarValue++)
//	{
//		EE_WriteVariable(VirtAddVarTab[0], VarValue);
//	}
//
//	/* Store 500 values of Variable2 in EEPROM */
//	for (VarValue = 0; VarValue < 500; VarValue++)
//	{
//		EE_WriteVariable(VirtAddVarTab[1], VarValue);
//	}
//
//	/* Store 800 values of Variable3 in EEPROM */
//	for (VarValue = 0; VarValue < 800; VarValue++)
//	{
//		EE_WriteVariable(VirtAddVarTab[2], VarValue);
//	}
// DEBOUNCE!!!!
//int button_is_pressed()
//{
//	/* the button is pressed when BUTTON_BIT is clear */
//	if (bit_is_clear(BUTTON_R_PIN, BUTTON_R_BIT))
//	{
//		_delay_ms(DEBOUNCE_TIME);
//		if (bit_is_clear(BUTTON_R_PIN, BUTTON_R_BIT))
//			return 1;
//	}
//	if (bit_is_clear(BUTTON_L_PIN, BUTTON_L_BIT))
//	{
//		_delay_ms(DEBOUNCE_TIME);
//		if (bit_is_clear(BUTTON_L_PIN, BUTTON_L_BIT))
//			return -1;
//	}
//	return 0;
//}
#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
