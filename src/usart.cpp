//#include "usart.h"
//
//USART::USART(uint32_t baudRate)
//{
//
//}
//
//void USART::send(uint8_t data)
//{
//
//}
//
//uint8_t USART::receive(void)
//{
//	switch(number)
//	{
//		case USART_0:
//			return receive0();
//		case USART_1:
//			return receive1();
//		case USART_2:
//			return receive2();
//		case USART_3:
//			return receive3();
//		default:
//		break;
//	}
//	return false;
//}
//
////void USART::sendStr(const char *str, char limit)
//void USART::send(const char *str, char limit)
//{
//	while(*str != limit){
//		//(this->*send)(*str);
//		send(*str);
//		str++;
//	}
//}
//
//void USART::send(volatile char *str, USART_TypeDef* USART)
//{
//	while(*str)
//	{
//		// wait until data register is empty. Transmission complete (TC) bit
//		while( !(USARTx->SR & 0x00000040) );
//		USART_SendData(USARTx, *str);
//		str++;
//	}
//}
//
//
//
//
////void USART1_puts0(USART_TypeDef* USARTx, volatile char *s)
////{
////	while(*s)
////	{
////		// wait until data register is empty. Transmission complete (TC) bit
////		while( !(USARTx->SR & 0x00000040) );
////		USART_SendData(USARTx, *s);
////		s++;
////	}
////}
