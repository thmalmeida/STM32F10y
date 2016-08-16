///*
// * usart.h
// *
// *  Created on: 25 de jun de 2016
// *      Author: titi
// */
//
//#ifndef USART_H_
//#define USART_H_
//
//#include <stm32f10x_usart.h>
//
//typedef enum USARTnumber{
//	USART_0 = 0,//!< Seleciona a porta USART 0
//	USART_1,    //!< Seleciona a porta USART 1
//	USART_2,    //!< Seleciona a porta USART 2
//	USART_3     //!< Seleciona a porta USART 3
//}USARTnumber;
//
//class USART
//{
//	public:
//		USART(uint32_t baudRate);
//
//		void send(uint8_t data);
//		void send(const char *str, char limit = '\0');
//		uint8_t receive(void);
//
//	private:
//		/**
//		* \brief Número da porta utilizada
//		*/
//		USARTnumber number;
//
//		/**
//		* \brief Envia um \a byte pela porta USART 0.
//		* @param [in] data Byte que será enviado.
//		*/
//		void send0(uint8_t data);
//		/**
//		* \brief Recebe um \a byte pela porta USART 0.
//		* @return Byte recebido.
//		*/
//		uint8_t receive0(void);
//
//		/**
//		* \brief Envia um \a byte pela porta USART 1.
//		* @param [in] data Byte que será enviado.
//		*/
//		void send1(uint8_t data);
//		/**
//		* \brief Recebe um \a byte pela porta USART 1.
//		* @return Byte recebido.
//		*/
//		uint8_t receive1(void);
//
//		/**
//		* \brief Envia um \a byte pela porta USART 2.
//		* @param [in] data Byte que será enviado.
//		*/
//		void send2(uint8_t data);
//		/**
//		* \brief Recebe um \a byte pela porta USART 2.
//		* @return Byte recebido.
//		*/
//		uint8_t receive2(void);
//
//		/**
//		* \brief Envia um \a byte pela porta USART 3.
//		* @param [in] data Byte que será enviado.
//		*/
//		void send3(uint8_t data);
//		/**
//		* \brief Recebe um \a byte pela porta USART 3.
//		* @return Byte recebido.
//		*/
//		uint8_t receive3(void);
//
//
//
//};
//
//#endif /* USART_H_ */
