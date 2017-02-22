///*
// * nRF24L01p.cpp
// *
// *  Created on: 29 de jul de 2016
// *      Author: titi
// */
//
//#include "nRF24L01p.h"
//
//void nRF24::CE(FunctionalState NewState)
//{
//	assert_param(IS_FUNCTIONAL_STATE(NewState));
//	if (NewState != DISABLE)
//	{
//		GPIO_SetBits(nRF24_PORT, nRF24_Pin_CE);
//		_delay_us(130);
//	}
//	else
//	{
//		GPIO_ResetBits(nRF24_PORT, nRF24_Pin_CE);
//		_delay_us(10);
//	}
//}
//void nRF24::CSN(FunctionalState NewState)
//{
//	assert_param(IS_FUNCTIONAL_STATE(NewState));
//	if (NewState != DISABLE)
//	{
//		//_delay_us(3);
//		GPIO_SetBits(nRF24_PORT, nRF24_Pin_CSN);
//	}
//	else
//	{
//		GPIO_ResetBits(nRF24_PORT, nRF24_Pin_CSN);
//		//_delay_us(3);
//	}
//}
//void nRF24::Configure()
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
////	RCC_AHBPeriphClockCmd(RCC_AHBPeriph, ENABLE);
//
//	/* Configure pins:  CE and CSN				*/
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//
//	/* Configure pins:  IRQ						*/
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//
//	Init_Master();
//}
//uint8_t nRF24::read_register(uint8_t reg)
//{
//	uint8_t nRF24_rx;
//
//	CSN(DISABLE);
//	transfer_Byte(R_REGISTER | ( REGISTER_MASK & reg ) );
//	nRF24_rx = transfer_Byte(NOP);
//	CSN(ENABLE);
//
//	return nRF24_rx;
//}
//uint8_t nRF24::read_register_buf(uint8_t reg, uint8_t* buf, uint8_t length)
//{
//	uint8_t status;
//	CSN(DISABLE);
//	status = transfer_Byte(R_REGISTER | ( REGISTER_MASK & reg ) );
//	while ( length-- ){
//		*buf++ = transfer_Byte(NOP);
//	}
//	CSN(ENABLE);
//
//	return status;
//}
//uint8_t nRF24::write_register(uint8_t reg, uint8_t value)
//{
//	uint8_t status;
//	CSN(DISABLE);
//	status = transfer_Byte(W_REGISTER | ( REGISTER_MASK & reg ) );
//	transfer_Byte(value);
//	CSN(ENABLE);
//
//	return status;
//}
//uint8_t nRF24::write_register_buf(uint8_t reg, uint8_t* buf, uint8_t length)
//{
//	uint8_t status;
//	CSN(DISABLE);
//	status = transfer_Byte(W_REGISTER | ( REGISTER_MASK & reg ) );
//	while(length--){
//		transfer_Byte(*buf++);
//	}
//	CSN(ENABLE);
//
//	return status;
//}
//void nRF24::nRF24_PWR(FunctionalState state)
//{
//	uint8_t regValue;
//
//	regValue = read_register(NRF_CONFIG);
//	if(state == ENABLE)
//	{
//		regValue |=  (1 << PWR_UP);
//	}
//	else
//	{
//		regValue &= ~(1 << PWR_UP);
//	}
//	write_register(NRF_CONFIG, regValue);
//
//	//_delay_ms(5);
//}
//uint8_t nRF24::get_pipe_payload_len(uint8_t pipe)
//{
//	uint8_t length, addr;
//	addr = RX_PW_P0 + pipe;
//
//	// read payload size
//	length = read_register(addr);
//
//	return length;
//}
//void nRF24::set_RF_PWR(uint8_t RF_power)
//{
//	uint8_t regValue;
//	regValue = read_register(RF_SETUP);
//	switch(RF_power)
//	{
//		case RF_PWR_18dBm:
//			regValue &= ~(1 << RF_PWR_high);
//			regValue &= ~(1 << RF_PWR_low);
//			break;
//
//		case RF_PWR_12dBm:
//			regValue &= ~(1 << RF_PWR_high);
//			regValue |=  (1 << RF_PWR_low);
//			break;
//
//		case RF_PWR_6dBm:
//			regValue |=  (1 << RF_PWR_high);
//			regValue &= ~(1 << RF_PWR_low);
//			break;
//
//		case RF_PWR_0dBm:
//			regValue |=  (1 << RF_PWR_high);
//			regValue |=  (1 << RF_PWR_low);
//			break;
//	}
//	regValue = write_register(RF_SETUP, regValue);
//}
//void nRF24::set_RX_mode(FunctionalState state)
//{
//	if(state == ENABLE)
//	{
//		uint8_t regValue;
//
//		// Set PRIM_RX to high;
//		regValue = read_register(NRF_CONFIG);
//
////		sprintf(buffer,"aftmode:%d",regValue);
////		USART1_println(buffer);
//
//		regValue |= (1 << PRIM_RX);
//		write_register(NRF_CONFIG, regValue);
//
//		// Just clear these bits
//		write_register(NRF_STATUS, (1 << RX_DR) | (1<<TX_DS) | (1<<MAX_RT));
//
//		// Set CE to high and wait 130 us;
//		CE(ENABLE);
////		//_delay_us(130);
//
////		sprintf(buffer,"rxmodeON:%d",regValue);
////		USART1_println(buffer);
//
//		stateMode = 2;
//	}
//	else
//	{
//		CE(DISABLE);
////		//_delay_us(130);
////		sprintf(buffer,"rxmodeOFF");
////		USART1_println(buffer);
//
//		stateMode = 1;
//	}
//}
//void nRF24::set_TX_mode(FunctionalState state)
//{
//	if(state == ENABLE)
//	{
//		uint8_t regValue;
//
//		// Set PRIM_RX to low (TX mode);
//		regValue = read_register(NRF_CONFIG);
//		regValue &= ~(1 << PRIM_RX);
//		write_register(NRF_CONFIG, regValue);
//
////		sprintf(buffer,"CONFIG: %d", regValue);
////		USART1_println(buffer);
////		USART1_println("sent!");
//		// Set CE to high and wait 130 us;
//		CE(ENABLE);
////		//_delay_us(130);
//
//		stateMode = 3;
//	}
//	else
//	{
//		CE(DISABLE);
//		//_delay_us(130);
//
//		stateMode = 1;
//	}
//
//}
//void nRF24::set_RX_ADDRn(uint8_t pipe, uint8_t *addr_rx, uint8_t addr_length)
//{
//	uint8_t regAddr = RX_ADDR_P0 + pipe;
//	int i;
//
//	CSN(DISABLE);
//	transfer_Byte(W_REGISTER | ( REGISTER_MASK & regAddr ) );
//	for(i=0; i<addr_length; i++)
//	{
//		transfer_Byte(addr_rx[i]);
//	}
//	CSN(ENABLE);
//
//#ifdef debug
//	for(i=0; i<addr_length; i++)
//	{
//		sprintf(buffer, "%d ", addr_rx[i]);
//		USART1_print(buffer);
//	}
//	USART1_println("");
//#endif
//}
//void nRF24::get_RX_ADDRn(uint8_t pipe, uint8_t *addr_rx, uint8_t addr_length)
//{
//	uint8_t regAddr = RX_ADDR_P0 + pipe;
//	int i;
//
//	CSN(DISABLE);
//	transfer_Byte(R_REGISTER | ( REGISTER_MASK & regAddr) );
//	for(i=0; i<addr_length; i++)
//	{
//		addr_rx[i] = transfer_Byte(NOP);
//	}
//	CSN(ENABLE);
//
//	for(i=0; i<addr_length; i++)
//	{
////		sprintf(buffer, "%d ", addr_rx[i]);
////		USART1_print(buffer);
//	}
////	USART1_println("");
//}
//void nRF24::set_TX_ADDR(uint8_t *addr, uint8_t addr_length)
//{
//	uint8_t regAddr = TX_ADDR;
//	int i;
//
//	CSN(DISABLE);
//	transfer_Byte(W_REGISTER | ( REGISTER_MASK & regAddr ) );
//	for(i=0; i<addr_length; i++)
//	{
//		transfer_Byte(addr[i]);
//	}
//	CSN(ENABLE);
//}
//void nRF24::get_TX_ADDR(uint8_t *addr_tx, uint8_t addr_length)
//{
//	CSN(DISABLE);
//	transfer_Byte(R_REGISTER | ( REGISTER_MASK & TX_ADDR) );
//	int i;
//	for(i=0; i<addr_length; i++)
//	{
////		nRF24_tx_addr[i] = transfer_Byte(NOP);
//		addr_tx[i] = transfer_Byte(NOP);
//	}
//	CSN(ENABLE);
//
//	for(i=0; i<addr_length; i++)
//	{
////		sprintf(buffer, "%d ", addr_tx[i]);
////		USART1_print(buffer);
//	}
////	USART1_println("");
//}
//uint8_t nRF24::write_payload(uint8_t *vect, uint8_t length)
//{
//	uint8_t status;
//	int i;
//	CSN(DISABLE);
//	status = transfer_Byte(W_TX_PAYLOAD);
//	for(i=0; i<length; i++)
//	{
//		transfer_Byte(vect[i]);
//	}
//	CSN(ENABLE);
//
//	return status;
//}
//uint8_t nRF24::read_payload(uint8_t *vect_rx, uint8_t length)
//{
//	int i, status;
//
//	CSN(DISABLE);
//	status = transfer_Byte(R_RX_PAYLOAD);
//
//	for(i=0; i<length; i++)
//	{
//		vect_rx[i] = transfer_Byte(NOP);
//	}
//	CSN(ENABLE);
//
//	return status;
//}
//void nRF24::flush_RX(void)
//{
//	CSN(DISABLE);
//	transfer_Byte(FLUSH_RX);
//	CSN(ENABLE);
//}
//void nRF24::flush_TX(void)
//{
//	CSN(DISABLE);
//	transfer_Byte(FLUSH_TX);
//	CSN(ENABLE);
//}
//void nRF24::set_CRC(FunctionalState state)
//{
//	uint8_t regValue;
//	regValue = read_register(NRF_CONFIG);
//	if(state == ENABLE)
//	{
//		regValue |=  (1 << EN_CRC);
//	}
//	else
//	{
//		regValue &= ~(1 << EN_CRC);
//	}
//	regValue = write_register(NRF_CONFIG, regValue);
//}
//void nRF24::set_250kbps()
//{
//	uint8_t regValue;
//	regValue  =  read_register(RF_SETUP);
//	regValue |=  (1<<RF_DR_LOW);
//	regValue &= ~(1<<RF_DR_HIGH);
//	regValue &= ~(1<<0);	// Dont care bit
//	regValue = write_register(RF_SETUP, regValue);
//}
//void nRF24::set_1Mbps()
//{
//	uint8_t regValue;
//	regValue = read_register(RF_SETUP);
//	regValue = (regValue & 0b11010111);	// Reset bit 5; Reset bit 3;
//	regValue = write_register(RF_SETUP, regValue);
//}
//void nRF24::set_2Mbps()
//{
//	uint8_t regValue;
//	regValue = read_register(RF_SETUP);
//	regValue = (regValue & 0b11011111) | 0b00001000;	// Reset bit 5; Set bit 3;
//	regValue = write_register(RF_SETUP, regValue);
//}
//int nRF24::get_RPD()
//{
//	return read_register(RPD);
//}
//void nRF24::set_pipe_state(uint8_t pipe, FunctionalState state)
//{
//	uint8_t regValue;
//
//	// Configure data pipe n
//	regValue = read_register(EN_RXADDR);
//	if(state == ENABLE)
//	{
//		regValue |=  (1 << pipe);
//	}
//	else
//	{
//		regValue &= ~(1 << pipe);
//	}
//	write_register(EN_RXADDR, regValue);
//}
//void nRF24::set_PowerUp()
//{
//	uint8_t regValue;
//	regValue = read_register(NRF_CONFIG);
//	regValue |= (1 << PWR_UP);
//	regValue = write_register(NRF_CONFIG, regValue);
//
//	stateMode = 1;
//
//	//_delay_ms(5);
//}
//void nRF24::set_PowerDown()
//{
//	CE(DISABLE);
//
//	uint8_t regValue;
//	regValue = read_register(NRF_CONFIG);
//	regValue &= ~(1 << PWR_UP);
//	regValue = write_register(NRF_CONFIG, regValue);
//
//	stateMode = 0;
//}
//void nRF24::set_ACK(uint8_t pipe, FunctionalState state)
//{
//	uint8_t regValue;
//	regValue = read_register(EN_AA);
//
//	if(state == ENABLE)
//	{
//		regValue |=  (1 << pipe);
//	}
//	else
//	{
//		regValue &= ~(1 << pipe);
//	}
//	regValue = write_register(EN_AA, regValue);
//}
//void nRF24::set_Auto_Retransmit(uint8_t wait_time, uint8_t arc)
//{
//	uint8_t regValue;
//	regValue = read_register(SETUP_RETR);
//	regValue |= ((wait_time << ARD) | (arc << ARC));
//	regValue = write_register(SETUP_RETR, regValue);
//}
//void nRF24::set_payload_width(uint8_t pipe, uint8_t length)
//{
//	uint8_t regAddr = RX_PW_P0 + pipe;
//	write_register(regAddr, length);
//}
//void nRF24::set_ACK_disable()
//{
//	// Disable all auto acknowledge (default 0x3F)
//	write_register(EN_AA, 0x00);
//	// Disable all auto retransmit (default 0x03)
//	write_register(SETUP_RETR, 0x00);
//}
//uint8_t nRF24::get_payload_width(uint8_t pipe)
//{
//	uint8_t regAddr = RX_PW_P0 + pipe, length;
//	length = read_register(regAddr);
//
//	return length;
//}
//void nRF24::fill_payload_TX(uint8_t *payload_TX, uint8_t length)
//{
//	write_register_buf(W_TX_PAYLOAD, payload_TX, length);
//}
//int nRF24::IRQ()
//{
//	//	if(!flag_nRF24_IRQ)
//	if(!GPIO_ReadInputDataBit(nRF24_PORT, nRF24_Pin_IRQ))
//	{
//		flag_nRF24_IRQ = 1;
//		return 1;
//	}
//	else
//	{
//		flag_nRF24_IRQ = 0;
//		return 0;
//	}
//}
//uint8_t nRF24::poll_RX()
//{
//	uint8_t regValue;//, pipe = 7;
//
////	//_delay_ms(100);
//
//	regValue = read_register(NRF_STATUS);
//	if(regValue & (1<<RX_DR))
//	{
////		USART1_println("GOT1!");
//		write_register(NRF_STATUS, (regValue | (1<<RX_DR)));
//	}
//
//
//	regValue = read_register(FIFO_STATUS);
////	pipe = ((regValue >> RX_P_NO) & 0x07);
////	sprintf(buffer,"%d", regValue);
////	USART1_println(buffer);
////	if(regValue & (1 << RX_DR))
//	if(!(regValue & (1 << RX_EMPTY)))
//	{
////		USART1_println("GOT2!");
//		return 1;
////		USART1_println("Got!");
////		if(pipe < 6)
////		{
////			return pipe;
////		}
////		else
////			return 0;
//	}
//	else
//	{
//		return 0;
//	}
//}
//void nRF24::openReadingPipe(void)//(uint8_t pipe, const uint8_t *addr)
//{
//
//}
//void nRF24::reset(void)
//{
//	write_register(NRF_CONFIG, 0x08);
////	write_register(EN_AA, 0x3F);
//
//	write_register(RF_SETUP, 0x0E);
//}
//void nRF24::Init()
//{
//	Init_Master();
//	Configure();
//	CSN(ENABLE);
//	CE(DISABLE);
//	set_PowerUp();
//
//	set_CRC(DISABLE);
//	set_ACK_disable();
//	set_250kbps();
//	set_RF_PWR(RF_PWR_0dBm);
//	set_payload_width(0, 2);
//	set_payload_width(1, 2);
//	set_RX_ADDRn(0, nRF24_rx_addr_p0_d, 5);	// Set rx address;
//	set_TX_ADDR(nRF24_rx_addr_p0_d, 5);		// Set destination address;
//	set_pipe_state(0, ENABLE);
//	set_pipe_state(1, DISABLE);
//
////	uint8_t len = get_payload_width(0);
////	sprintf(buffer,"len0=%d", len);
////	USART1_println(buffer);
//
////	flush_RX();								// flush rx;
////	set_RX_mode(ENABLE);						// Set PRIM bit
//}
//void nRF24::test()
//{
//	// Bit 1: PWR_UP =1
//	write_register(NRF_CONFIG, 0x02);
//	// Channel 5
//	write_register(RF_CH, 0x00);
//	// Bit 7: CONT_WAVE = 1; Bit 4: PLL_LOCK = 1
//	write_register(RF_SETUP, 0x9F);
//
//	CE(ENABLE);
//}
//void nRF24::summary()
//{
//	uint8_t regValue;
//
//	regValue = read_register(NRF_CONFIG);
////	sprintf(buffer,"PWR_UP: %d", (regValue & (1<<PWR_UP)));
////	USART1_println(buffer);
//}
//void nRF24::send_2bytes()
//{
//	/* Transmitter */
//
//	// Write bytes to FIFO and it will define the payload width.
//	write_payload(vect, 2);
//	if(vect[1] < 0x5b)
//	{
//		vect[0]++;
//		vect[1]++;
//	}
//	else
//	{
//		vect[0] = 0x41;
//		vect[1] = 0x42;
//	}
//
//	set_RX_mode(DISABLE);
//	set_TX_mode(ENABLE);
//
////			while((~nRF24_read_register(NRF_STATUS)) & (1 << TX_DS))
////			{
////				USART1_println("FU");
////			}
////
////			uint8_t regValue = nRF24_read_register(NRF_STATUS);
////			regValue |= (1 << TX_DS);
////			write_register(NRF_STATUS, regValue);
////
////			flush_TX();								// flush tx;
////			while((~nRF24_read_register(FIFO_STATUS)) & (1 << TX_EMPTY))
////			{
////				USART1_println("WAIT!");
////			}
////	sprintf(buffer,"sent:%c %c", vect[0], vect[1]);
////	USART1_println(buffer);
//	set_TX_mode(DISABLE);
////			//_delay_ms(10);
//	set_RX_mode(ENABLE);
//}