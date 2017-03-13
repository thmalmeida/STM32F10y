/*
    Copyright (c) 2007 Stefan Engelke <mbox@stefanengelke.de>
	Portions Copyright (C) 2011 Greg Copeland

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without
    restriction, including without limitation the rights to use, copy,
    modify, merge, publish, distribute, sublicense, and/or sell copies
    of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
    DEALINGS IN THE SOFTWARE.
*/

#include "../Hardware/spi.h"

/* Memory Map */
#define NRF_CONFIG	0x00	// Configuration register
#define EN_AA		0x01	// Enable Auto Acknowledgment Function Disable...
#define EN_RXADDR	0x02	// Enable RX addresses
#define SETUP_AW	0x03	// Setup of Address Widths (common for all data pipes)
#define SETUP_RETR	0x04	// Setup of automatic retransmittion
#define RF_CH		0x05	// RF Channel
#define RF_SETUP	0x06	// RF Setup register
#define NRF_STATUS	0x07	// Status register
#define OBSERVE_TX	0x08	// Transmit observe register
#define CD			0x09	// RPD
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD	    0x1C
#define FEATURE	    0x1D

/* Bit Mnemonics */
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0
#define AW          0
#define ARD         4
#define ARC         0
#define PLL_LOCK    4
#define RF_DR       3
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0
#define DPL_P5	    5
#define DPL_P4	    4
#define DPL_P3	    3
#define DPL_P2	    2
#define DPL_P1	    1
#define DPL_P0	    0
#define EN_DPL	    2
#define EN_ACK_PAY  1
#define EN_DYN_ACK  0

#define RF_PWR		  	6
#define RF_PWR_18dBm 	0	// 7.0 mA
#define RF_PWR_12dBm 	1	// 7.5 mA
#define RF_PWR_6dBm 	2	// 9.0 mA
#define RF_PWR_0dBm 	3	// 11.3mA
#define RF_PWR_high		2
#define RF_PWR_low		1

/* Auto Retransmit Mnemonics */
#define AR_WAIT_250		0x00
#define AR_WAIT_500		0x01
#define AR_WAIT_750		0x02
#define AR_WAIT_1000	0x02
#define AR_WAIT_1250	0x03
#define AR_WAIT_1500	0x04
#define AR_WAIT_1750	0x05
#define AR_WAIT_2000	0x06
#define ARC_disable		0x00
#define ARC_1x			0x01

/* Instruction Mnemonics */
#define REGISTER_MASK		0x1F

#define R_REGISTER			0x00
#define W_REGISTER			0x20
#define ACTIVATE			0x50
#define R_RX_PL_WID			0x60
#define R_RX_PAYLOAD		0x61
#define W_TX_PAYLOAD  		0xA0
#define W_ACK_PAYLOAD 		0xA8
#define W_TX_PAYLOAD_NO_ACK	0xB0
#define FLUSH_TX			0xE1
#define FLUSH_RX			0xE2
#define REUSE_TX_PL			0xE3
#define NOP					0xFF

/* Non-P omissions */
#define LNA_HCURR   0

/* P model memory Map */
#define RPD         0x09
#define W_TX_PAYLOAD_NO_ACK  0xB0

/* P model bit Mnemonics */
#define RF_DR_LOW   5
#define RF_DR_HIGH  3
#define RF_PWR_LOW  1
#define RF_PWR_HIGH 2

// nRF24L01 Defines
#define nRF24_PORT		GPIOB
#define nRF24_Pin_CE	GPIO_Pin_0
#define nRF24_Pin_CSN	GPIO_Pin_1
#define nRF24_Pin_IRQ	GPIO_Pin_10

class nRF24L01p : SPI {
public:

	uint8_t address[5] = { 0xCC,0xCE,0xCC,0xCE,0xCC };
	uint8_t flag_nRF24_IRQ = 0;
	typedef enum states0 {
		powerDown,
		standByI,
		modeRX,
		modeTX,
		standByII
	}stateMachine;
	/*
	 * 0 = power down
	 * 1 = power up
	 * 2 = rx mode
	 * 3 = tx mode
	 */
	uint8_t stateMode = 0;
	uint8_t nRF24_rx_addr_p0[5];
	uint8_t nRF24_rx_addr_p0_d[5] =	{0xB3, 0xB4, 0xB5, 0xB6, 0x05};
	//uint8_t nRF24_tx_addr[5];
	//uint8_t nRF24_tx_addr_d[5] = 	{0x01, 0x02, 0x03, 0x04, 0x05};
	uint8_t vect[2] = {0x41, 0x42};
	uint8_t nRF24_flag_send_cont =0;

	void begin_nRF24L01p();
	uint8_t read_register(uint8_t reg);
	uint8_t read_register_buf(uint8_t reg, uint8_t* buf, uint8_t length);
	uint8_t write_register(uint8_t reg, uint8_t value);
	uint8_t write_register_buf(uint8_t reg, uint8_t* buf, uint8_t length);
	void nRF24_PWR(FunctionalState state);
	uint8_t get_pipe_payload_len(uint8_t pipe);
	void set_RF_PWR(uint8_t RF_power);
	void set_mode_rx(FunctionalState state);
	void set_mode_tx(FunctionalState state);
	void set_RX_ADDRn(uint8_t pipe, uint8_t *addr_rx, uint8_t addr_length);
	void get_RX_ADDRn(uint8_t pipe, uint8_t *addr_rx, uint8_t addr_length);
	void set_TX_ADDR(uint8_t *addr, uint8_t addr_length);
	void get_TX_ADDR(uint8_t *addr_tx, uint8_t addr_length);
	uint8_t write_payload(uint8_t *vect, uint8_t length);
	uint8_t read_payload(uint8_t *vect_rx, uint8_t length);
	void flush_RX(void);
	void flush_TX(void);
	void set_crc(FunctionalState state);
	void set_250kbps();
	void set_1Mbps();
	void set_2Mbps();
	int get_RPD();
	void set_state_pipe(uint8_t pipe, FunctionalState state);
	void set_PowerUp();
	void set_PowerDown();
	void set_ACK(uint8_t pipe, FunctionalState state);
	void set_Auto_Retransmit(uint8_t wait_time, uint8_t arc);
	void set_payload_width(uint8_t pipe, uint8_t length);
	void set_ACK_disable();
	uint8_t get_payload_width(uint8_t pipe);
	void fill_payload_TX(uint8_t *payload_TX, uint8_t length);
	int IRQ();
	uint8_t poll_RX();
	void openReadingPipe(void);//(uint8_t pipe, const uint8_t *addr)
	void reset(void);
	void sendTestSignal(void);
	void summary();
	void send_2bytes();

	void configurePins();
	void ce(FunctionalState NewState);
	void csn(FunctionalState NewState);

private:
};
