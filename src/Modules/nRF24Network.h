/*
 * nRF24Network.h
 *
 *  Created on: 11 de fev de 2017
 *      Author: titi
 */

#ifndef NRF24NETWORK_H_
#define NRF24NETWORK_H_

#include "nRF24L01p.h"

class packet {
public:
	unsigned char _addrHost;
	unsigned char _addrDest;
	unsigned char *_data;
};

class nRF24Network : nRF24L01p {
public:
	uint8_t _addrHost;
	uint8_t _topology;
	uint8_t _addrDestination;
	uint8_t *_pktRcvd;

	struct nRF24pkt {
		unsigned char addrHost;
		unsigned char addrDest;
		unsigned char *data;

	};

	void beginNetwork(uint8_t addrHost, uint8_t topology);
	void pkt_send(uint8_t addrDest, uint8_t size, uint8_t *frame);
	uint8_t pkt_receive();
	void pkt_available();
	void ping();


private:
};



#endif /* NRF24NETWORK_H_ */
