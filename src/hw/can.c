#include "can.h"
int32_t default_tx_id;
int default_tx_eid;
int default_tx_remote_transmit;

void can_set_default_id(int32_t txIdentifier, uint16_t eid, uint16_t remoteTransmit) {
	default_tx_id = txIdentifier;
	default_tx_eid = eid;
	default_tx_remote_transmit = remoteTransmit;
}

void can_pkt_set_default_id(uint16_t* pkt) {
	can_pkt_set_id(pkt, default_tx_id, default_tx_eid, default_tx_remote_transmit);
}


void can_pkt_set_data(uint16_t* pkt, int8_t len, uint8_t* data) {
	uint8_t* b = can_pkt_get_data(pkt, &len);
	int i;
	for(i=0; i < len; i++) {
		b[i] = data[i];
	}
}
