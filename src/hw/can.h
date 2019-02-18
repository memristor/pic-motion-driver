#ifndef CAN_H
#define CAN_H
#include <stdint.h>
#include "../config.h"

#define  ECAN1_BUF_LENGTH 32
#define  ECAN1_BUF_TX_LENGTH 8
#define  ECAN1_MSG_LENGTH 8



void can_init(int can_id, int use_eid);
void can_init_pins(void);

uint16_t* can_get_packet();
uint16_t* can_get_free_tx_buffer();

void can_set_rx_id(int32_t rx_id);
void can_send_tx_buffer(uint16_t* buf);


void can_write_rx_accept_filter(int n, int32_t identifier, uint16_t exide, uint16_t bufPnt,uint16_t maskSel);
void can_write_rx_accept_mask(int m, int32_t identifier, uint16_t mide, uint16_t exide);
void can_disable_rx_accept_filter(int n);

// get data pointer and length (or set length if dlen != 0)
uint8_t* can_pkt_get_data(uint16_t* pkt, int8_t* dlen); 
void can_pkt_set_default_id(uint16_t* pkt);
void can_pkt_set_id(uint16_t *pkt, int32_t txIdentifier, uint16_t eid, uint16_t remoteTransmit);


// void can_write_tx_msg_buf_data(uint16_t *buf, uint16_t dataLength, uint16_t data1, uint16_t data2, uint16_t data3, uint16_t data4);

// hw independent
void can_pkt_set_data(uint16_t* pkt, int8_t len, uint8_t* data);
void can_write_tx_default_id(uint16_t* buf);

void can_set_default_id(int32_t txIdentifier, uint16_t eid, uint16_t remoteTransmit);

#endif
