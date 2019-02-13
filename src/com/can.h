#ifndef CAN_H
#define CAN_H
#include <stdint.h>
#include "../config.h"

#define  ECAN1_BUF_LENGTH 32
#define  ECAN1_BUF_TX_LENGTH 8
#define  ECAN1_MSG_LENGTH 8


typedef uint16_t ECAN1BUF [ECAN1_BUF_LENGTH][ECAN1_MSG_LENGTH];
extern ECAN1BUF  can_buf DMA_SPACE;

enum CanMode {
	can_normal_mode,
	can_idk1,
	can_idk2,
	can_idk3,
	can_config_mode
};

void can_set_mode(int mode);
void can_init(int can_id, int use_eid);
void can_init_pins(void);
uint16_t* can_get_packet();
uint16_t* can_get_free_tx_buffer();
int can_get_buffer_number(uint16_t* buf);
void can_set_rx_id(int32_t rx_id);
void can_send_tx_buffer(uint16_t* buf);
void can_write_rx_accept_filter(int n, int32_t identifier, uint16_t exide, uint16_t bufPnt,uint16_t maskSel);
void can_disable_rx_accept_filter(int n);
void can_write_rx_accept_mask(int m, int32_t identifier, uint16_t mide, uint16_t exide);
void can_write_tx_msg_buf_id(uint16_t *buf, int32_t txIdentifier, uint16_t eid, uint16_t remoteTransmit);
void can_write_tx_msg_buf_data(uint16_t *buf, uint16_t dataLength, uint16_t data1, uint16_t data2, uint16_t data3, uint16_t data4);
void can_write_tx_default_id(uint16_t* buf);
void can_set_default_id(int32_t txIdentifier, uint16_t eid, uint16_t remoteTransmit);

#endif
