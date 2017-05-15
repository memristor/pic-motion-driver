#ifndef CAN_H
#define CAN_H


#define  ECAN1_BUF_LENGTH 32
#define  ECAN1_BUF_TX_LENGTH 8
#define  ECAN1_MSG_LENGTH 8
typedef unsigned int ECAN1BUF [ECAN1_BUF_LENGTH][ECAN1_MSG_LENGTH];
extern ECAN1BUF  can_buf __attribute__((space(dma)));

enum CanMode {
	can_normal_mode,
	can_idk1,
	can_idk2,
	can_idk3,
	can_config_mode
};

void can_set_mode(int mode);
void can_init(void);
void can_init_pins(void);
unsigned int* can_get_packet();
unsigned int* can_get_free_tx_buffer();
int can_get_buffer_number(unsigned int* buf);
void can_set_rx_id(long rx_id);
void can_send_tx_buffer(unsigned int* buf);
void can_write_rx_accept_filter(int n, long identifier, unsigned int exide, unsigned int bufPnt,unsigned int maskSel);
void can_disable_rx_accept_filter(int n);
void can_write_rx_accept_mask(int m, long identifier, unsigned int mide, unsigned int exide);
void can_write_tx_msg_buf_id(unsigned int *buf, long txIdentifier, unsigned int eid, unsigned int remoteTransmit);
void can_write_tx_msg_buf_data(unsigned int *buf, unsigned int dataLength, unsigned int data1, unsigned int data2, unsigned int data3, unsigned int data4);
void can_write_tx_default_id(unsigned int* buf);
void can_set_default_id(long txIdentifier, unsigned int eid, unsigned int remoteTransmit);

#endif
