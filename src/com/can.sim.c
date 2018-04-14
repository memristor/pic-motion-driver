#include "can.h"
#include <stdint.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <fcntl.h>

/** Returns true on success, or false if there was an error */
int SetSocketBlockingEnabled(int fd, int blocking)
{
   if (fd < 0) return 0;

// #ifdef _WIN32
   // unsigned long mode = blocking ? 0 : 1;
   // return (ioctlsocket(fd, FIONBIO, &mode) == 0) ? 1 : 0;
// #else


   int flags = fcntl(fd, F_GETFL, 0);
   if (flags == -1) return 0;
   flags = blocking ? (flags & ~O_NONBLOCK) : (flags | O_NONBLOCK);
   return (fcntl(fd, F_SETFL, flags) == 0) ? 1 : 0;
// #endif
}

ECAN1BUF can_buf;
int sock;

long default_tx_id;
int default_tx_eid;
int default_tx_remote_transmit;

void can_init(int can_id, int use_eid) {
	/*
	sudo modprobe vcan
	sudo ip link add dev vcan0 type vcan
	sudo ip link set up vcan0
	*/
	int nbytes;
	struct sockaddr_can addr;
	struct can_frame frame;
	struct ifreq ifr;

	default_tx_id = can_id;
	default_tx_eid = use_eid;
	
	const char *ifname = "vcan0";

	if((sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Error while opening socket");
		return;
	}

	strcpy(ifr.ifr_name, ifname);
	ioctl(sock, SIOCGIFINDEX, &ifr);
	SetSocketBlockingEnabled(sock, 0);
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	printf("%s at index %d\n", ifname, ifr.ifr_ifindex);

	if(bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("Error in socket bind");
		return;
	}
	
	printf("can initialized\n");
}


int can_get_buffer_number(uint16_t* buf) {
}

void can_write_rx_accept_filter(int n, long identifier, unsigned int exide, unsigned int bufPnt,unsigned int maskSel) {
}

void can_disable_rx_accept_filter(int n) {
}

void can_write_rx_accept_mask(int m, long identifier, unsigned int mide, unsigned int exide) {
}

void can_set_rx_id(long rx_id) {
}

void can_init_pins(void) {
}

void can_set_mode(int mode) {
}


void can_write_tx_msg_buf_id(unsigned int* buf, long txIdentifier, unsigned int eid, unsigned int remoteTransmit) {
}


void can_set_default_id(long txIdentifier, unsigned int eid, unsigned int remoteTransmit) {
	
}

void can_write_tx_default_id(uint16_t* buf) {
	
}


uint16_t* can_get_packet() {
	uint16_t* pkt = (uint16_t*)can_buf[0];
	struct can_frame frame;
	// printf("reading pkt\n");
	int r = read(sock, &frame, sizeof(struct can_frame));
	if(r != sizeof(struct can_frame)) {
		// printf("fail to read pkt\n");
		return 0;
	}
	if(frame.can_id != (default_tx_id | (default_tx_eid ? CAN_EFF_FLAG : 0))) {
		printf("wrong can id: %x %x\n", (uint32_t)frame.can_id);
		return 0;
	}
	int8_t *msg = (int8_t*)&pkt[3];
	memcpy(msg, frame.data, 8);
	pkt[2] = frame.can_dlc;
	return pkt;
}

uint16_t* can_get_free_tx_buffer() {
	// printf("get free tx buffer\n");
	return (uint16_t*)can_buf[0];
}

void can_send_tx_buffer(uint16_t* pkt) {
	// printf("can_send_tx_buffer\n");
	int8_t *msg = (int8_t*)&pkt[3];
	struct can_frame frame;
	frame.can_dlc = pkt[2];
	frame.can_id = default_tx_id;
	frame.can_id |= default_tx_eid ? CAN_EFF_FLAG : 0;
	memcpy(frame.data, msg, 8);
	int nbytes = write(sock, &frame, sizeof(struct can_frame));
}

/* ECAN Transmit Data

	Inputs :
	buf -> Transmit Buffer Number

	dataLength -> {0..7} Length of Data in Bytes to be transmitted

	data1/data2/data3/data4 ->  Transmit Data Bytes 
*/
void can_write_tx_msg_buf_data(unsigned int* buf, unsigned int dataLength, unsigned int data1, unsigned int data2, unsigned int data3, unsigned int data4) {
	buf[2] = ((buf[2] & 0xFFF0) + dataLength);
	buf[3] = data1;
	buf[4] = data2;
	buf[5] = data3;
	buf[6] = data4;
}

