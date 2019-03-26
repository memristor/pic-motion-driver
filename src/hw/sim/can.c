#include "../can.h"
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

extern char* can_iface;

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

typedef uint16_t ECAN1BUF [ECAN1_BUF_LENGTH][ECAN1_MSG_LENGTH];
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
	
	const char *ifname = can_iface;

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


void can_write_rx_accept_filter(int n, int32_t identifier, uint16_t exide, uint16_t bufPnt,uint16_t maskSel){}
void can_write_rx_accept_mask(int m, int32_t identifier, uint16_t mide, uint16_t exide){}
void can_disable_rx_accept_filter(int n){}
void can_init_pins(void) {}
void can_pkt_set_id(uint16_t* buf, int32_t txIdentifier, uint16_t eid, uint16_t remoteTransmit) {}


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
		//printf("wrong can id: %x\n", (uint32_t)frame.can_id);
		return 0;
	}
	int8_t *msg = (int8_t*)&pkt[3];
	memcpy(msg, frame.data, 8);
	pkt[2] = frame.can_dlc;
	return pkt;
}

uint8_t* can_pkt_get_data(uint16_t* pkt, int8_t* dlen) {
	if (dlen) {
		if(*dlen != 0) {
			pkt[2] = (pkt[2] & 0xFFF0) + (*dlen & 0xf);
		} else {
			*dlen = pkt[2] & 0xf;
		}
	}
	return (uint8_t*)&pkt[3];
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
	// printf("sending: %c %c\n", frame.data[0], frame.data[1]);
	int nbytes = write(sock, &frame, sizeof(struct can_frame));
}
