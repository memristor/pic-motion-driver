/*
	chip spec:
		protocols implemented: CAN 1.2, CAN 2.0A, CAN 2.0B
		automatic response to remote transmission requests
		8 transmission buffers (8 bytes each)
		32 receive buffers (8 bytes each)
		16 filters
		3 masks
		* page 253
		
		* pins: C1Tx C1Rx
		
		CAN message specification: page 277
*/

#include "../can.h"
#include "../bootloader.h"

typedef uint16_t ECAN1BUF [ECAN1_BUF_LENGTH][ECAN1_MSG_LENGTH];
ECAN1BUF can_buf __attribute__((space(dma),aligned(32*16)));

enum CanMode {
	can_normal_mode,
	can_idk1,
	can_idk2,
	can_idk3,
	can_config_mode
};

void can_set_mode(int mode);

static void dma1init(void) {
	DMACS0=0;
	DMA0CON=0x2020;
	DMA0PAD=(int)&C1TXD;		/* ECAN 1 (C1TXD) */
	DMA0CNT=0x0007;	
	DMA0REQ=0x46; 	/* ECAN 1 Transmit */
	DMA0STA= __builtin_dmaoffset(can_buf);
	DMA0CONbits.CHEN=1;	
}


/* Dma Initialization for ECAN2 Reception */
static void dma2init(void){
	
	DMACS0=0;
	DMA2CON=0x0020;
	DMA2PAD=0x0440;	/* ECAN 1 (C1RXD) */
	DMA2CNT=0x0007;
	DMA2REQ=0x0022;	/* ECAN 1 Receive */
	DMA2STA= __builtin_dmaoffset(can_buf);	
	DMA2CONbits.CHEN=1;
}

/* CAN Baud Rate Configuration 		*/
#ifdef USE_FRCPLL
#define CHIP_FREQUENCY 30000000
#else
#define CHIP_FREQUENCY 32000000
#endif
#define BITRATE 500000
#define NTQ 	20		// 20 Time Quanta in a Bit Time
#define BRP_VAL		((CHIP_FREQUENCY/(2*NTQ*BITRATE))-1)

/*
 * some predefined configs
 * 	125 - 125kbps
 *  500 - 500kbps
 */
static void set_bitrate(int val) {
	if(val == 125) {
		/* Synchronization Jump Width set to 4 TQ */
		C1CFG1bits.SJW = 0x3;
		/* Baud Rate Prescaler */
		C1CFG1bits.BRP = (CHIP_FREQUENCY/(2*20*125000))-1;
		// C1CFG1bits.BRP = -1; // very low baud for testing
		/* Phase Segment 1 time is 8 TQ */
		C1CFG2bits.SEG1PH=0x7;
		/* Phase Segment 2 time is set to be programmable */
		C1CFG2bits.SEG2PHTS = 0x1;
		/* Phase Segment 2 time is 6 TQ */
		C1CFG2bits.SEG2PH = 0x5;
		/* Propagation Segment time is 5 TQ */
		C1CFG2bits.PRSEG = 0x4;
		/* Bus line is sampled three times at the sample point */
		C1CFG2bits.SAM = 0x1;
		
	} else if(val == 530) { // 500Kbps on 30Mhz clock but less TQ
		/* Synchronization Jump Width set to 4 TQ */
		const long chip_clock = CHIP_FREQUENCY;
		const long can_freq = 500000;
		#ifdef USE_FRCPLL
		const long TQ = 15;
		#else
		const long TQ = 16;
		#endif
		
		// 4 TQ
		C1CFG1bits.SJW = 0x3;
		
		/* Baud Rate Prescaler */
		// C1CFG1bits.BRP = 2-1;//(chip_clock/(2*TQ*can_freq))-1;
		C1CFG1bits.BRP = (chip_clock/(2*TQ*can_freq))-1;
		
		// C1CFG1bits.BRP = -1; // very low baud for testing

		/* Phase Segment 1 time is 8 TQ */
		C1CFG2bits.SEG1PH=0x6;
		
		/* Phase Segment 2 time is set to be programmable */
		C1CFG2bits.SEG2PHTS = 0x1;
		
		/* Phase Segment 2 time is 3 TQ */
		C1CFG2bits.SEG2PH = 0x2;
		
		/* Propagation Segment time is 8 TQ */
		#ifdef USE_FRCPLL
		C1CFG2bits.PRSEG = 0x3;
		#else
		C1CFG2bits.PRSEG = 0x4;
		#endif
		
		/* Bus line is sampled three times at the sample point */
		C1CFG2bits.SAM = 0x1;
	}
}

static void can_clk_init(void){

	/* FCAN is selected to be FCY */
	/* FCAN = FCY = 40MHz */
	// C1CTRL1bits.CANCKS = 0x1;

	/*
	Bit Time = (Sync Segment + Propagation Delay + Phase Segment 1 + Phase Segment 2)=20*TQ
	Phase Segment 1 = 8TQ
	Phase Segment 2 = 6Tq
	Propagation Delay = 5Tq
	Sync Segment = 1TQ
	CiCFG1<BRP> =(FCAN /(2 ×N×FBAUD))
	*/

	set_bitrate(530);
	// set_bitrate(1252);
	// set_bitrate(125);
}

uint16_t* BOOT can_get_packet() {
	if(C1RXFUL1 == 0 && C1RXFUL2 == 0) {
		return 0;
	}
	
	int nbuf = C1FIFObits.FNRB;
	if(nbuf < 16) {
		C1RXFUL1 &= ~(1 << nbuf);
	} else {
		C1RXFUL2 &= ~(1 << (nbuf-16));
	}

	return can_buf[nbuf];
}

static int8_t is_buffer_free(int p) {
	uint16_t* r = (uint16_t*)&C1TR01CON + (p/2);
	if((p&1 == 0) && (*r & 0x0008) == 0) {
		return 1;
	} else if((p&1 == 1) && (*r & 0x0800) == 0) {
		return 1;
	}
}

uint16_t* can_get_free_tx_buffer() {
	uint16_t* r = (uint16_t*)&C1TR01CON;
	static int last_pkt = 0;
	int i=0;
	/*
	for(i=0; i < 4; i++, r++) {
		int p = 0;
		if((*r & 0x0008) == 0) {
			p = i*2;
		} else if((*r & 0x0800) == 0) {
			p = i*2+1;
		}
		return can_buf[p];
	}
	*/
	for(i=last_pkt+1; i < 8; i++) {
		if(is_buffer_free(i)) {
			last_pkt = i;
			return can_buf[i];
		}
	}
	for(i=0; i < last_pkt; i++) {
		if(is_buffer_free(i)) {
			last_pkt = i;
			return can_buf[i];
		}
	}
	return 0;
}

// int can_get_buffer_number(unsigned int* buf) {
	// int diff = buf-(uint16_t*)can_buf[0];
	// if(diff < 0 || diff >= ECAN1_MSG_LENGTH*ECAN1_BUF_TX_LENGTH) return -1;
	// int nbuf = (int)(diff >> 3);
	// return nbuf;
// }

void can_send_tx_buffer(unsigned int* buf) {
	int diff = buf-(uint16_t*)can_buf[0];
	if(diff < 0 || diff >= ECAN1_MSG_LENGTH*ECAN1_BUF_TX_LENGTH) return;
	int nbuf = (int)(diff >> 3);
	// unsigned int* b = can_buf[nbuf];
	// if((b[2] & 0xf) == 0) return;
	unsigned int* r = (unsigned int*)&C1TR01CON;
	r += (nbuf >> 1);
	*r |= (0x8 << ( (nbuf & 1) << 3 ));
}

/*
 * n - which filter {0,15}
 * identifier - identifier to match with
 * exide - {0,1} - use extended id?
 * bufPnt - {0..15}
 * maskSel - {0,1,2}
 */
void can_write_rx_accept_filter(int n, long identifier, unsigned int exide, unsigned int bufPnt,unsigned int maskSel) {

	unsigned long sid10_0=0, eid15_0=0, eid17_16=0;
	unsigned int *sidRegAddr,*bufPntRegAddr,*maskSelRegAddr, *fltEnRegAddr;

	C1CTRL1bits.WIN=1;

	// Obtain the Address of CiRXFnSID, CiBUFPNTn, CiFMSKSELn and CiFEN register for a given filter number "n"
	sidRegAddr = (unsigned int *)(&C1RXF0SID + (n * 2));
	bufPntRegAddr = (unsigned int *)(&C1BUFPNT1 + (n / 4));
	maskSelRegAddr = (unsigned int *)(&C1FMSKSEL1 + (n / 8));
	fltEnRegAddr = (unsigned int *)(&C1FEN1);

	// Bit-filed manupulation to write to Filter identifier register
	if(exide==1) { 	// Filter Extended Identifier
		eid15_0 = (identifier & 0xFFFF);
		eid17_16= (identifier>>16) & 0x3;
		sid10_0 = (identifier>>18) & 0x7FF;

		*sidRegAddr=(((sid10_0)<<5) + 0x8) + eid17_16;	// Write to CiRXFnSID Register
	    *(sidRegAddr+1)= eid15_0;					// Write to CiRXFnEID Register

	} else {			// Filter Standard Identifier
		sid10_0 = (identifier & 0x7FF);			
		*sidRegAddr=(sid10_0)<<5;					// Write to CiRXFnSID Register
		*(sidRegAddr+1)=0;							// Write to CiRXFnEID Register
	}

   *bufPntRegAddr = (*bufPntRegAddr) & (0xFFFF ^ (0xF << (4 * (n & 3)))); // clear nibble
   *bufPntRegAddr = ((bufPnt << (4 * (n & 3))) | (*bufPntRegAddr));       // Write to C1BUFPNTn Register

   *maskSelRegAddr = (*maskSelRegAddr) & (0xFFFF ^ (0x3 << ((n & 7) * 2))); // clear 2 bits
   *maskSelRegAddr = ((maskSel << (2 * (n & 7))) | (*maskSelRegAddr));      // Write to C1FMSKSELn Register

   *fltEnRegAddr = ((0x1 << n) | (*fltEnRegAddr)); // Write to C1FEN1 Register

   C1CTRL1bits.WIN=0;
}

void can_disable_rx_accept_filter(int n) {
	unsigned int *fltEnRegAddr;
   C1CTRL1bits.WIN=1;
   fltEnRegAddr = (unsigned int *)(&C1FEN1);
   *fltEnRegAddr = (*fltEnRegAddr) & (0xFFFF - (0x1 << n));
   C1CTRL1bits.WIN=0;

}

/*
 * m - {0,1,2} which mask to edit
 * identifier - mask of identifier
 * mide - {0,1} mask of exide bit
 * exide - {0,1} use extended id for identifier when writing the mask
 */
void can_write_rx_accept_mask(int m, long identifier, unsigned int mide, unsigned int exide) {
	unsigned long sid10_0=0, eid15_0=0, eid17_16=0;
	unsigned int *maskRegAddr;

	C1CTRL1bits.WIN=1;

	// Obtain the Address of CiRXMmSID register for given Mask number "m"
	maskRegAddr = (unsigned int *)(&C1RXM0SID + (m * 2));

	// Bit-filed manupulation to write to Filter Mask register
	if(exide==1) {
		// Filter Extended Identifier
		eid15_0 = (identifier & 0xFFFF);
		eid17_16= (identifier>>16) & 0x3;
		sid10_0 = (identifier>>18) & 0x7FF;

		*maskRegAddr = ((sid10_0)<<5) +  + eid17_16; // Write to CiRXMnSID Register
		
		if(mide==1) {
			*maskRegAddr |= 0x0008;
		}
			
	    *(maskRegAddr+1) = eid15_0;	// Write to CiRXMnEID Register

	} else {
		// Filter Standard Identifier
		sid10_0 = (identifier & 0x7FF);			
		*maskRegAddr=((sid10_0)<<5); // Write to CiRXMnSID Register
			
		if(mide==1) {
			*maskRegAddr |= 0x0008;
		}
		
		*(maskRegAddr+1) = 0; // Write to CiRXMnEID Register
	}

	C1CTRL1bits.WIN=0;	
}

#define C1TX_pin 0b10000
#define C1RX_pin 5

void can_set_rx_id(long rx_id) {
	can_write_rx_accept_filter(0, rx_id, 1, 15, 0);
	can_write_rx_accept_mask(0, 0x1FFFFFFF, 1, 1);
}

void can_init_pins(void) {
	RPOR3bits.RP6R = C1TX_pin; // choose pin for tx
	RPINR26bits.C1RXR = C1RX_pin; // choose pin for rx
}


void can_set_mode(int mode) {
	C1CTRL1bits.REQOP=mode;
	while(C1CTRL1bits.OPMODE!=mode);
}

void can_init(int can_id, int use_eid) {
	// pin config, pages 170-194
	dma1init();
	dma2init();
	
	// Enter Config Mode
	can_set_mode(can_config_mode);


	C1CTRL1bits.CANCKS = 0;
	// init CAN clock
	can_clk_init();	

	// FIFO Starts at Message Buffer 20
	C1FCTRLbits.FSA=20;
	
	// 32 CAN Message Buffers in DMA RAM
	C1FCTRLbits.DMABS=0b110;
	
	C1FEN1 = 0;
	
	can_set_default_id(can_id, use_eid, 0);
	can_set_rx_id(can_id);
	
	// Enter Normal Mode
	can_set_mode(can_normal_mode);
	
	// ECAN transmit/receive message control
	C1RXFUL1=C1RXFUL2=C1RXOVF1=C1RXOVF2=0x0000;
	
	// enable all buffers
	C1TR01CONbits.TXEN0=1;
	C1TR01CONbits.TXEN1=1;
	C1TR23CONbits.TXEN2=1;
	C1TR23CONbits.TXEN3=1;
	C1TR45CONbits.TXEN4=1;
	C1TR45CONbits.TXEN5=1;
	C1TR67CONbits.TXEN6=1;
	C1TR67CONbits.TXEN7=1;

	// set priorities (same priorities)
	C1TR01CONbits.TX0PRI=0b11;
	C1TR01CONbits.TX1PRI=0b11;
	C1TR23CONbits.TX2PRI=0b11;
	C1TR23CONbits.TX3PRI=0b11;
	C1TR45CONbits.TX4PRI=0b11;
	C1TR45CONbits.TX5PRI=0b11;
	C1TR67CONbits.TX6PRI=0b11;
	C1TR67CONbits.TX7PRI=0b11;
	
	// enable interrupts
	IEC2bits.C1IE = 1;
	C1INTEbits.TBIE = 1;	
	C1INTEbits.RBIE = 1;
}



/* ECAN Transmit Message Buffer Configuration

Inputs:
buf	-> Transmit Buffer Number

txIdentifier ->	


Extended Identifier (29-bits) : 0b000f ffff ffff ffff ffff ffff ffff ffff
								     |____________|_____________________|
									        SID10:0           EID17:0


Standard Identifier (11-bits) : 0b0000 0000 0000 0000 0000 0fff ffff ffff
														    |___________|
															      SID10:0

Standard Message Format: 
											Word0 : 0b000f ffff ffff ffff
													     |____________|||___
 									        				SID10:0   SRR   IDE     

											Word1 : 0b0000 0000 0000 0000
														   |____________|
															  EID17:6

											Word2 : 0b0000 00f0 0000 ffff
													  |_____||	  	 |__|
													  EID5:0 RTR   	  DLC
										
																  
																	
Extended Message Format: 
											Word0 : 0b000f ffff ffff ffff
													     |____________|||___
 									        				SID10:0   SRR   IDE     

											Word1 : 0b0000 ffff ffff ffff
														   |____________|
															  EID17:6

											Word2 : 0bffff fff0 0000 ffff
													  |_____||	  	 |__|
													  EID5:0 RTR   	  DLC

ide -> "0"  Message will transmit standard identifier
	   "1"  Message will transmit extended identifier



remoteTransmit -> "0" Message transmitted is a normal message
				  "1" Message transmitted is a remote message

				Standard Message Format: 
											Word0 : 0b000f ffff ffff ff1f
													     |____________|||___
 									        				SID10:0   SRR   IDE     

											Word1 : 0b0000 0000 0000 0000
														   |____________|
															  EID17:6

											Word2 : 0b0000 0010 0000 ffff
													  |_____||	  	 |__|
													  EID5:0 RTR   	  DLC
										
																  
																	
				Extended Message Format: 
											Word0 : 0b000f ffff ffff ff1f
													     |____________|||___
 									        				SID10:0   SRR   IDE     

											Word1 : 0b0000 ffff ffff ffff
														   |____________|
															  EID17:6

											Word2 : 0bffff ff10 0000 ffff
													  |_____||	  	 |__|
													  EID5:0 RTR   	  DLC

*/

void can_pkt_set_id(uint16_t* buf, int32_t txIdentifier, uint16_t eid, uint16_t remoteTransmit) {

	unsigned long word0=0, word1=0, word2=0;
	unsigned long sid10_0=0, eid5_0=0, eid17_6=0;

	if(eid) {
		eid5_0  = (txIdentifier & 0x3F);
		eid17_6 = (txIdentifier>>6) & 0xFFF;
		sid10_0 = (txIdentifier>>18) & 0x7FF;
		word1 = eid17_6;
	} else {
		sid10_0 = (txIdentifier & 0x7FF);
	}
	
	if(remoteTransmit==1) { 	// Transmit Remote Frame
		word0 = ((sid10_0 << 2) | eid | 0x2);
		word2 = ((eid5_0 << 10)| 0x0200);
	} else {
		word0 = ((sid10_0 << 2) | eid);
		word2 = (eid5_0 << 10);
	}
			
	// Obtain the Address of Transmit Buffer in DMA RAM for a given Transmit Buffer number

	if(eid)
		buf[0] = (word0 | 0x0002);
	else
		buf[0] = word0;

	buf[1] = word1;
	buf[2] = word2;
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



void __attribute__((interrupt, no_auto_psv))_C1Interrupt(void)  
{    
	IFS2bits.C1IF = 0; // clear interrupt flag

	if(C1INTFbits.TBIF) {
		C1INTFbits.TBIF = 0;
    }

    if(C1INTFbits.RBIF) {
		// check which buffer has received data
		C1INTFbits.RBIF = 0;
	}
}

