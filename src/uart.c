#include "uart.h"
#include <p33FJ128MC802.h>
#include <libpic30.h>

static volatile uint8_t rx_buf[RX_BUF_LEN];
static volatile uint8_t rx_index1 = 0; // new data is put at this offset
static volatile uint8_t rx_index2 = 0; // data is read from this offset
static volatile uint8_t rxData;
static volatile uint8_t rxCounter = 0;

void UART_Init(long baud)
{
	U1BRG = (double)FCY / (16 * baud) - 1;
	U1MODEbits.STSEL = 0;   // 1 Stop bit
	U1MODEbits.PDSEL = 0;   // No Parity, 8 data bits
	U1MODEbits.BRGH = 0;    // Low-Speed mode
	//U1BRG = 31;             // BAUD Rate Setting for 57600

	U1STAbits.URXISEL = 0; //rx interrupt po prijemu jednog bajta
	//U1STAbits.UTXISEL = ;
	IFS0bits.U1RXIF = 0;

	_U1RXIE = 1;    		//Enable UART1 Rx interrupt
	IPC2bits.U1RXIP = 6;	//prioritet nije 7 zato sto se onda ne mogu iskljuciti prekidi
	IPC0bits.T1IP = 1; 		//T1 interrupt priority set to minimum

	rx_index1 = rx_index2 = rxCounter = 0;

	U1MODEbits.UARTEN = 1;  // Enable UART
	U1STAbits.UTXEN = 1;

	__delay_ms(100);
}

/**********************************************************************
 * Function Name     : BusyUART1
 * Description       : This returns status whether the transmission 
 is in progress or not, by checking Status bit TRMT
 * Parameters        : None
 * Return Value      : char info whether transmission is in progress 
 ***********************************************************************/

inline char BusyUART1(void)
{  
	return !U1STAbits.TRMT;
}

/*********************************************************************
 * Function Name     : CloseUART1
 * Description       : This function disables the UART and clears the 
 *                     Interrupt enable & flag bits
 * Parameters        : None
 * Return Value      : None
 *********************************************************************/

void CloseUART1(void)
{  
	U1MODEbits.UARTEN = 0;

	IEC0bits.U1RXIE = 0;
	IEC0bits.U1TXIE = 0;

	IFS0bits.U1RXIF = 0;
	IFS0bits.U1TXIF = 0;
}

/*********************************************************************
 * Function Name     : ConfigIntUART1
 * Description       : This function sets priority for RX,TX interrupt  
 *                     and enable/disables the interrupt
 * Parameters        : unsigned int config enable/disable and priority
 * Return Value      : None
 *********************************************************************/
void ConfigIntUART1(unsigned int config)
{
	/* clear IF flags */
	IFS0bits.U1RXIF = 0;
	IFS0bits.U1TXIF = 0;

	/* set priority */
	IPC2bits.U1RXIP = 0x0007 & config;
	IPC3bits.U1TXIP = (0x0070 & config) >> 4; //OVDE JE PRE BILO IPC2

	/* enable/disable interrupt */
	IEC0bits.U1RXIE = (0x0008 & config) >> 3;
	IEC0bits.U1TXIE = (0x0080 & config) >> 7;

	__delay_ms(100);
}

/*********************************************************************
 * Function Name     : DataRdyUart1
 * Description       : This function checks whether there is any data 
 *                     that can be read from the input buffer, by 
 *                     checking URXDA bit
 * Parameters        : None
 * Return Value      : char if any data available in buffer 
 *********************************************************************/
inline char DataRdyUART1(void)
{
	U1STAbits.OERR = 0;       //skrnavac usrani mora rucno da se resetuje
	return U1STAbits.URXDA;
}

/****************************************************************************
 * Function Name     : getsUART1
 * Description       : This function gets a string of data of specified length  
 *                     if available in the UxRXREG buffer into the buffer 
 *                     specified.
 * Parameters        : unsigned int length the length expected
 *                     unsigned int *buffer  the received data to be 
 *                                  recorded to this array
 *                     unsigned int uart_data_wait timeout value
 * Return Value      : unsigned int number of data bytes yet to be received
 *************************************************************************/
unsigned int getsUART1(unsigned int length,unsigned int *buffer,
		unsigned int uart_data_wait)
{
	int wait = 0;
	char *temp_ptr = (char *)buffer;

	while(length)                         /* read till length is 0 */
	{
		while(!DataRdyUART1())
			if( (!uart_data_wait) && (wait++ > uart_data_wait) )
				return length;           /*Time out- Return words/bytes to be read */

		if(U1MODEbits.PDSEL == 3)         /* check if TX/RX is 8bits or 9bits */
			*buffer++ = U1RXREG;          /* data word from HW buffer to SW buffer */
		else
			*temp_ptr++ = U1RXREG & 0xFF; /* data byte from HW buffer to SW buffer */

		length--;
	}

	return length;                       /* number of data yet to be received i.e.,0 */
}

/*********************************************************************
 * Function Name     : OpenUART1
 * Description       : This function configures the UART mode,
 *                     UART Interrupt modes and the Baud Rate
 * Parameters        : unsigned int config1 operation setting
 *                     unsigned int config2 TX & RX interrupt modes
 *                     unsigned int ubrg baud rate setting
 * Return Value      : None
 *********************************************************************/

void OpenUART1(unsigned int config1, unsigned int config2, unsigned int ubrg)
{
	U1BRG  = ubrg;     /* baud rate */
	U1MODE = config1;  /* operation settings */
	U1STA = config2;   /* TX & RX interrupt modes */
}

/*************************************************************************
 * Function Name     : putsUART1
 * Description       : This function puts the data string to be transmitted 
 *                     into the transmit buffer (till NULL character)
 * Parameters        : unsigned int * address of the string buffer to be 
 *                     transmitted
 * Return Value      : None
 *********************************************************************/

void putsUART1(unsigned int *buffer)
{
	char *temp_ptr = (char *)buffer;

	/* transmit till NULL character is encountered */
	while((*buffer != '\0') && (*temp_ptr != '\0')) 
	{
		while(U1STAbits.UTXBF);     /* wait if the buffer is full */

		if(U1MODEbits.PDSEL == 3)   /* check if TX is 8bits or 9bits */
			U1TXREG = *buffer++;    /* transfer data word to TX reg */
		else 
			U1TXREG = *temp_ptr++;	/* transfer data byte to TX reg */

	}
}

/*************************************************************************
 * Function Name     : ReadUART1
 * Description       : This function returns the contents of UxRXREG buffer
 * Parameters        : None
 * Return Value      : unsigned int value from UxRXREG receive buffer 
 *************************************************************************/

unsigned int ReadUART1(void)
{
	if(U1MODEbits.PDSEL == 3)
		return U1RXREG;
	else
		return U1RXREG & 0xFF;
}

/*********************************************************************
 * Function Name     : WriteUART1
 * Description       : This function writes data into the UxTXREG,
 * Parameters        : unsigned int data the data to be written
 * Return Value      : None
 *********************************************************************/

void WriteUART1(unsigned int data)
{
	if(U1MODEbits.PDSEL == 3)
		U1TXREG = data;
	else
		U1TXREG = data & 0xFF;
}

void putch(unsigned char c)
{
	while(U1STAbits.UTXBF);
	U1TXREG = c;
}

void putint16(unsigned int s) {
	while(U1STAbits.UTXBF);
	
	U1TXREG = s >> 8;
	
	while(U1STAbits.UTXBF);
	
	U1TXREG = s;
}

unsigned int getint16(void) {
	unsigned int ret;
	
	while(rxCounter < 2);
	
	if(++rx_index2 == RX_BUF_LEN)
		rx_index2 = 0;
	
	ret = rx_buf[rx_index2] << 8;
	
	if(++rx_index2 == RX_BUF_LEN)
		rx_index2 = 0;
		
	ret |= rx_buf[rx_index2];
	
	SRbits.IPL = 7;
	rxCounter-=2;
	SRbits.IPL = 0;
	return ret;
}

static unsigned char status;

void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)
{
	IFS0bits.U1RXIF = 0;

	status = U1STAbits.FERR | U1STAbits.PERR | U1STAbits.OERR;
	rxData = U1RXREG;

	if(status == 0)
	{
		if(++rx_index1 == RX_BUF_LEN)
			rx_index1 = 0;
		rx_buf[rx_index1] = rxData;
		rxCounter++;
	}

	U1STAbits.OERR = 0;
}

void putstr(const char* s) {
	while(*s) {
		putch(*s);
		s++;
	}
}

void flush(void) {
	SRbits.IPL = 7;
	rxCounter = 0;
	rx_index2 = rx_index1;
	SRbits.IPL = 0;
}

unsigned char getch(void)
{
	while(rxCounter == 0);
	//while(rx_index1 == rx_index2);	//wait for character to be received
	SRbits.IPL = 7;
	rxCounter--;
	if(++rx_index2 == RX_BUF_LEN)
		rx_index2 = 0;
	SRbits.IPL = 0;
	return rx_buf[rx_index2];
	//return data;
}

unsigned char UART_GetLastByte(void)
{
	return rx_buf[rx_index1];
}

unsigned char UART_CheckRX(void)
{
	return rxCounter;
}
// end

// ------------- UART packet protocol ------------

/*
	SCTLxxxxxxxx

	S - 1 Byte sync (0x3c)
	C - 1 Byte checksum ( upper nibble - header checksum, lower nibble payload checksum )
	T - 1 Byte type
	L - 1 Byte payload length
	x - L Bytes data
*/


#define MAX_PKT_SIZE 32
#define PACKET_SYNC 0x3c
#define PACKET_HEADER 4

/*
	@param
		length - pointer to byte where length will be written if packet is found
	@return
		0 - no packet found
		other - pointer to data of length set by this function (in this data)
*/

static uint8_t rx_get() {
	if(++rx_index2 >= RX_BUF_LEN) {
		rx_index2 = 0;
	}
	return rx_buf[rx_index2];
}

static uint8_t rx_pkt_buf[MAX_PKT_SIZE];
static uint8_t rx_pkt_type;
static uint8_t rx_pkt_len;
static uint8_t rx_pkt_checksum;
static uint8_t rx_pkt_wait_for_data = 0;
static uint8_t rx_pkt_read_cursor;

uint8_t try_read_packet(uint8_t* pkt_type, uint8_t *length) {
	uint8_t read = 0;
	uint8_t pass = 0;
	if(rx_pkt_wait_for_data == 0) {
		while(rxCounter - read >= PACKET_HEADER) {
			
			read++;
			// if found packet_sync
			if(rx_get() == PACKET_SYNC ) {
				rx_pkt_checksum = rx_get();
				rx_pkt_type = rx_get();
				rx_pkt_len = rx_get();
				read += 3;

				// check header checksum
				if( (rx_pkt_checksum >> 4) == ((rx_pkt_type+rx_pkt_len) & 0xf) ) {
					rx_pkt_wait_for_data = 1;
					break;
				}
			}
		}
	}
	
	if(rx_pkt_wait_for_data == 1 && rxCounter-read >= rx_pkt_len) {
		// check content checksum
		uint8_t c = 0;
		
		// checksum calculate for all data
		uint8_t i;
		for(i=0; i < rx_pkt_len; i++) {
			rx_pkt_buf[i] = rx_get();
			c += rx_pkt_buf[i];
		}
		
		// if checksum pass
		if( (rx_pkt_checksum & 0xf) == (c & 0xf) ) {
			*pkt_type = rx_pkt_type;
			*length = rx_pkt_len;
			pass = 1;
		}
		
		read += rx_pkt_len;
		rx_pkt_wait_for_data = 0;
	}
	
	SRbits.IPL = 7;
	if(rxCounter >= read)
		rxCounter -= read;
	else
		rxCounter = 0;
	SRbits.IPL = 0;
	
	
	if(pass) {
		rx_pkt_read_cursor = 0;
		start_packet('A');
			put_byte(*pkt_type);
		end_packet();
		return 1;
	}
	return 0;
}

uint8_t get_byte() {
	if(rx_pkt_read_cursor < rx_pkt_len)
		return rx_pkt_buf[rx_pkt_read_cursor++];
	else
		return 0;
}
uint16_t get_word() {
	if(rx_pkt_read_cursor+1 < rx_pkt_len) {
		uint16_t r = ((uint16_t)rx_pkt_buf[rx_pkt_read_cursor] << 8) | rx_pkt_buf[rx_pkt_read_cursor+1];
		rx_pkt_read_cursor += 2;
		return r;
	} else {
		return 0;
	}
}

static uint8_t pkt_buf[MAX_PKT_SIZE];
static uint8_t pkt_size = 0;

void start_packet(uint8_t type) {
	pkt_size = PACKET_HEADER;
	pkt_buf[2] = type;
}
void put_byte(int8_t b) {
	pkt_buf[pkt_size++] = b;
}
void put_word(int16_t w) {
	pkt_buf[pkt_size] = w >> 8;
	pkt_buf[pkt_size+1] = w;
	pkt_size+=2;
}

void end_packet() {
	pkt_buf[0] = PACKET_SYNC;
	uint8_t payload_size = pkt_size-PACKET_HEADER;
	pkt_buf[3] = payload_size;
	pkt_buf[1] = 0;
	int i;
	for(i=PACKET_HEADER; i < pkt_size; i++) {
		pkt_buf[1] += pkt_buf[i];
	}
	
	pkt_buf[1] = ((payload_size + pkt_buf[2]) << 4) | (pkt_buf[1] & 0xf);
	
	for(i=0; i < pkt_size; i++) {
		while(U1STAbits.UTXBF);
		U1TXREG = pkt_buf[i];
	}
}
