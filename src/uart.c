#include "uart.h"
#include <p33FJ128MC802.h>
#include <libpic30.h>

//BAFER ZA SERIJSKU KOMUNIKACIJU
static volatile unsigned char rx_buf[RX_BUF_LEN];
static volatile unsigned char rx_index1 = 0;
static volatile unsigned char rx_index2 = 0;
static volatile unsigned char rxData;
static volatile unsigned char rxCounter = 0;

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

void OpenUART1(unsigned int config1,unsigned int config2, unsigned int ubrg)
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
/*
   unsigned char getch()
   {
   U1STAbits.OERR=0;		//skrnavac usrani mora rucno da se resetuje
   while (!DataRdyUART1());	//ceka da stigne karakter preko serijskog porta
   return (unsigned char)(ReadUART1() & 0x00ff);
   }
   */

void putch(unsigned char c)
{
	//while(BusyUART1());
	//WriteUART1(c);
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
	//while(rx_index1 == rx_index2);	//wait for character to be received
	SRbits.IPL = 7;
	if(++rx_index2 == RX_BUF_LEN)
		rx_index2 = 0;
	
	ret = rx_buf[rx_index2] << 8;
	
	if(++rx_index2 == RX_BUF_LEN)
		rx_index2 = 0;
		
	ret |= rx_buf[rx_index2];
	
	rxCounter-=2;
	SRbits.IPL = 0;
	return ret;
}

void dbg(unsigned char code, unsigned int val) {
	putch(code);
	putint16(val);
}

void SendLong(long num)
{
	unsigned char c1, c10, c100, c1000, c10000, c100000;

	if(num < 0)
	{
		while(BusyUART1());
		WriteUART1('-');
		num = -num;
	}
	else
	{
		while(BusyUART1());
		WriteUART1('+');
	}

	c1 = num % 10;
	num = num / 10;

	c10 = num % 10;
	num = num / 10;

	c100 = num % 10;
	num = num / 10;

	c1000 = num % 10;
	num = num / 10;

	c10000 = num % 10;
	num = num / 10;

	c100000 = num % 10;
	num = num / 10;

	while(BusyUART1());
	WriteUART1('0' + c100000);

	while(BusyUART1());
	WriteUART1('0' + c10000);

	while(BusyUART1());
	WriteUART1('0' + c1000);

	while(BusyUART1());
	WriteUART1('0' + c100);

	while(BusyUART1());
	WriteUART1('0' + c10);

	while(BusyUART1());
	WriteUART1('0' + c1);
}

void NewLine(void)
{
	putch('\r');
	putch('\n');
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
