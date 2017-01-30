/* 
 * File:   uart.h
 * Author: mrmot
 *
 * Created on November 24, 2012, 3:39 PM
 */
 
#include <stdint.h>

#ifndef UART_H
#define	UART_H

#define RX_BUF_LEN  32
#define RX_BUF_MASK  0x0f

#ifndef FCY
#define FCY 29491200ULL
#endif

void UART_Init(long);

/**********************************************************************
 * Function Name     : BusyUART1
 * Description       : This returns status whether the transmission
 is in progress or not, by checking Status bit TRMT
 * Parameters        : None
 * Return Value      : char info whether transmission is in progress
 ***********************************************************************/

char BusyUART1(void);

/*********************************************************************
 * Function Name     : CloseUART1
 * Description       : This function disables the UART and clears the
 *                     Interrupt enable & flag bits
 * Parameters        : None
 * Return Value      : None
 *********************************************************************/

void CloseUART1(void);

/*********************************************************************
 * Function Name     : ConfigIntUART1
 * Description       : This function sets priority for RX,TX interrupt
 *                     and enable/disables the interrupt
 * Parameters        : unsigned int config enable/disable and priority
 * Return Value      : None
 *********************************************************************/
void ConfigIntUART1(unsigned int config);

/*********************************************************************
 * Function Name     : DataRdyUart1
 * Description       : This function checks whether there is any data
 *                     that can be read from the input buffer, by
 *                     checking URXDA bit
 * Parameters        : None
 * Return Value      : char if any data available in buffer
 *********************************************************************/
char DataRdyUART1(void);

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
		unsigned int uart_data_wait);

/*********************************************************************
 * Function Name     : OpenUART1
 * Description       : This function configures the UART mode,
 *                     UART Interrupt modes and the Baud Rate
 * Parameters        : unsigned int config1 operation setting
 *                     unsigned int config2 TX & RX interrupt modes
 *                     unsigned int ubrg baud rate setting
 * Return Value      : None
 *********************************************************************/

void OpenUART1(unsigned int config1,unsigned int config2, unsigned int ubrg);

/*************************************************************************
 * Function Name     : putsUART1
 * Description       : This function puts the data string to be transmitted
 *                     into the transmit buffer (till NULL character)
 * Parameters        : unsigned int * address of the string buffer to be
 *                     transmitted
 * Return Value      : None
 *********************************************************************/

void putsUART1(unsigned int *buffer);

/*************************************************************************
 * Function Name     : ReadUART1
 * Description       : This function returns the contents of UxRXREG buffer
 * Parameters        : None
 * Return Value      : unsigned int value from UxRXREG receive buffer
 *************************************************************************/

unsigned int ReadUART1(void);

/*********************************************************************
 * Function Name     : WriteUART1
 * Description       : This function writes data into the UxTXREG,
 * Parameters        : unsigned int data the data to be written
 * Return Value      : None
 *********************************************************************/

void WriteUART1(unsigned int data);

unsigned char UART_CheckRX(void);
unsigned char UART_GetLastByte(void);

unsigned char getch(void);
unsigned int getint16(void);
void putch(unsigned char c);
void putint16(unsigned int c);
void putstr(const char* s);


// -------- uart packet protocol ----------

// reading packet
uint8_t try_read_packet(uint8_t* pkt_type, uint8_t *length);
// after succeeding try_read_packet
uint8_t get_byte();
uint16_t get_word();


// writing packet
void start_packet(uint8_t type);
void put_byte(uint8_t b);
void put_word(uint16_t w);
#define put_byte_word(b,w) put_byte(b); put_word(w);
void end_packet();

#endif	/* UART_H */
