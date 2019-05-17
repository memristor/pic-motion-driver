#include "hw.h"
#include "../bootloader.h"
#include "../../packet.h"



#define EEKEY1 0xEDB7
#define EEKEY2 0x1248

void EEInitialize(void) // Basic EEPROM enable and initialization
{ 
    __builtin_disable_interrupts();
    
    CFGCON2bits.EEWS = 5;// 8 = for system clocks < 75MHz 
    
    EECONbits.ON = 1;

	
    while (EECONbits.RDY == 0);// Wait until EEPROM is ready (~125 us)

    EECONbits.WREN = 1;// Enable writing to the EEPROM
    EECONbits.CMD = 0b100;// Configuration register Write command (WREN bit must be set)

	
    EEADDR = 0x00;// Addr 0x00 = DEVEE1; EEADR<11:0>: Data EEPROM Address bits
    EEDATA = DEVEE0; //EEDATA<31:0>: Data EEPROM Data bits ; Public Test Flash -> DATA EE CAL(DEVEE0-DEVEE3)
    EEKEY = EEKEY1; // Unlock the EEPROM to enable writing
    EEKEY = EEKEY2;
    EECONSET = _EECON_RW_MASK;
    while (EECONbits.RW); // desired

    /*
    EEADDR = 0x04;// Addr 0x04 = DEVEE2;
    EEDATA = DEVEE1;
    EEKEY = EEKEY1; // Unlock the EEPROM to enable writing
    EEKEY = EEKEY2;
    EECONSET = _EECON_RW_MASK;
    while (EECONbits.RW); // desired

    
    EEADDR = 0x08;// Addr 0x08 = DEVEE3;
    EEDATA = DEVEE2;
    EEKEY = EEKEY1; // Unlock the EEPROM to enable writing
    EEKEY = EEKEY2;
    EECONSET = _EECON_RW_MASK;
    while (EECONbits.RW); // desired

    EEADDR = 0x0C;// Addr 0x08 = DEVEE3;
    EEDATA = DEVEE3;
    EEKEY = EEKEY1; // Unlock the EEPROM to enable writing
    EEKEY = EEKEY2;
    EECONSET = _EECON_RW_MASK;
    while (EECONbits.RW); // desired
	*/
	
    EECONbits.WREN = 0; // Turn off writes.    
	__builtin_enable_interrupts();
}

uint32_t data_EEPROM_read(uint32_t ee_addr)
{
    uint32_t Data;
    while (EECONbits.RDY==0);
    EEADDR = ee_addr & 0xFFC; // Set address on 32-bit boundary
    EECONbits.CMD = 0; // Load CMD<2:0> with
    // Data EEPROM read command
    EECONbits.WREN = 0; // Access for read

    __builtin_disable_interrupts();
    EEKEY = EEKEY1; // Write unlock sequence
    EEKEY = EEKEY2;
    EECONSET = _EECON_RW_MASK; // Start the operation 
    __builtin_enable_interrupts();

    while (EECONbits.RW==1); // Wait until read is complete
    Data = EEDATA; // Read the data
    
    return Data;
}


void data_EEPROM_write(uint32_t ee_addr, uint32_t ee_data)
{
	while (EECONbits.RDY==0);
    EECONbits.CMD = 1; // Load CMD<2:0> with write command
    EECONbits.WREN = 1; // Access for write

    EEADDR = ee_addr & 0xFFC; // Load address on a 32-bit boundary
    EEDATA = ee_data;

    __builtin_disable_interrupts();
    EEKEY = EEKEY1; // Write unlock sequence
    EEKEY = EEKEY2;
    EECONSET = _EECON_RW_MASK; 
    __builtin_enable_interrupts();

    while (EECONbits.RW == 1);
}






static int initialized = 0;
int8_t eeprom_ram[EEPROM_SIZE+1];

int eeprom_initialized() {
	eeprom_load();
	uint32_t *p = (uint32_t*)(eeprom_get_ptr());
	start_packet('H');
		put_long(*p);
	end_packet();
	return *p;
}

void eeprom_save() {
	if(!initialized) {
		EEInitialize();
		initialized=1;
	}
	int32_t i;
	uint32_t *p1 = (uint32_t*)(eeprom_get_ptr());
	*p1 = 0xffffffff;
	
	for(i=0; i < EEPROM_SIZE/4; i++) {
		uint32_t *p = (uint32_t*)(eeprom_get_ptr());
		data_EEPROM_write(i*4, *(p+i));
	}
	eeprom_load();
	uint32_t *p = (uint32_t*)(eeprom_get_ptr());
	start_packet('J');
		put_long(*p);
	end_packet();
}

void eeprom_load() {
	if(!initialized) {
		EEInitialize();
		initialized=1;
	}
	
	int32_t i;
	for(i=0; i < EEPROM_SIZE/4; i++) {
		uint32_t *p = (uint32_t*)(eeprom_get_ptr());
		//*p = data_EEPROM_read(i);
		*(p+i) = data_EEPROM_read(i*4);
	}
	uint32_t *p = (uint32_t*)(eeprom_get_ptr());
	start_packet('F');
		put_long(*p);
	end_packet();
}

int8_t* eeprom_get_ptr() {
	return eeprom_ram;
}

void BOOT bootloader_start() {

}

void BOOT bootloader_erase_page(uint32_t adr) {
	
}

void BOOT bootloader_write_row(uint32_t adr, uint8_t* data, int len) {

}

void BOOT bootloader_write_sequence() {

}

void BOOT bootloader_write_config() {
}

// 
void BOOT read_prog_mem(uint32_t adr, uint8_t* data, int num) {
	
}
