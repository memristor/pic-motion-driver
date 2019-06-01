#ifndef BOOTLOADER_H
#define BOOTLOADER_H
#include <stdint.h>
#include "../packet.h"
void bootloader_start();
void sw_reset();
#ifndef SIM
	#define BOOT 
	// #define BOOT __attribute__((section ("bootloader")))
	void bootloader_write_sequence();
	void BOOT read_prog_mem(uint32_t adr, uint8_t* data, int num);
	void BOOT bootloader_write_row(uint32_t adr, uint8_t* data, int len);
	void BOOT bootloader_erase_page(uint32_t adr);
#else
	#define BOOT
#endif

	#define EEPROM_SIZE 256*8 // 256 saved configs
	int eeprom_initialized();
	void eeprom_load();
	void eeprom_save();
	int8_t* eeprom_get_ptr();

#endif
