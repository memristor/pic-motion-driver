#ifndef BOOTLOADER_H
#define BOOTLOADER_H

void bootloader_start();


#ifndef SIM
	#define BOOT __attribute__((section ("bootloader")))
	void bootloader_write_sequence();
	void BOOT read_prog_mem(uint32_t adr, uint8_t* data, int num);
	void BOOT bootloader_write_row(uint32_t adr, uint8_t* data, int len);
	void BOOT bootloader_erase_page(uint32_t adr);
#else
	#define BOOT
#endif

#endif
