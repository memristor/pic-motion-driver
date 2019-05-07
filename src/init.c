#include "init.h"
void initialize(void) {
	config_init();
	packet_init();
	regulator_init();
	// must come after regulator_init !
	config_load();
	
	hw_init();
	reset_driver();
	motor_init();
}
