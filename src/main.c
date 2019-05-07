#include "config.h"
#include "init.h"
#include "hw/motor.h"

#include "packet.h"
#include "regulator.h"


#ifdef SIM
#include <unistd.h>
char* can_iface = "can0";
int main(char argc, char* argv[]) {
#else
int main(void) {
#endif

	int16_t x, y, tmp, angle, distance;
	char command, v, direction;
	int radius;
	
	// printf("%ld\n", clipl2(7, -2500));
	// printf("%ld\n", clipl2(7, 2500));
	#ifdef SIM
	if(argc > 1) {
		can_iface = argv[1];
	}
	#endif
	
	initialize();
	
	while(1)
	{
	
		// start_packet('L');
			// put_byte('L');
		// end_packet();
		#ifdef SIM
		usleep(1000);
		#endif
		Packet* pkt = pkt=try_read_packet();
		if(!pkt) continue;
		#ifdef SIM
		printf("pkt: %c (%x)\n", pkt->type, pkt->type);
		#endif
		command = pkt->type;
		reset_stuck();
		switch(command) {
			case CMD_SET_POSITION_AND_ORIENTATION:
				// x [mm], y [mm], orientation
				x = get_word();
				y = get_word();
				angle = get_word();
				set_position(x, y, angle);
				break;
			
			case CMD_SET_CONFIG:
				config_load_from_stream(pkt->size, pkt->data);
				break;
			
			case CMD_GET_CONFIG: {
				int key = get_byte();
				uint32_t val;
				int exponent;
				int sign;
				config_get_as_fixed_point(key, (int32_t*)&val, &exponent, &sign);
				start_packet(CMD_GET_CONFIG);
					put_word(val >> 16);
					put_word(val);
					put_byte(sign);
					put_byte(exponent);
				end_packet();
				break;
			}
			
			case CMD_SET_CONFIG_HASH: {
				int hash, exp, key;
				int32_t val;
				hash = get_word();
				val = get_long();
				exp = get_byte();
				config_set_as_fixed_point(config_get_key(hash), val, exp);
				break;
			}
				
			case CMD_GET_CONFIG_HASH: {
				uint32_t val;
				int exponent, sign, hash, key;
				hash = get_word();
				key = config_get_key(hash);
				config_get_as_fixed_point(key, (int32_t*)&val, &exponent, &sign);
				
				start_packet(CMD_GET_CONFIG);
					put_word(val >> 16);
					put_word(val);
					put_byte(sign);
					put_byte(exponent);
				end_packet();
				break;
			}
			
			case CMD_SAVE_CONFIG: {
				config_save_to_program_memory();
				break;
			}
			
			case CMD_SEND_STATUS_AND_POSITION:
				// read status and position
				send_status_and_position();
				break;

			case CMD_SET_SPEED:
				// set speed; Vmax(0-255)
				set_speed(get_byte());
				break;

			case CMD_SET_ROTATION_SPEED: {
				uint8_t max_speed = get_byte(); 
				uint8_t max_accel = get_byte(); 
				set_rotation_speed(max_speed, max_accel);
				break;
			}
				
			case CMD_FORWARD:
				// move forward [mm]
				distance = get_word();
				cmd_forward(distance);
				break;
				
			case CMD_FORWARD_LAZY:
				distance = get_word();
				v = get_byte();
				cmd_forward_lazy(distance, v);
				break;

				
			case CMD_ABSOLUTE_ROTATE:
				// absolute angle [degrees]
				angle = get_word();
				cmd_absrot(angle);
				break;

			case CMD_RELATIVE_ROTATE:
				// relative angle [degrees]
				angle = get_word();
				cmd_turn(angle);
				break;
				
			case CMD_TURN_AND_GO:
				// rotate to and then move to point (Xc, Yc, v, direction) [mm]
				x = get_word();
				y = get_word();
				direction = get_byte(); // + means forward, - means backward
				cmd_goto(x, y, direction); //(x, y, end_speed, direction)
				break;
				     
			case CMD_CURVE:
				x = get_word();
				y = get_word();
				angle = get_word();
				direction = get_byte();
				cmd_curve(x, y, angle, direction);
				break;
				
			case CMD_CURVE_RELATIVE:
				x = get_word();
				angle = get_word();
				cmd_curve_rel(x, angle);
				break;
				
			case CMD_DIFF_DRIVE:
				x = get_word();
				y = get_word();
				angle = get_word();
				direction = get_byte();
				cmd_diff_drive( x, y, angle, direction );
				break;
			
			case CMD_MOVE_TO: {
				// x [mm], y [mm], direction {-1 - backwards, 0 - pick closest, 1 - forward}
				x = get_word();
				y = get_word();
				radius = get_word();
				direction = get_byte();
				
				cmd_move(x, y, radius, direction);
				break;
			}
			case CMD_HARD_STOP:
				cmd_stop();
				break;
				
			case CMD_SMOOTH_STOP:
				cmd_smooth_stop();
				break;
				
			case CMD_SOFT_STOP:
				// stop and kill PWM
				cmd_soft_stop();
				break;
				
			case CMD_RESET_DRIVER:
				// reset position, status and speed
				reset_driver();
				break;
			
			case CMD_UNSTUCK:
				reset_stuck();
				break;
			
			case 'E':
				start_packet('E');
				put_byte(pkt->size);
				end_packet();
				break;
			/*
			case 'e':{
					uint8_t cnt = 0;
					while(1) {
						start_packet('e');
							put_byte(cnt);
						end_packet();
						cnt++;
						__delay_ms(1);
					}
				}
				break;
			*/
			
			case CMD_MOTOR: {
				x = get_word();
				y = get_word();
				cmd_motor_const(x, y);
				break;
			}
			
			case CMD_KEEP_SPEED: {
				x = (int16_t)get_word();
				y = (int16_t)get_word();
				cmd_speed_const(x, y);
				break;
			}
			
			case CMD_MOTOR_INIT: {
				motor_init();
				break;
			}
			
			/*
			// debugging bootloader
			case '.': {
				uint32_t adr = get_long();
				uint16_t len = get_word();
				
				uint8_t d[64*3];
				
				_DISI = 1;
				// SRbits.IPL = 7;
				while(len > 0) {
					int block = 64;
					if(len < block) block = len;
					read_prog_mem(adr, d, block);
					
					int i;
					int left = len*3;
					int ofs = 0;
					int c = 0;
					while(left > 0) {
						int l = 7;
						if(left < l) l = left;
						start_packet('r');
							for(i=0; i < l; i++) {
								put_byte(d[ofs+i]);
							}
						end_packet();
						__delay_ms(1);
						left -= l;
						ofs += l;
					}
					adr += 64*2;
					len -= block;
				}
				break;
			}
			case ']': {
				uint32_t adr = get_long();
				// start_packet('[');
				// end_packet();
				
				// uint8_t d[6];
				// int i;
				// for(i=0; i < 3; i++) {
					// d[i] = get_byte();
				// }
				
				// _DISI = 1;
				
				// _GIE = 1;
				
				break;
			}
			*/

			default:
				report_status(STATUS_ERROR);
		}		
	}

	return 0;
}
