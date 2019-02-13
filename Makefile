TOOLCHAIN_PATH := /opt/microchip/xc16/v1.30/bin/
TOOLCHAIN := $(TOOLCHAIN_PATH)xc16-


robot := big
# board := V1_NEW
board := V2
mplab=v4.15
dev := vcan0

##########################################

src := \
	src/main.c \
	src/com/uart.c \
	src/com/can.c \
	src/com/packet.c \
	src/drive/encoder.c \
	src/drive/motor.c \
	src/timer.c \
	src/util/math.c \
	src/regulator.c \
	src/config.c \
	src/init.c \
	src/bootloader.c
	
src_v2 := \
	src/main.c \
	src/com/uart.v2.c \
	src/com/can.v2.c \
	src/com/packet.c \
	src/drive/encoder.v2.c \
	src/drive/motor.v2.c \
	src/timer.c \
	src/util/math.c \
	src/regulator.c \
	src/config.c \
	src/init.v2.c
# 	src/bootloader.c
	
sim_src := \
	src/regulator.c \
	src/config.c \
	src/main.c \
	src/init.sim.c \
	src/com/packet.c \
	src/util/math.c \
	src/timer.sim.c \
	src/drive/motor.sim.c \
	src/drive/encoder.sim.c \
	src/com/uart.sim.c \
	src/com/can.sim.c

.PHONY: sim config


obj = $(patsubst %.c,%.o,$(src))
app := AppImage

ifeq ($(board),V2)
	TOOLCHAIN_PATH := /opt/microchip/xc32/v2.15/bin/
	TOOLCHAIN := $(TOOLCHAIN_PATH)xc32-
	chip=32MK1024MCF064
	flags := -mips32 -mprocessor=$(chip)
	flags2 := -mips32 -mprocessor=$(chip)
	src := $(src_v2)
else
	chip=33FJ128MC802
	report_flags := -Wa,-alh,-L -Wl,--report-mem 
	flags := -T p33FJ128MC802.gld -mcpu=$(chip) -omf=elf\
		-Wl,,--defsym=__MPLAB_BUILD=1,,--script=p33FJ128MC802.gld,--check-sections,\
		--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,\
		--stackguard=16,--library-path="..",--no-force-link,--smart-io $(report_flags) -Wfatal-error
	# 		--ROM,default,-e000-11ff
	flags2 := -omf=elf -no-legacy-libc -msmart-io=1 -Wall  -mcpu=$(chip)
endif


clock=USE_CRYSTAL
# clock=USE_FRCPLL
# -msfr-warn=off

############################

$(app).hex: config AppImage.elf
	$(TOOLCHAIN)bin2hex -a AppImage.elf
	
$(app).elf: $(obj)
	$(TOOLCHAIN)gcc $^ -o $@  $(flags) 
	
%.o: %.c
	$(TOOLCHAIN)gcc  $^ -c -DBOARD_$(board) -D$(clock) -o $@ $(flags2)


############################

config:
	python3 conf/gen_config.py mcu $(robot) > src/config_keys.h

js:
	python3 conf/gen_config.py js $(robot) > config_const.js
	
py:
	python3 conf/gen_config.py py $(robot) > const_motion.py
	
sim: $(sim_src)
	python3 conf/gen_config.py mcu sim > src/config_keys.h
	gcc $^ -g -o sim -DSIM -lm -pthread


sim_dev:
	sudo modprobe vcan
	sudo ip link add dev $(dev) type vcan
	sudo ip link set up $(dev)
	
dev: sim_dev

upload:
	mkdir -p tmp; cd tmp; sudo java -jar /opt/microchip/mplabx/$(mplab)/mplab_ipe/ipecmd.jar -TPPK3 -P33FJ128MC802 -M -F../AppImage.hex; sudo rm -rf tmp
	
clean:
	rm -f const_motion.py config_const.js src/config_keys.h
	rm -f src/*.o src/drive/*.o src/com/*.o src/util/*.o
	rm -f $(app).elf $(app).hex regions.txt
	rm -rf sim tmp
