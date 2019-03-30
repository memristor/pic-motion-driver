# TOOLCHAIN_PATH := /opt/microchip/xc16/v1.36/bin/
TOOLCHAIN_PATH := /opt/microchip/xc16/v1.30/bin/
TOOLCHAIN := $(TOOLCHAIN_PATH)xc16-


robot := big
# board := V1_NEW
# board := V1
board = V2


mplab=v4.15
mplab=v5.15
dev := can0

##########################################
src_base := 			\
	src/main.c			\
	src/init.c			\
	src/regulator.c     \
	src/packet.c        \
	src/math.c          \
	src/config.c        \
	src/hw/can.c        \
	src/hw/uart.c       \
	src/hw/interrupt.c  \
	
	
hw_src := 		  \
	init.c		  \
	can.c         \
	uart.c        \
	encoder.c     \
	motor.c       \
	timer.c       \
	fuses.c  	  \
	interrupt.c   \
	

.PHONY: sim config

sim: board = SIM

lcv := $(shell echo $(board) | tr A-Z a-z)
ucv := $(shell echo $(board) | tr a-z A-Z)
ver := $(ucv)

flags := 
###### TOOLCHAIN CONFIG ######
ifeq ($(ucv),V1)
	chip=33FJ128MC802
	report_flags := -Wa,-alh,-L -Wl,--report-mem 
	flags := $(flags) -T p33FJ128MC802.gld -mcpu=$(chip) -omf=elf\
		-Wl,,--defsym=__MPLAB_BUILD=1,,--script=p33FJ128MC802.gld,--check-sections,\
		--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,\
		--stackguard=16,--library-path="..",--no-force-link,--smart-io $(report_flags) -Wfatal-error
	# 		--ROM,default,-e000-11ff
	flags2 := -omf=elf -no-legacy-libc -msmart-io=1 -Wall  -mcpu=$(chip)
endif

ifeq ($(ucv),V2)
	TOOLCHAIN_PATH := /opt/microchip/xc32/v2.15/bin/
	TOOLCHAIN := $(TOOLCHAIN_PATH)xc32-
	chip=32MK1024MCF064
	harmony=/home/nikola/microchip/harmony/v2_06/
	flags := $(flags)  -mprocessor=$(chip)  	-L$(harmony)/bin/framework/peripheral/ -l:PIC$(chip)_peripherals.a
	flags2 := -mprocessor=$(chip) -I$(harmony)/framework/peripheral -I$(harmony)/framework/ -O1
	src := $(src_v2)
endif
###############################


src := $(src_base) $(addprefix src/hw/$(lcv)/,$(hw_src))
obj := $(patsubst %.c,%.o,$(src))
app := AppImage

clock=USE_CRYSTAL
# clock=USE_FRCPLL
# -msfr-warn=off

############################

$(app).hex: config AppImage.elf
	$(TOOLCHAIN)bin2hex -a AppImage.elf
	
$(app).elf: $(obj)
	$(TOOLCHAIN)gcc $^ -o $@  $(flags) 
	
%.o: %.c
	$(TOOLCHAIN)gcc  $^ -c -DBOARD_$(ver) -D$(clock) -o $@ $(flags2)


############################

config:
	python3 conf/gen_config.py mcu $(robot) > src/config_keys.h
	echo '#include' '"'hw/$(lcv)/hw.h'"' >> src/config_keys.h

js:
	python3 conf/gen_config.py js $(robot) > config_const.js
	
py:
	python3 conf/gen_config.py py $(robot) > const_motion.py


src_sim := $(src_base) $(addprefix src/hw/sim/,$(hw_src))
sim: $(src_sim)
	python3 conf/gen_config.py mcu sim > src/config_keys.h
	echo '#include' '"'hw/sim/hw.h'"' >> src/config_keys.h
	gcc $^ -g -o sim -DSIM -lm -pthread


sim_dev:
	sudo modprobe vcan
	sudo ip link add dev $(dev) type vcan
	sudo ip link set up $(dev)
	
dev: sim_dev

ipe_path:="/opt/microchip/mplabx/v5.15/mplab_platform/mplab_ipe/ipecmd.jar"
upload:
	mkdir -p tmp; cd tmp; sudo java -jar "$(ipe_path)" -TPPK3 -P33FJ128MC802 -M -F../AppImage.hex; sudo rm -rf tmp
clean:
	rm -f const_motion.py config_const.js src/config_keys.h
	find -name '*.o' -delete
	rm -f $(app).elf $(app).hex regions.txt
	rm -rf sim tmp
