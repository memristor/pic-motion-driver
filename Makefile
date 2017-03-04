TOOLCHAIN_PATH := /opt/microchip/xc16/v1.30/bin/
TOOLCHAIN := $(TOOLCHAIN_PATH)xc16-

src := \
	src/main.c \
	src/uart.c \
	src/init.c \
	src/pwm.c \
	src/math.c \
	src/regulator.c \


obj := $(patsubst %.c,%.o,$(src))

test.hex: test.elf
	$(TOOLCHAIN)bin2hex test.elf

test.elf: $(obj)
	$(TOOLCHAIN)gcc $^ -o $@ -T p33FJ128MC802.gld -mcpu=33FJ128MC802 -omf=elf  -Wl,,--defsym=__MPLAB_BUILD=1,,--script=p33FJ128MC802.gld,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--library-path="..",--no-force-link,--smart-io

%.o: %.c
	$(TOOLCHAIN)gcc -mcpu=33FJ128MC802 $^ -c -o $@ -omf=elf -no-legacy-libc -msmart-io=1 -Wall -msfr-warn=off

clean:
	rm -f src/*.o
	rm -f test.elf test.hex
