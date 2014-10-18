FW_VER=1

PREFIX = arm-none-eabi
CC = $(PREFIX)-gcc
LD = $(PREFIX)-gcc
SIZE = $(PREFIX)-size
GDB = $(PREFIX)-gdb
OBJCOPY = $(PREFIX)-objcopy
OOCD = openocd

LDSCRIPT = stm32-sbv4.ld
OOCD_BOARD = oocd/sbv4.cfg

# Export these facts to bootloader
SR_BOOTLOADER_VID=0x1BDA  # ECS VID
SR_BOOTLOADER_PID=0x0011  # Servo board PID
SR_BOOTLOADER_REV=0x0400  # BCD version, board 4.0.
export SR_BOOTLOADER_VID SR_BOOTLOADER_PID SR_BOOTLOADER_REV

CFLAGS += -mcpu=cortex-m3 -mthumb -msoft-float -DSTM32F1 \
	  -Wall -Wextra -Os -std=gnu99 -g -fno-common \
	  -Ilibopencm3/include -DFW_VER=$(FW_VER) -g \
	  -DSR_DEV_VID=$(SR_BOOTLOADER_VID) \
	  -DSR_DEV_PID=$(SR_BOOTLOADER_PID) \
	  -DSR_DEV_REV=$(SR_BOOTLOADER_REV)
BASE_LDFLAGS += -lc -lm -Llibopencm3/lib \
	   -Llibopencm3/lib/stm32/f1 -lnosys \
	   -nostartfiles -Wl,--gc-sections,-Map=sbv4.map -mcpu=cortex-m3 \
	   -mthumb -march=armv7-m -mfix-cortex-m3-ldrd -msoft-float
LDFLAGS = $(BASE_LDFLAGS) -T$(LDSCRIPT)

BOOTLOADER_OBJS = dfu-bootloader/usb_dfu_blob.o dfu-bootloader/usbdfu.o
O_FILES = main.o led.o sbusb.o $(BOOTLOADER_OBJS)
TEST_O_FILES = test.o led.o servo.o usart.o battery.o cdcacm.o $(BOOTLOADER_OBJS)

all: force_bootloader.o bootloader $(O_FILES) sbv4.bin sbv4_test.bin sbv4_noboot.bin

test: sbv4_test.bin

include depend

bootloader:
	FORCE_BOOTLOADER_OBJ=`pwd`/force_bootloader.o $(MAKE) -C dfu-bootloader

sbv4.elf: $(O_FILES) $(LDSCRIPT)
	$(LD) -o $@ $(O_FILES) $(LDFLAGS) -lopencm3_stm32f1
	$(SIZE) $@

sbv4_test.elf: $(TEST_O_FILES) $(LDSCRIPT)
	$(LD) -o $@ $(TEST_O_FILES) $(LDFLAGS) -lopencm3_stm32f1
	$(SIZE) $@

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

# Produce a no-bootloader binary, suitable for shunting straight into the app
# segment of flash, by droping the first 8k of the flat image.
sbv4_noboot.bin: sbv4.elf
	tmpfile=`mktemp /tmp/sr-sbv4-XXXXXXXX`; $(OBJCOPY) -O binary $< $$tmpfile; dd if=$$tmpfile of=$@ bs=4k skip=2; rm $$tmpfile

depend: *.c
	rm -f depend
	for file in $^; do \
		$(CC) $(CFLAGS) -MM $$file -o - >> $@ ; \
	done ;

.PHONY: all test clean flash bootloader

flash: sbv4.elf
	$(OOCD) -f "$(OOCD_BOARD)" \
	        -c "init" \
	        -c "reset init" \
	        -c "stm32f1x mass_erase 0" \
	        -c "flash write_image $<" \
	        -c "reset" \
	        -c "shutdown"

debug: sbv4.elf
	$(OOCD) -f "$(OOCD_BOARD)" \
	        -c "init" \
	        -c "reset halt" &
	$(GDB)  $^ -ex "target remote localhost:3333" -ex "mon reset halt" && killall openocd

clean:
	$(MAKE) -C dfu-bootloader clean
	-rm -f sbv4.elf depend *.o
