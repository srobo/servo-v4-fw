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

CFLAGS += -mcpu=cortex-m3 -mthumb -msoft-float -DSTM32F1 \
	  -Wall -Wextra -Os -std=gnu99 -g -fno-common \
	  -Ilibopencm3/include -DFW_VER=$(FW_VER)
#LDFLAGS += -lc -lm -L$(TOOLCHAIN_DIR)/lib/thumb/cortex-m3 -L$(TOOLCHAIN_DIR)/lib \
#	   -L$(TOOLCHAIN_DIR)/lib/stm32/f1 -lnosys -T$(LDSCRIPT) \
#	   -nostartfiles -Wl,--gc-sections,-Map=pbv4.map -mcpu=cortex-m3 \
#	   -mthumb -march=armv7-m -mfix-cortex-m3-ldrd -msoft-float
LDFLAGS += -lc -lm -Llibopencm3/lib -lopencm3_stm32f1 -lnosys -T$(LDSCRIPT) \
	   -nostartfiles -Wl,--gc-sections,-Map=sbv4.map -mcpu=cortex-m3 \
	   -mthumb -march=armv7-m -mfix-cortex-m3-ldrd -msoft-float

O_FILES = main.o
TEST_O_FILES = test.o

all: sbv4.bin sbv4_test.bin

test: sbv4_test.bin

include depend

sbv4.elf: $(O_FILES) $(LD_SCRIPT)
	$(LD) -o $@ $(O_FILES) $(LDFLAGS)
	$(SIZE) $@

sbv4_test.elf: $(TEST_O_FILES) $(LD_SCRIPT)
	$(LD) -o $@ $(TEST_O_FILES) $(LDFLAGS)
	$(SIZE) $@

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

depend: *.c
	rm -f depend
	for file in $^; do \
		$(CC) $(CFLAGS) -MM $$file -o - >> $@ ; \
	done ;

.PHONY: all test clean flash

flash: sbv4_test.elf
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
	      #  -c "reset halt" &
	$(GDB)  sbv4.elf

clean:
	-rm -f sbv4.elf depend *.o
