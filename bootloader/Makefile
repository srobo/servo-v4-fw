PREFIX = arm-none-eabi
CC = $(PREFIX)-gcc

# Directory containing include/ and lib/ subdirectories of libopencm3 installation.
LIBOPENCM3 ?= $(realpath ../libopencm3)

# Export these facts to bootloader
SR_BOOTLOADER_VID=0x1BDA  # ECS VID
SR_BOOTLOADER_PID=0x0011  # Servo board PID
SR_BOOTLOADER_REV=0x0402  # BCD version, board 4.0.
SR_BOOTLOADER_FLASHSIZE=0x8000  # 32kb of onboard flash.
export SR_BOOTLOADER_VID SR_BOOTLOADER_PID SR_BOOTLOADER_REV SR_BOOTLOADER_FLASHSIZE

CFLAGS += -mcpu=cortex-m3 -mthumb -msoft-float -DSTM32F1 \
	  -Wall -Wextra -Os -std=gnu99 -g -fno-common \
	  -I$(LIBOPENCM3)/include -DFW_VER=$(FW_VER) -g \
	  -DSR_DEV_VID=$(SR_BOOTLOADER_VID) \
	  -DSR_DEV_PID=$(SR_BOOTLOADER_PID) \
	  -DSR_DEV_REV=$(SR_BOOTLOADER_REV)

bootloader: setup force_bootloader.o
	FORCE_BOOTLOADER_OBJ=$(realpath force_bootloader.o) $(MAKE) -C dfu-bootloader usb_dfu.bin
	cp dfu-bootloader/usb_dfu.bin usb_dfu.bin

setup:
	$(Q)$(MAKE) -C dfu-bootloader/libopencm3

clean:
	$(Q)$(MAKE) -C dfu-bootloader clean
	rm force_bootloader.o

# Using implit C compile rule: '$(CC) $(CPPFLAGS) $(CFLAGS) -c'
