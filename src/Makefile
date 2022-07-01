##
## This file is part of the libopencm3 project.
##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
## Copyright (C) 2010 Piotr Esden-Tempski <piotr@esden.net>
##
## This library is free software: you can redistribute it and/or modify
## it under the terms of the GNU Lesser General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This library is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU Lesser General Public License for more details.
##
## You should have received a copy of the GNU Lesser General Public License
## along with this library.  If not, see <http://www.gnu.org/licenses/>.
##
FW_VER ?= 3.0

# Name of C file with main function
BINARY = main
# Name of all other C files to be compiled (with .o extension)
OBJS = cdcacm.o msg_handler.o i2c.o servo.o systick.o led.o

LDSCRIPT = $(OPENCM3_DIR)/../utils/stm32-sbv4.ld

# Default USB PID:VID
ifeq ($(VID),)
ifeq ($(PID),)
VID	:=	1bda
PID	:=	0011
endif
endif

PYTHON ?= $(shell which python3)
# Convert FW_VER to BCD
FW_REV = $(shell echo $(FW_VER) | $(PYTHON) -c 'print("".join(f"{int(x):02x}" for x in input().split(".")))')

DEFS = -DFW_VER=\"$(FW_VER)\" \
	  -DSR_DEV_VID=0x$(VID) \
	  -DSR_DEV_PID=0x$(PID) \
	  -DSR_DEV_REV=0x$(FW_REV)

all: bin

include ../utils/rules.mk

elf: $(BINARY).elf
bin: $(BINARY).bin
hex: $(BINARY).hex
srec: $(BINARY).srec
list: $(BINARY).list
GENERATED_BINARIES=$(BINARY).elf $(BINARY).bin $(BINARY).hex $(BINARY).srec $(BINARY).list $(BINARY).map

images: $(BINARY).images
flash: $(BINARY).flash
dfu: $(BINARY).dfu

# Define a helper macro for debugging make errors online
# you can type "make print-OPENCM3_DIR" and it will show you
# how that ended up being resolved by all of the included
# makefiles.
print-%:
	@echo $*=$($*)

clean:
	@#printf "  CLEAN\n"
	$(Q)$(RM) $(GENERATED_BINARIES) generated.* $(OBJS) $(OBJS:%.o=%.d)

stylecheck: $(STYLECHECKFILES:=.stylecheck)
styleclean: $(STYLECHECKFILES:=.styleclean)

size: $(BINARY).elf
	$(Q)arm-none-eabi-size -G -d $(BINARY).elf

.PHONY: images clean stylecheck styleclean elf bin hex srec list

-include $(OBJS:.o=.d)
