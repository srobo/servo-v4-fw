#pragma once

#include <stdbool.h>

// Varables uses across modules
// These are defined in main.c

#define SERIALNUM_BOOTLOADER_LOC 0x08001FE0
#define BOARD_NAME_SHORT "SBv4B"

extern bool detected_power_good;

extern int board_voltage_mv;
extern int board_current_ma;

extern bool current_sense_updated;
