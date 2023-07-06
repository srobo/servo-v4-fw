#pragma once

#include <stdbool.h>

// Uses 30 nop commands for approximately 36 clocks per loop
#define delay(ms) do { \
    for (int i = 0; i < ms * 2000; i++) \
        __asm__( \
            "nop;nop;nop;nop;nop;" \
            "nop;nop;nop;nop;nop;" \
            "nop;nop;nop;nop;nop;" \
            "nop;nop;nop;nop;nop;" \
            "nop;nop;nop;nop;nop;" \
            "nop;nop;nop;nop;nop;"); \
    } while(0)

// Varables uses across modules
// These are defined in main.c

#define SERIALNUM_BOOTLOADER_LOC 0x08001FE0
#define REENTER_BOOTLOADER_RENDEZVOUS 0x08001FFC
#define BOARD_NAME_SHORT "SBv4B"

extern bool detected_power_good;

extern int board_voltage_mv;
extern int board_current_ma;
