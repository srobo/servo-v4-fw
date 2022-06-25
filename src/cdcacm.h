#pragma once

void usb_init(void);
void usb_deinit(void);
void usb_poll(void);

extern bool re_enter_bootloader;

#define delay(x) do { for (int i = 0; i < x * 1000; i++) \
                          __asm__("nop"); \
                    } while(0)
