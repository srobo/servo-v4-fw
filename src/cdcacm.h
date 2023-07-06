#pragma once

void usb_init(void);
void usb_deinit(void);
void usb_poll(void);

extern bool re_enter_bootloader;
