#ifndef __CDCACM_H
#define __CDCACM_H

#include <libopencm3/usb/usbd.h>

usbd_device* cdcacm_init(void);

void cdcacm_poll(usbd_device*);

void cdcacm_send(usbd_device*, char*);

#endif /* __CDCACM_H */
