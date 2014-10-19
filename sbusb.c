#include <stdlib.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/dfu.h>

#include "usb.h"

#include "dfu-bootloader/usbdfu.h"

#define delay(x) do { for (int i = 0; i < x * 1000; i++) \
                          __asm__("nop"); \
                    } while(0);

bool re_enter_bootloader = false;

static usbd_device *usbd_dev;

static const struct usb_device_descriptor usb_descr = {
        .bLength = USB_DT_DEVICE_SIZE,
        .bDescriptorType = USB_DT_DEVICE,
        .bcdUSB = 0x0200,
        .bDeviceClass = 0,
        .bDeviceSubClass = 0,
        .bDeviceProtocol = 0,
        .bMaxPacketSize0 = 64,
        .idVendor = SR_DEV_VID,
        .idProduct = SR_DEV_PID,
        .bcdDevice = SR_DEV_REV,
        .iManufacturer = 1,
        .iProduct = 2,
        .iSerialNumber = 3,
        .bNumConfigurations = 1,
};

const struct usb_dfu_descriptor sr_dfu_function = {
        .bLength = sizeof(struct usb_dfu_descriptor),
        .bDescriptorType = DFU_FUNCTIONAL,
        .bmAttributes = USB_DFU_CAN_DOWNLOAD | USB_DFU_WILL_DETACH,
        .wDetachTimeout = 255,
        .wTransferSize = 128,
        .bcdDFUVersion = 0x011A,
};

const struct usb_interface_descriptor dfu_iface = {
        .bLength = USB_DT_INTERFACE_SIZE,
        .bDescriptorType = USB_DT_INTERFACE,
        .bInterfaceNumber = 0,
        .bAlternateSetting = 0,
        .bNumEndpoints = 0,
        .bInterfaceClass = 0xFE, // Application specific class code
        .bInterfaceSubClass = 0x01, // DFU
        .bInterfaceProtocol = 0x01, // Protocol 1.0
        .iInterface = 4,
	.extra = &sr_dfu_function,
	.extralen = sizeof(sr_dfu_function),
};

const struct usb_interface usb_ifaces[] = {{
        .num_altsetting = 1,
        .altsetting = &dfu_iface,
}};

static const struct usb_config_descriptor usb_config = {
        .bLength = USB_DT_CONFIGURATION_SIZE,
        .bDescriptorType = USB_DT_CONFIGURATION,
        .wTotalLength = 0,
        .bNumInterfaces = 1,
        .bConfigurationValue = 1,
        .iConfiguration = 0,
        .bmAttributes = 0xC0,  // Bit 6 -> self powered
        .bMaxPower = 5,        // Will consume 10ma from USB (a guess)
	.interface = usb_ifaces,
};

static const char *usb_strings[] = {
        "Student Robotics",
        "Servo Board v4",
        (const char *)SERIALNUM_BOOTLOADER_LOC,
	"Student Robotics Servo Board v4", // Iface 1
	"Student Robotics Servo Board DFU loader", // IFace 2, DFU
};

static uint8_t usb_data_buffer[128];

static int
handle_read_req(struct usb_setup_data *req, int *len, uint8_t **buf)
{
	int result = USBD_REQ_NOTSUPP; // Will result in a USB stall
	uint16_t *u16ptr;
	uint32_t *u32ptr;

	return result;
}

static int
handle_write_req(struct usb_setup_data *req)
{
	return USBD_REQ_NOTSUPP; // Will result in a USB stall
}

static int
control(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
	uint16_t *len,
	void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	// Only respond to a single device request, number 64. USB spec 3.1
	// Section 9.3.1 allows us to define additional requests, and Table 9.5
	// identifies all reserved requests. So, pick 64, it could be any.
	if (req->bRequest != 64)
		return USBD_REQ_NEXT_CALLBACK;

	// Data and length are in *buf and *len respectively. Output occurs by
	// modifying what those point at.

	if (req->bmRequestType & USB_REQ_TYPE_IN) { // i.e., input to host
		return handle_read_req(req, len, buf);
	} else {
		return handle_write_req(req);
	}
}

static int
iface_control(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
	uint16_t *len,
	void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{

	// For standard requests, handle only set_iface, with no alternative
	// ifaces.
	if (req->bmRequestType == (USB_REQ_TYPE_STANDARD|USB_REQ_TYPE_INTERFACE)
			&& req->bRequest == USB_REQ_SET_INTERFACE
			&& req->wValue == 0) {
		// Two ifaces: this one and DFU.
		if (req->wIndex == 0) {
			// Do a special dance; but later.
			return USBD_REQ_HANDLED;
		}
	}

	// Otherwise, we might be getting DFU requests
	if ((req->bmRequestType & (USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT))
		== (USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE))
	{
		switch (req->bRequest) {
		case DFU_GETSTATUS:
			*len = 6;
			(*buf)[0] = STATE_APP_IDLE;
			(*buf)[1] = 100; // ms
			(*buf)[2] = 0;
			(*buf)[3] = 0;
			(*buf)[4] = STATE_APP_IDLE;
			(*buf)[5] = 0;
			return USBD_REQ_HANDLED;
		case DFU_DETACH:
			re_enter_bootloader = true;
			return USBD_REQ_HANDLED;
		}
	}

	return USBD_REQ_NOTSUPP;
}

static void
set_config_cb(usbd_device *usbd_dev, uint16_t wValue)
{

  // We want to handle device requests sent to the default endpoint: match the
  // device request type (0), with zero recpient address. Use type + recipient
  // mask to filter requests.
  usbd_register_control_callback(usbd_dev, USB_REQ_TYPE_DEVICE,
		  USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
		  control);

  // Additionally, register our own SR-interface callback. This is simply to
  // handle SET_INTERFACE, which libopencm3 doesn't do. Filter options are to
  // match standard request, to the interface recipient.
  usbd_register_control_callback(usbd_dev,
		  USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE,
		  USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
		  iface_control);

  // Use the same function to catch initial DFU requests. These are class
  // commands.
  usbd_register_control_callback(usbd_dev,
		  USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
		  USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
		  iface_control);
}

void
usb_init()
{

  gpio_clear(GPIOA, GPIO8);
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);

  usbd_dev = usbd_init(&stm32f103_usb_driver, &usb_descr, &usb_config,
		 usb_strings, 5, usb_data_buffer, sizeof(usb_data_buffer));

  usbd_register_set_config_callback(usbd_dev, set_config_cb);

  gpio_set(GPIOA, GPIO8);
}

void
usb_deinit()
{

  // Gate USB; this will cause a reset for us and the  host.
  gpio_clear(GPIOA, GPIO8);

  // Do nothing for a few ms, then poll a few times to ensure that the driver
  // has reset itself
  delay(20);
  usbd_poll(usbd_dev);
  usbd_poll(usbd_dev);
  usbd_poll(usbd_dev);
  usbd_poll(usbd_dev);

  // That should be enough
  return;
}

void
usb_poll()
{
	usbd_poll(usbd_dev);
}
