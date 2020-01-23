Servo Board v4 Firmware
=======================

The Servo Board can be used to control up to 12 RC servos. Many devices are available that can be controlled as servos, such as RC motor speed controllers, and these can also be used with the board.

Other devices that are controllable using PWM outputs can also be attached to the board outputs.

USB Interface
-------------

The Vendor ID is `1bda` (University of Southampton) and the product ID
is `0011`.

The Servo Board is controlled over USB by sending requests to the
control endpoint.

```python
ctrl_transfer(
    0x00,
    64,
    wValue=req_val,
    wIndex=command.code,
    data_or_wLength=req_data,
)
```

| Parameter     | Value |
|---------------|-------|
| bmRequestType | 0x00  |
| bRequest      | 64    |

There are a list of ids defined in the firmware of the servo board that
will let you read and write values to it.

It is recommended to read the source to further understand how to
control this device.

It should also be noted that as the control endpoint `0x00` is used to
send data to this device, it is not actually compliant with the USB 2.0
specification.

udev Rule
---------

If you are connecting the Servo Board to a Linux computer with udev, the
following rule can be added in order to access the Servo Board interface
without root privileges:

`SUBSYSTEM=="usb", ATTRS{idVendor}=="1bda", ATTRS{idProduct}=="0011", GROUP="plugdev", MODE="0666"`

It should be noted that `plugdev` can be changed to any Unix group of
your preference.

Designs
-------

You can access the schematics and source code of the hardware in the following places.
-   [Full Schematics](https://www.studentrobotics.org/resources/kit/servo-schematic.pdf)
-   [Student Facing Docs](https://studentrobotics.org/docs/kit/servo_board)
-   [Hardware designs](https://github.com/srobo/servo-v4-hw)
