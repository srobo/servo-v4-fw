#!/usr/bin/env python

from __future__ import print_function

import sys
import struct
import argparse
from usb1 import * # In lieu of packages for pyusb, I `pip install libusb1`'d.
import time

parser = argparse.ArgumentParser()
parser.add_argument("reqtype", help="read or write")
parser.add_argument("reqname", help="name of request to make")
parser.add_argument("argument", help="For writes, an output argument", nargs='?', default=-1, type=int)
args = parser.parse_args()

write_ids = {
        'output0' : 0,
        'output1' : 1,
        'output2' : 2,
        'output3' : 3,
        'output4' : 4,
        'output5' : 5,
        'runled' : 6,
        'errorled' : 7,
        }

read_ids = {
        'output0' : 0,
        'output1' : 1,
        'output2' : 2,
        'output3' : 3,
        'output4' : 4,
        'output5' : 5,
        '5vrail' : 6,
        'batt' : 7,
        'button' : 8,
        'fwver' : 9,
        }

if args.reqtype == "read":
    req_map = read_ids
    is_read = True
elif args.reqtype == "write":
    req_map = write_ids
    is_read = False
    if len(sys.argv) != 4:
        print("You need to pass an argument to write", file=sys.stderr)
        sys.exit(1)
else:
    print("Unrecognized request type", file=sys.stderr)
    sys.exit(1)

if args.reqname not in req_map:
    print("\"{0}\" is not a valid request for {1}".format(args.reqname, args.reqtype), file=sys.stderr)
    sys.exit(1)

req_id = req_map[args.reqname]

###############################################################################

ctx = USBContext()
dev = ctx.getByVendorIDAndProductID(0x1bda, 0x11)

if dev == None:
    print("Could not find servo board attached", file=sys.stderr)
    sys.exit(1)

handle = dev.open()

if handle == None:
    print("Could not open servo board", file=sys.stderr)
    sys.exit(1)

# Always command the board to init the servo stuff
handle.controlWrite(0, 64, 0, 12)

if is_read:
    ret = handle.controlRead(0x80, 64, 0, req_id, 8)
    if len(ret) == 4:
        a, = struct.unpack("i", ret)
        print("{0}".format(a))
    elif len(ret) == 8:
        a, b = struct.unpack("ii", ret)
        print("{0} {1}".format(a, b))
    else:
        print("Short read (or otherwise), board returned {0} bytes".format(len(ret)), file=sys.stderr)
        sys.exit(1)
else:
    handle.controlWrite(0, 64, args.argument, req_id, "")

#def set_servo_pos(idx, pos):
    #handle.controlWrite(0, 64, pos, idx, "")

#import math

#while True:
    #for step in range(100):
        #for i in range(12):
            #p = int(100*math.sin(step*((2*math.pi)/100)  + i*(2*math.pi/16)))
            #print p
            #set_servo_pos(i, p)
        #time.sleep(0.01)
