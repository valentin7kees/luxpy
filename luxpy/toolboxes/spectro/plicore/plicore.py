# -*- coding: utf-8 -*-
"""
The python script reads raw detector data from plicore EVX spectral sensor
evaluation kit. Addtionally the internal temperature sensors on the silicon
photo detector are enabled to read out.
The photo-detector exposure time value has to be set to identically for both
detectors!
"""

import sys                      # check platform specifics
                                # (https://docs.python.org/3/library/sys.html)
import array as arr             # python numeric arrays
                                # (https://docs.python.org/3/library/array.html)
import numpy as np              # numpy scientific computing library
                                # (https://numpy.org/doc/1.22/reference/index.html#reference)
import matplotlib.pyplot as plt # plot library to draw diagrams
                                # (https://matplotlib.org/stable/api/_as_gen/matplotlib.pyplot.html)
import usb                      # pyusb library
                                # (https://github.com/pyusb/pyusb/blob/master/docs/tutorial.rst)





def m_i2c_write_register(args):
    """Writes a byte value into a register on device slave address.

    arguments order:
        m_i2c_write_register(device, endpoint_out, endpoint_in, slave_address, regnum, value)

        device        -- i2c device address
        endpoint_out  -- endpoint out address
        endpoint_in   -- endpoint in address
        slave_address -- slave device address
        regnum        -- register address
        value         -- register value
    """
    device = args[0]
    endpoint_out = args[1]
    endpoint_in = args[2]
    slave_address = args[3]
    regnum = args[4]
    value = args[5]

    cmdblock_i2c = arr.array('B', [
        0x40,
        3,
        0,
        0,
        ((slave_address >> 8) & 0xff),
        (slave_address & 0xff),
        ((regnum >> 8) & 0xff),
        (regnum & 0xff),
        ((value >> 8) & 0xff),
        (value & 0xff),
        ])

    retval = device.write(endpoint_out.bEndpointAddress, bytes(cmdblock_i2c))
    if retval != len(cmdblock_i2c):
        print("Error in Message Transfer to Endpoint")

    retval = device.read(endpoint_in.bEndpointAddress, 10)





def m_i2c_read_register(args) -> int:
    """ Reads byte value from slave device register.

    arguments order:
        m_i2c_read_register(device, endpoint_out, endpoint_in, slave_address, regnum, value)

        device        -- i2c device address
        endpoint_out  -- endpoint out address
        endpoint_in   -- endpoint in address
        slave_address -- slave device address
        regnum        -- register address

        retval        -- register value read from slave address
    """
    device = args[0]
    endpoint_out = args[1]
    endpoint_in = args[2]
    slave_address = args[3]
    regnum = args[4]

    cmdblock_i2c = arr.array('B', [
        0x40,
        3,
        0,
        1,
        ((slave_address >> 8) & 0xff),
        (slave_address & 0xff),
        ((regnum >> 8) & 0xff),
        (regnum & 0xff),
        0,
        0,
    ])

    retval = device.write(endpoint_out.bEndpointAddress, bytes(cmdblock_i2c))
    if retval != len(cmdblock_i2c):
        print("Error in Message Transfer to Endpoint {}", endpoint_out.bEndpointAddress)

    retval = device.read(endpoint_in.bEndpointAddress, 10)

    return retval





def m_read_detector(args) -> arr.array:
    """Reads a single detector full frame of 1024 pixels.

    arguments order:
        m_read_detector(device, endpoint_out, endpoint_in, detector, exp_us, preclrpix)

        device        -- i2c device address
        endpoint_out  -- endpoint out address
        endpoint_in   -- endpoint in address
        detector      -- detector address
        exp_us        -- exposure time in µs
        preclrpix     -- clear pixel value
    """
    device = args[0]
    endpoint_out = args[1]
    endpoint_in = args[2]
    detector = args[3]
    exp_us = args[4]
    preclrpix = args[5]

    cmdblock_i2c = arr.array('B', [
        detector,
        3,
        ((exp_us >> 24) & 0xff),
        ((exp_us >> 16) & 0xff),
        ((exp_us >>  8) & 0xff),
        (exp_us & 0xff),
        ((preclrpix >> 8) & 0xff),
        (preclrpix & 0xff),
        4,
        0,
    ])

    retval = device.write(endpoint_out.bEndpointAddress, bytes(cmdblock_i2c))

    if retval != len(cmdblock_i2c):
        print("Error in Message Transfer to Endpoint")

    frame = device.read(endpoint_in.bEndpointAddress, 2048, 1500)

    return frame


def dvc_open():
    # find usb device
    VENDOR_ID = 0x0483
    PRODUCT_ID = 0xa376
    DEV = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)

    if DEV is None:
        raise ValueError("Device not found")

    # set the active configuration. With no arguments, the first
    # configuration will be the active one
    try:
        DEV.set_configuration()
    except usb.USBError as exc:
        print("Cant set configuration. Error : {}".format(exc.strerror))

    # claim device interface (first interface - only one configured - default)
    try:
        usb.util.claim_interface(DEV, 0)
    except usb.USBError as exc:
        print("Cant claim interface. Error : {}".format(exc.strerror))

    print("USB device acquired, VID={:#06x}, PID={:#06x}".format(VENDOR_ID, PRODUCT_ID))

    # get an endpoint instance
    CFG = DEV.get_active_configuration()

    # print endpoint addresses for device configuration
    for CFG in DEV:
        for i in CFG:
            for e in i:
                EP_DIR = ""
                if usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN:
                    EP_DIR = "IN"
                if usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT:
                    EP_DIR = "OUT"
                print("  ENDPOINT: {:#04x} - {}".format(e.bEndpointAddress, EP_DIR))

    # get interface from configuration index
    INTF = CFG[(0, 0)]

    # get first ENDPOINT_OUT
    EP_OUT = usb.util.find_descriptor(
        INTF,
        # match the first OUT endpoint
        custom_match = \
        lambda e: \
            usb.util.endpoint_direction(e.bEndpointAddress) == \
            usb.util.ENDPOINT_OUT)

    assert EP_OUT is not None

    # get first ENDPOINT_IN
    EP_IN = usb.util.find_descriptor(
        INTF,
        # match the first OUT endpoint
        custom_match = lambda e: \
            usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN)

    assert EP_IN is not None

    return DEV, EP_OUT, EP_IN

# initialize sequence
# m_i2c_write_register([DEV, EP_OUT, EP_IN, 0x00, 0x06, 0x00])
#
# m_i2c_write_register([DEV, EP_OUT, EP_IN, 0x15, 0xd0, 0x4a])
# m_i2c_write_register([DEV, EP_OUT, EP_IN, 0x15, 0xd1, 0x66])
# m_i2c_write_register([DEV, EP_OUT, EP_IN, 0x15, 0xd6, 0x03])
# m_i2c_write_register([DEV, EP_OUT, EP_IN, 0x15, 0x02, 0x99])
# m_i2c_write_register([DEV, EP_OUT, EP_IN, 0x15, 0x00, 0x1c])
#
# m_i2c_write_register([DEV, EP_OUT, EP_IN, 0x14, 0xd0, 0x4a])
# m_i2c_write_register([DEV, EP_OUT, EP_IN, 0x14, 0xd1, 0x66])
# m_i2c_write_register([DEV, EP_OUT, EP_IN, 0x14, 0xd6, 0x03])
# m_i2c_write_register([DEV, EP_OUT, EP_IN, 0x14, 0x02, 0x99])
# m_i2c_write_register([DEV, EP_OUT, EP_IN, 0x14, 0x00, 0x1c])
#
# m_i2c_write_register([DEV, EP_OUT, EP_IN, 0x14, 0xa4, 0x2c])
# m_i2c_write_register([DEV, EP_OUT, EP_IN, 0x15, 0xa4, 0x2c])


def get_spd(exposure_time_mu=500, clear_pixel_setting=272,
            detector_a_slave_address=0x41, detector_b_slave_address=0x42):

    DEV, EP_OUT, EP_IN = dvc_open()
    values_a = np.frombuffer(m_read_detector(
        [DEV, EP_OUT, EP_IN, detector_a_slave_address, exposure_time_mu, clear_pixel_setting]), dtype=np.uint16)
    values_b = np.frombuffer(m_read_detector(
        [DEV, EP_OUT, EP_IN, detector_b_slave_address, exposure_time_mu, clear_pixel_setting]), dtype=np.uint16)

    # concat arrays to plot both detectors in one figure
    # values = np.concatenate((valuesA, valuesB), axis=0)

    # free all usb resources
    usb.util.dispose_resources(DEV)

    return values_a, values_b






