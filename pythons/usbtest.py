import usb.core
import usb.util

def makedev():
    # find our device
    #dev = usb.core.find(idVendor=0xa720, idProduct=0xf803)
    dev = usb.core.find(idVendor=0x0483, idProduct=0x575a)

    # was it found?
    if dev is None:
        raise ValueError('Device not found')

    if dev.is_kernel_driver_active(0):
        try:
            dev.detach_kernel_driver(0)
        except usb.core.USBError as e:
            sys.exit("Could not detatch kernel driver from interface({0}): {1}".format(i, str(e)))       

    # set the active configuration. With no arguments, the first
    # configuration will be the active one
    dev.set_configuration()

    # get an endpoint instance
    cfg = dev.get_active_configuration()
    intf = cfg[(0,0)]

    ep = usb.util.find_descriptor(
        intf,
        # match the first OUT endpoint
        custom_match = \
        lambda e: \
            usb.util.endpoint_direction(e.bEndpointAddress) == \
            usb.util.ENDPOINT_OUT)



    ep1 = usb.util.find_descriptor(
        intf,
        # match the first OUT endpoint
        custom_match = \
        lambda e: \
            usb.util.endpoint_direction(e.bEndpointAddress) == \
            usb.util.ENDPOINT_IN)


    assert ep is not None
    assert ep1 is not None

    return (ep, ep1)

if (__name__ == '__main__'):
    ep, ep1 = makedev()
    
    if (1):# write the data
        ep.write('test')
        a = ep1.read(100)
        print('end')
        print(a)
