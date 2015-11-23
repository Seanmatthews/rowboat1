#include "pololu_maestro/maestro_usb.h"


namespace navigator
{
    MaestroUsb::MaestroUsb()
    {
        // Add all Maestros to product list
        productList_.push_back(0x89); // 6
        productList_.push_back(0x8A); // 12
        productList_.push_back(0x8B); // 18
        productList_.push_back(0x8C); // 24
    }

    MaestroUsb::~MaestroUsb()
    {
        // Release claimed interfaces & close session
        libusb_release_interface(deviceHandle_, 0);
        libusb_exit(context_);
    }

    // Init, find, and claim Maestro USB interface
    bool MaestroUsb::connect()
    {
        int init = libusb_init(&context_);
        if (init = LIBUSB_SUCCESS) return false;

        // Set suggested debug verbosity level
        libusb_set_debug(context_, 3);

        if (!findMaestro()) return false;

        // Claim first interface of device
        int detach = libusb_detach_kernel_driver(deviceHandle_, 0);
        if (detach != 0) return false;
        int claim = libusb_claim_interface(deviceHandle_, 0);
        if (claim != 0) return false;
        
        return true;
    }

    bool MaestroUsb::connectionOpen() const
    {
        return deviceHandle_ != NULL;
    }

    bool MaestroUsb::isError(int code)
    {
        return false;
    }

    bool MaestroUsb::writeBytes(unsigned char requestType, unsigned char request, unsigned char* const data, unsigned int numBytes)
    {
        int err = writeBytes(requestType, request, 0, 0, data, numBytes);
        return isError(err);
    }

    bool MaestroUsb::writeBytes(unsigned char requestType, unsigned char request, unsigned short value, unsigned short index)
    {
        int ret = writeBytes(requestType, request, value, index, (unsigned char*)0, (unsigned short)0);
        return isError(ret);
    }

    // The function is not required by the interface, but serves to
    // interface the libusb transfer control function directly.
    bool MaestroUsb::writeBytes(unsigned char requestType, unsigned char request, unsigned short value,
                                unsigned short index, unsigned char* const data, unsigned short length)
    {
        int err = libusb_control_transfer(deviceHandle_,
                                          requestType,
                                          request,
                                          value,
                                          index,
                                          (unsigned char*)data,
                                          length,
                                          (unsigned int)5000);
        return !isError(err);
    }

    bool MaestroUsb::readBytes(unsigned char* data, unsigned int numBytes)
    {
        return true;
    }

    // Finds all connected USB devices, then picks out the first Maestro
    bool MaestroUsb::findMaestro()
    {
        for (std::vector<unsigned short>::iterator it = productList_.begin(); it != productList_.end(); it++)
        {
            deviceHandle_ = libusb_open_device_with_vid_pid(context_, *it, PRODUCT_ID);
            if (deviceHandle_ != NULL)
            {
                switch(PRODUCT_ID)
                {
                case 0x89: servoCount_ = 6;
                case 0x8A: servoCount_ = 12;
                case 0x8B: servoCount_ = 18;
                case 0x8C: servoCount_ = 24;
                default: servoCount_ = 0;
                }
                break;
            }
        }
        return deviceHandle_ != NULL;
    }

    // Finds all interfaces for a device and claims them
    bool MaestroUsb::claimDeviceInterfaces(libusb_device_handle* handle)
    {
        // Not implemented
        return false;
    };
}
