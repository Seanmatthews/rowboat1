#include <ros/ros.h>
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
        if (init = LIBUSB_SUCCESS) {
            ROS_ERROR_STREAM("Could not initialize a USB device");
            return false;
        }

        // Set suggested debug verbosity level
        libusb_set_debug(context_, 3);

        if (!findMaestro()) {
            ROS_ERROR_STREAM("Could not find a Maestro device with device handle " << deviceHandle_);
            return false;
        }

        // Claim first interface of device
        int detach = libusb_detach_kernel_driver(deviceHandle_, 0);
        if (detach != 0)
        {
            ROS_INFO_STREAM("Could not detach kernel driver. Error " << detach);
            return false;
        }
        int claim = libusb_claim_interface(deviceHandle_, 0);
        if (claim != 0) {
            ROS_ERROR_STREAM("Could not claim interface " << deviceHandle_ );
            return false;
        }
        
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

    bool MaestroUsb::writeBytes(unsigned char request, unsigned char* data, unsigned int length)
    {
        return writeBytes(request, 0, 0, data, length);
    }

    bool MaestroUsb::writeBytes(unsigned char request, unsigned short value, unsigned short index)
    {
        return writeBytes(request, value, index, (unsigned char*)0, (unsigned short)0);
    }

    // The function is not required by the interface, but serves to
    // interface the libusb transfer control function directly.
    bool MaestroUsb::writeBytes(unsigned char request, unsigned short value,
                                unsigned short index, unsigned char* const data, unsigned short length)
    {
        int err = libusb_control_transfer(deviceHandle_,
                                          VENDOR_WRITE,
                                          request,
                                          value,
                                          index,
                                          (unsigned char*)data,
                                          length,
                                          (unsigned int)5000);
        return !isError(err);
    }

    bool MaestroUsb::readBytes(unsigned char request, unsigned char* data, unsigned int length)
    {
        return readBytes(request, 0, 0, data, length);
    }

    bool MaestroUsb::readBytes(unsigned char request, unsigned short value, unsigned short index,
                               unsigned char* data, unsigned short length)
    {
        unsigned int bytesRead = libusb_control_transfer(deviceHandle_,
                                                         VENDOR_READ,
                                                         request,
                                                         value,
                                                         index,
                                                         data,
                                                         length,
                                                         (unsigned int)5000);
        return true;
    }

    // Finds all connected USB devices, then picks out the first Maestro
    bool MaestroUsb::findMaestro()
    {
        for (std::vector<unsigned short>::iterator it = productList_.begin(); it != productList_.end(); it++)
        {
            ROS_INFO_STREAM("Attempting to open maestro with vendorID:productID " << VENDOR_ID << ":" << *it);
            deviceHandle_ = libusb_open_device_with_vid_pid(context_, VENDOR_ID, *it);
            ROS_INFO_STREAM("device handle " << deviceHandle_);
            if (deviceHandle_ != NULL)
            {
                switch(*it)
                {
                case 0x89: numChannels_ = 6;
                case 0x8A: numChannels_ = 12;
                case 0x8B: numChannels_ = 18;
                case 0x8C: numChannels_ = 24;
                default: numChannels_ = 0;
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
