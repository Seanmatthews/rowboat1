#include <ros/ros.h>
#include "pololu_maestro/maestro_usb.h"


namespace rowboat1
{
    MaestroUsb::MaestroUsb() 
    : MaestroCommsInterface()
    {
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

        // FOR DEBUGGING: Set suggested debug verbosity level
        // libusb_set_debug(context_, 4);

        if (!findMaestro()) {
            ROS_ERROR_STREAM("Could not find a Maestro device with device handle " << deviceHandle_);
            return false;
        }

        // Claim first interface of device
        int detach = libusb_detach_kernel_driver(deviceHandle_, 0);
        //        if (detach != 0)
        //{
        //    ROS_INFO_STREAM("Could not detach kernel driver. Error " << detach);
        //    return false;
        //}
        int claim = libusb_claim_interface(deviceHandle_, 0);
        if (claim != 0) {
            ROS_ERROR_STREAM("Could not claim interface " << deviceHandle_ );
            return false;
        }

        // Send "detect baud rate" signal 0xAA?
        //writeBytes();
        
        return true;
    }

    bool MaestroUsb::connectionOpen() const
    {
        return deviceHandle_ != NULL;
    }

    bool MaestroUsb::isError(int code)
    {
        return code > 0;
    }

//    bool MaestroUsb::writeBytes(unsigned char request, unsigned char* data, unsigned int length)
//    {
//        return writeBytes(request, 0, 0, data, length);
//    }

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
        ROS_INFO_STREAM("[MaestroUsb] value: " << value << " index: " << index);
        ROS_INFO_STREAM("[MaestroUsb] trying to write " << length << " bytes" );
        ROS_INFO_STREAM("[MaestroUsb] err: " << err);
        return !isError(err);
    }

    int MaestroUsb::readBytes(unsigned char request, unsigned char* data, unsigned short length)
    {
        return libusb_control_transfer(deviceHandle_,
                                       VENDOR_READ,
                                       request,
                                       0,
                                       0,
                                       data,
                                       length,
                                       (unsigned int)5000);
    }

    // This read is used only for getting firmware information.
    int MaestroUsb::standardReadBytes(unsigned char request, unsigned char* data, unsigned short length)
    {
        return libusb_control_transfer(deviceHandle_,
                                       STANDARD_READ,
                                       request,
                                       0,
                                       0,
                                       data,
                                       length,
                                       (unsigned int)5000);
    }

    // Finds all connected USB devices, then picks out the first Maestro
    bool MaestroUsb::findMaestro()
    {
        for (std::map<unsigned short, unsigned char>::iterator it = productList_.begin(); it != productList_.end(); it++)
        {
            ROS_INFO_STREAM("Attempting to open maestro with vendorID:productID " << VENDOR_ID << ":" << it->first);
            deviceHandle_ = libusb_open_device_with_vid_pid(context_, VENDOR_ID, it->first);

            if (deviceHandle_ != NULL)
            {
                ROS_INFO_STREAM("device handle " << deviceHandle_);
                numChannels_ = 0;
                if (productList_.find(it->first) != productList_.end())
                {
                    numChannels_ = productList_[it->first];
                }
                break;
            }
        }
        ROS_INFO_STREAM("NUM CHANNELS " << (int)numChannels_ );
        return deviceHandle_ != NULL;
    }

    // Finds all interfaces for a device and claims them
    bool MaestroUsb::claimDeviceInterfaces(libusb_device_handle* handle)
    {
        // Not implemented
        return false;
    };
}
