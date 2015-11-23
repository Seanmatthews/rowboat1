#ifndef MAESTRO_USB_H_
#define MAESTRO_USB_H_

#include <libusb-1.0/libusb.h>
#include <vector>
#include "pololu_maestro/maestro_comms_interface.h"


namespace navigator
{
    class MaestroUsb : public MaestroCommsInterface
    {
      public:
        MaestroUsb();
        ~MaestroUsb();
        bool connect();
        bool connectionOpen() const;

        static const unsigned short PRODUCT_ID = 0x1ffb;
            
      private:
        bool isError(int code);
        bool writeBytes(unsigned char requestType, unsigned char request, unsigned short value, unsigned short index);
        bool writeBytes(unsigned char requestType, unsigned char request, unsigned short value,
                        unsigned short index, unsigned char* data, unsigned short length);
        bool writeBytes(unsigned char requestType, unsigned char request, unsigned char* const data, unsigned int numBytes);
        bool readBytes(unsigned char* data, unsigned int numBytes);
        bool findMaestro();
        bool claimDeviceInterfaces(libusb_device_handle* handle);
        
        libusb_context *context_;
        libusb_device *device_;
        libusb_device_handle *deviceHandle_;
        unsigned short servoCount_;
        std::vector<unsigned short> productList_;
        
    };
}

#endif // MAESTRO_USB_H_
