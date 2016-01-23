#ifndef MAESTRO_USB_H_
#define MAESTRO_USB_H_

#include <libusb-1.0/libusb.h>
#include <vector>
#include "pololu_maestro/maestro_comms_interface.h"

/**
 * Bitmap for libusb bmRequestType
 *
 * D7 Data Phase Transfer Direction
 *  0 = Host to Device
 *  1 = Device to Host
 * D6..5 Type
 *  0 = Standard
 *  1 = Class
 *  2 = Vendor
 *  3 = Reserved
 * D4..0 Recipient
 *  0 = Device
 *  1 = Interface
 *  2 = Endpoint
 *  3 = Other
 *  4..31 = Reserved
 *
 * This gives us:
 *  0xC0 = Read from vendor device
 *  0x40 = Write to vendor device
 *  0x80 = Standard read from device
 */
namespace navigator
{
    class MaestroUsb : public MaestroCommsInterface
    {
      public:
        MaestroUsb();
        ~MaestroUsb();
        bool connect();
        bool connectionOpen() const;

        static const unsigned short VENDOR_ID = 0x1ffb;
            
      private:
        bool isError(int code);
        bool writeBytes(unsigned char request, unsigned short value, unsigned short index);
        bool writeBytes(unsigned char request, unsigned short value, unsigned short index,
                        unsigned char* data, unsigned short length);
        int readBytes(unsigned char request, unsigned char* data, unsigned short length);
        int standardReadBytes(unsigned char request, unsigned char* data, unsigned short length);
        bool findMaestro();
        bool claimDeviceInterfaces(libusb_device_handle* handle);
        unsigned short convertTargetToMicros(char target);
        
        libusb_context *context_;
        libusb_device *device_;
        libusb_device_handle *deviceHandle_;
        //std::vector<unsigned short> productList_;
        
        static const unsigned char VENDOR_READ = 0xC0;
        static const unsigned char VENDOR_WRITE = 0x40;

        // For getting firmware version
        static const unsigned char STANDARD_READ = 0x80;
    };
}

#endif // MAESTRO_USB_H_
