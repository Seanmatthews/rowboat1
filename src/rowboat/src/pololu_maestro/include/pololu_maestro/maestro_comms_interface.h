#ifndef COMMS_INTERFACE_H_
#define COMMS_INTERFACE_H_

#include <string>
#include <climits>
#include <vector>

namespace navigator
{

    struct ServoStatus
    {
        // The position in units of quarter-microseconds.
        unsigned short position;

        // The target position in units of quarter-microseconds.
        unsigned short target;

        // The speed limit.  Units depends on your settings.
        unsigned short speed;

        // The acceleration limit.  Units depend on your settings.
        unsigned char acceleration;
    };

    class MaestroCommsInterface
    {
      public:
        //static CommsInterface* createCommsInterface( const std::string& portName, unsigned int baudRate, std::string* error=NULL);
        virtual ~MaestroCommsInterface();

        virtual bool connectionOpen() const=0;
        virtual bool connect()=0;
        static unsigned short getMinChannelValue() { return minChannelValue_; } 
        static unsigned short getMaxChannelValue() { return maxChannelValue_; }

        // Movement settings
        bool setTarget(unsigned char channelNumber, signed char target);
        bool setTargetMiniSCC(unsigned char channelNumber, unsigned char normalizedTarget);
        bool setMaxSpeed(unsigned char channelNumber, unsigned short speed);
        bool setMaxAcceleration(unsigned char channelNumber, unsigned short acceleration);
        bool goHome();

        // Admin settings & info
        std::string getFirmwareVersion();
        bool reinitialize(unsigned short waitSeconds);
        bool clearErrors();
        bool setPWM(unsigned short dutyCycle, unsigned short period);
        int getNumChannels();
        std::vector<ServoStatus> getAllPWMInfo();

      protected:
        MaestroCommsInterface();
        unsigned char numChannels_;
        std::map<unsigned short, unsigned char> productList_;
        
      private:
        virtual bool writeBytes(unsigned char request, unsigned short value, unsigned short index) = 0;
        virtual int readBytes(unsigned char request, unsigned char* data, unsigned short length) = 0;
        virtual int standardReadBytes(unsigned char request, unsigned char* data, unsigned short length) = 0;
        unsigned short convertTargetToMicros(signed char target);
        
        enum Commands
        {
            COMMAND_SET_TARGET = 0x85, // 3 data bytes
            COMMAND_SET_SPEED = 0x84, // this is command set variable
            COMMAND_SET_ACCELERATION = 0x84, // this is command set variable
            COMMAND_GO_HOME = 0xA2, // 0 data
            COMMAND_MINI_SSC = 0xFF, // (2 data bytes)
            COMMAND_SET_PWM = 0x8A,
            COMMAND_CLEAR_ERRORS = 0x86,
            COMMAND_REINITIALIZE = 0x90,
            REQUEST_GET_SERVO_SETTINGS = 0x87,
            REQUEST_GET_FIRMWARE = 0x06,
            REQUEST_GET_ERRORS = 0xA1, // 0 data
            
            // Additional commands, to be implemented
//            COMMAND_STOP_SCRIPT = 0xA4, // 0 data
//            COMMAND_RESTART_SCRIPT_AT_SUBROUTINE = 0xA7, // 1 data bytes
//            COMMAND_RESTART_SCRIPT_AT_SUBROUTINE_WITH_PARAMETER = 0xA8, // 3 data bytes
//            COMMAND_GET_SCRIPT_STATUS = 0xAE, // 0 data            
//            COMMAND_SET_ALL_TARGETS = 0x9F,
//            POLOLU_SET_ALL_TARGETS = 0xAA,
        };

        static const unsigned short CLEAR = 0x7f;
        static const short minChannelValue_ = 3968;
        static const short maxChannelValue_ = 8000;
        static const short channelHomeValue_ = 6000;
        static const short deadZoneValue_ = 25; // This is specific to T200 thrusters
        std::string errorMessage_;
    };
}

#endif // COMMS_INTERFACE_H_
