#ifndef COMMS_INTERFACE_H_
#define COMMS_INTERFACE_H_

#include <string>
#include <climits>

namespace navigator
{

    class MaestroCommsInterface
    {
      public:

        //static CommsInterface* createCommsInterface( const std::string& portName, unsigned int baudRate, std::string* error=NULL);
        virtual ~MaestroCommsInterface();

        virtual bool connectionOpen() const=0;
        virtual bool connect()=0;
        static unsigned short getMinChannelValue() { return minChannelValue_; } 
        static unsigned short getMaxChannelValue() { return maxChannelValue_; }
        
        bool setTarget(unsigned char channelNumber, unsigned short target);
        bool setTargetMiniSCC(unsigned char channelNumber, unsigned char normalizedTarget);
        bool setMaxSpeed(unsigned char channelNumber, unsigned short speed);
        bool setMaxAcceleration(unsigned char channelNumber, unsigned short acceleration);

        bool getPosition(unsigned char channelNumber, unsigned short& position);
        bool getMovingState(unsigned char channelNumber, bool& servosMoving);
        bool getError(unsigned short& error);

        bool goHome();

      protected:
        MaestroCommsInterface();
        
      private:

        enum Commands
        {
            COMMAND_SET_TARGET = 0x84, // 3 data bytes
            COMMAND_SET_SPEED = 0x87, // 3 data bytes
            COMMAND_SET_ACCELERATION = 0x89, // 3 data bytes
            COMMAND_GET_POSITION = 0x90, // 0 data
            COMMAND_GET_MOVING_STATE = 0x93, // 0 data
            COMMAND_GET_ERRORS = 0xA1, // 0 data
            COMMAND_GO_HOME = 0xA2, // 0 data
            COMMAND_STOP_SCRIPT = 0xA4, // 0 data
            COMMAND_RESTART_SCRIPT_AT_SUBROUTINE = 0xA7, // 1 data bytes
            COMMAND_RESTART_SCRIPT_AT_SUBROUTINE_WITH_PARAMETER = 0xA8, // 3 data bytes
            COMMAND_GET_SCRIPT_STATUS = 0xAE, // 0 data
            COMMAND_MINI_SSC = 0xFF, // (2 data bytes)
        };

        static const unsigned short CLEAR = 0x7f;
        static const unsigned short minChannelValue_ = 3968;
        static const unsigned short maxChannelValue_ = 8000;
        virtual bool writeBytes(unsigned char requestType, unsigned char request, unsigned char* const data, unsigned int numBytes) = 0;
        virtual bool readBytes(unsigned char* data, unsigned int numBytes) = 0;

        std::string errorMessage_;
    };
}

#endif // COMMS_INTERFACE_H_
