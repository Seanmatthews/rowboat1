#include "pololu_maestro/maestro_comms_interface.h"

namespace navigator
{
    
    //CommsInterface* CommsInterface::createCommsInterface(const std::string& portName, unsigned int baudRate, std::string* error )
    //{
    //    // TODO
    //    return new CommsInterface();
    //}

    MaestroCommsInterface::MaestroCommsInterface()
        : errorMessage_()
    {
    }
    
    MaestroCommsInterface::~MaestroCommsInterface() {}

    bool MaestroCommsInterface::setTarget(unsigned char channelNumber, unsigned short target)
    {
        unsigned char const data[] = {COMMAND_SET_TARGET, channelNumber, target & CLEAR, (target >> 7) & CLEAR};
        return writeBytes(data, 4);
    }

    bool MaestroCommsInterface::setTargetMiniSCC(unsigned char channelNumber, unsigned char normalizedTarget)
    {
        unsigned char const data[] = {COMMAND_MINI_SSC, channelNumber, normalizedTarget};
        return writeBytes(data, 2);
    }
        
    bool MaestroCommsInterface::setMaxSpeed(unsigned char channelNumber, unsigned short speed)
    {
        unsigned char const data[] = {COMMAND_SET_SPEED, channelNumber, speed & CLEAR, (speed >> 7) & CLEAR};
        return writeBytes(data, 4);
    }

    bool MaestroCommsInterface::setMaxAcceleration(unsigned char channelNumber, unsigned short acceleration)
    {
        unsigned char const data[] = {COMMAND_SET_ACCELERATION,
                                      channelNumber,
                                      acceleration & CLEAR,
                                      (acceleration >> 7) & CLEAR};
        return writeBytes(data, 4);
    }

    bool MaestroCommsInterface::goHome()
    {
        unsigned char const data[] = {COMMAND_GO_HOME};
        return writeBytes(data, 1);
    }

    // DANGER: blocking?
    bool MaestroCommsInterface::getPosition(unsigned char channelNumber, unsigned short& position)
    {
        unsigned char const data[] = {COMMAND_GET_POSITION, channelNumber};
        if (writeBytes(data, 2))
        {
            unsigned char response[2] = {0x00, 0x00};
            if(readBytes(response, 2))
            {
                position = response[1] | response[0];
                return true;
            }
        }
        return false;
    }

    bool MaestroCommsInterface::getMovingState(unsigned char channelNumber, bool& servosMoving)
    {
        // TODO
        return false;
    }

    bool MaestroCommsInterface::getError(unsigned short& error)
    {
        // TODO
        return false;
    }

    
}
