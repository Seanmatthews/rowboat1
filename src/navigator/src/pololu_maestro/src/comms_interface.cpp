#include "comms_interface.h"

namespace navigator
{
    CommsInterface* CommsInterface::createCommsInterface(const std::string& portName )
    {
     
    }

    CommsInterface::CommsInterface()
        : mErrorMessage()
    {
    }
    
    ~CommsInterface::CommsInterface() {}

    bool CommsInterface::setTarget(unsigned char channelNumber, unsigned short target)
    {
        unsigned char const data[] = {Commands::COMMAND_SET_TARGET, channelNumber, target & CLEAR, (target >> 7) & CLEAR};
        return writeBytes(data, 4);
    }

    bool CommsInterface::setTargetMiniSCC(unsigned char channelNumber, unsigned char normalizedTarget)
    {
        unsigned char const data[] = {Commands::COMMAND_MINI_SSC, channelNumber, normalizedTarget};
        return writeBytes(data, 2);
    }
        
    bool CommsInterface::setMaxSpeed(unsigned char channelNumber, unsigned short speed)
    {
        unsigned char const data[] = {Commands::COMMAND_SET_SPEED, channelNumber, speed & CLEAR, (speed >> 7) & CLEAR};
        return writeBytes(data, 4);
    }

    bool CommsInterface::setMaxAcceleration(unsigned char channelNumber, unsigned short acceleration)
    {
        unsigned char const data[] = {Commands::COMMAND_SET_ACCELERATION,
                                      channelNumber,
                                      acceleration & CLEAR,
                                      (acceleration >> 7) & CLEAR};
        return writeBytes(data, 4);
    }

    bool CommsInterface::goHome(unsigned char channelNumber)
    {
        unsigned char const data[] = {Commands::COMMAND_GO_HOME};
        return writeBytes(data, 1);
    }

    // DANGER: blocking?
    bool CommsInterface::getPosition(unsigned char channelNumber, unsigned short& position)
    {
        unsigned char const data[] = {Commands::COMMAND_GET_POSITION, channelNumber};
        if (writeBytes(data, 2))
        {
            unsigned short response;
            if(readBytes(&response, 2))
            {
                position = response >> 8 | response << 8;
                return true;
            }
        }
        return false;
    }

    bool CommsInterface::getMovingState(unsigned char channelNumber, bool& servosMoving)
    {
        // TODO
        return false;
    }

    bool CommsInterface::getError(unsigned short& error)
    {
        // TODO
        return false;
    }

    
}
