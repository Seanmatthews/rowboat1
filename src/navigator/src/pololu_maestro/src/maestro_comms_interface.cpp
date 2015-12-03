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

    // Convert target position [-100, 100] to microseconds.
    // Note: Keep this function simple and optimized for speed.
    unsigned short MaestroCommsInterface::convertTargetToMicros(char target)
    {
        if (target > 100) target = 100;
        if (target < -100) target = -100;

        float scaleFactor = target / 100.0;
        float delta = 0;
        if (target < 0)
        {
            delta = (float)(channelHomeValue_ - deadZoneValue_ - minChannelValue_) * scaleFactor;
        }
        else if (target > 0)
        {
            delta = (float)(maxChannelValue_ - channelHomeValue_ + deadZoneValue_) * scaleFactor;
        }
        return (unsigned short)((float)deadZoneValue_ + delta);
    }
    
    bool MaestroCommsInterface::setTarget(unsigned char channelNumber, unsigned short target)
    {
        unsigned char data[] = {channelNumber, target & CLEAR, (target >> 7) & CLEAR};
        return writeBytes(COMMAND_SET_TARGET, data, 3);
    }

    // Simultaneously set targets on all Maestro channels
    bool MaestroCommsInterface::setAllTargets(std::vector<char> targets)
    {
        // Assert that:
        // 1) we're not overstepping the device's channel limit
        // 2) the user has proviede at least one channel
        // 3) the Maestro supports this message (12, 18, 24 channel versions)
        if (targets.size() <= numChannels_ && targets.size() > 0 && numChannels_ >= 12)
        {
            int dataLength = 3 * targets.size() + 1;
            unsigned char data[dataLength]; 
            data[0] = numChannels_;
            for (int i=0; i<targets.size(); ++i)
            {
                unsigned short targetMicros = convertTargetToMicros(targets[i]);
                data[i*3+1] = i;
                data[i*3+2] = targetMicros & CLEAR;
                data[i*3+3] = (targetMicros >> 7) & CLEAR;
            }
            return writeBytes(COMMAND_SET_ALL_TARGETS, data, dataLength);
        }
        return false;
    }

    bool MaestroCommsInterface::setTargetMiniSCC(unsigned char channelNumber, unsigned char normalizedTarget)
    {
        unsigned char data[] = {channelNumber, normalizedTarget};
        return writeBytes(COMMAND_MINI_SSC, data, 2);
    }
        
    bool MaestroCommsInterface::setMaxSpeed(unsigned char channelNumber, unsigned short speed)
    {
        unsigned char data[] = {channelNumber, speed & CLEAR, (speed >> 7) & CLEAR};
        return writeBytes(COMMAND_SET_SPEED, data, 3);
    }

    bool MaestroCommsInterface::setMaxAcceleration(unsigned char channelNumber, unsigned short acceleration)
    {
        unsigned char data[] = {channelNumber, acceleration & CLEAR, (acceleration >> 7) & CLEAR};
        return writeBytes(COMMAND_SET_ACCELERATION, data, 3);
    }

    bool MaestroCommsInterface::goHome()
    {
        unsigned char data[] = {};
        return writeBytes(COMMAND_GO_HOME, 0, 0);
    }

    // DANGER: blocking?
    /*bool MaestroCommsInterface::getPosition(unsigned char channelNumber, unsigned short& position)
    {
        //unsigned char  data[] = {channelNumber};
        //if (writeBytes(COMMAND_GET_POSITION, data, 2))
        //{
            unsigned char response[2] = {0x00, 0x00};
            if(readBytes(COMMAND_GET_POSITION, response, 2))
            {
                position = response[1] | response[0];
                return true;
            }
        //}
        return false;
    }*/

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
