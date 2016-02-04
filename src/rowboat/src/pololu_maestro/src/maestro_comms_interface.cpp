#include <ros/ros.h>
#include "pololu_maestro/maestro_comms_interface.h"


namespace rowboat1
{
    
    MaestroCommsInterface::MaestroCommsInterface()
        : errorMessage_()
    {
        // Add all Maestros to product list
        productList_[0x89] = 6; 
        productList_[0x8A] = 12; 
        productList_[0x8B] = 18; 
        productList_[0x8C] = 24;
    }
    
    MaestroCommsInterface::~MaestroCommsInterface() {}

    // Convert target position [-100, 100] to microseconds.
    // Note: Keep this function simple and optimized for speed.
    unsigned short MaestroCommsInterface::convertTargetToMicros(signed char target)
    {
        if (target > 100) target = 100;
        if (target < -100) target = -100;

        float scaleFactor = target / 100.0;
        short sign = (short)(scaleFactor / fabs(scaleFactor));
        float delta = 0;

        if (target < 0)
        {
            delta = channelHomeValue_ - minChannelValue_;
        }
        else if (target > 0)
        {
            delta = maxChannelValue_ - channelHomeValue_;
        }
        else
        {
            sign = 0; // Discount dead zone when target is 0
        }

        delta -= deadZoneValue_;
        delta *= scaleFactor;
        return (unsigned short)(channelHomeValue_ + sign*deadZoneValue_ + (short)delta);
    }

    // Set position for one target
    bool MaestroCommsInterface::setTarget(unsigned char channelNumber, signed char target)
    {
        unsigned short targetMicros = convertTargetToMicros(target);
        return writeBytes(COMMAND_SET_TARGET, targetMicros, (unsigned short)channelNumber);
    }

    // Sets the target for a channel, based on an 8-bit value, which the maestro translates
    // to the full-valued quarter MS value, where 127 is "neutral".
    bool MaestroCommsInterface::setTargetMiniSCC(unsigned char channelNumber, unsigned char normalizedTarget)
    {
        return writeBytes(COMMAND_MINI_SSC, channelNumber, normalizedTarget);
    }

    // Set the maximum speed at which a servo may move
    bool MaestroCommsInterface::setMaxSpeed(unsigned char channelNumber, unsigned short speed)
    {
        return writeBytes(COMMAND_SET_SPEED, speed, channelNumber);
    }

    // Set the max acceleration of the servo's movement
    bool MaestroCommsInterface::setMaxAcceleration(unsigned char channelNumber, unsigned short acceleration)
    {
        return writeBytes(COMMAND_SET_ACCELERATION, acceleration, (unsigned char)(channelNumber | 0x80));
    }

    // Send all servos to home position
    bool MaestroCommsInterface::goHome()
    {
        bool success = true;
        for (int i=0; i<numChannels_; ++i)
        {
            success &= writeBytes(COMMAND_SET_TARGET, convertTargetToMicros(0), i);
        }
        return success;
    }

    // Get number of PWM channels
    int MaestroCommsInterface::getNumChannels()
    {
        return numChannels_;
    }

    // Get the target, position, speed, acceleration of all servos
    std::vector<ServoStatus> MaestroCommsInterface::getAllPWMInfo()
    {
        std::vector<ServoStatus> servos;
        unsigned char data[numChannels_ * sizeof(ServoStatus)];
        int bytesRead = readBytes(REQUEST_GET_SERVO_SETTINGS, data, sizeof(data));
//        ROS_INFO_STREAM("[MaestroComms] read " << bytesRead << " bytes");
        
        if (bytesRead == numChannels_ * (sizeof(ServoStatus) - 1))
        {
            for (int i=0; i<numChannels_; ++i)
            {
                ServoStatus status;
                status.position = data[i*7] | data[i*7+1] << 8;
                status.target = data[i*7+2] | data[i*7+3] << 8;
                status.speed = data[i*7+4] | data[i*7+5] << 8;
                status.acceleration = data[i*7+6];
                servos.push_back(status);
            }
        }
	else 
	{
	  ROS_ERROR_STREAM("Bytes read (" << bytesRead << ") does not equal expected size (" << numChannels_ * sizeof(ServoStatus) << ")"); 
	}  
        return servos;
    }

    // Extract firmware version from mystery function.
    // Return empty string on failure.
    std::string MaestroCommsInterface::getFirmwareVersion()
    {
        unsigned char buf[14];
        int read = readBytes(REQUEST_GET_FIRMWARE, buf, sizeof(buf));
        if (read != 14) return "";
        unsigned char version[3];
        version[0] = (buf[12] & 0xF) + (buf[12] >> 4 & 0xF) * 10;
        version[1] = '.';
        version[2] = (buf[13] & 0xF) + (buf[13] >> 4 & 0xF) * 10;
        return std::string((const char*)version);
    }

    // "Reboot" the device.
    bool MaestroCommsInterface::reinitialize(unsigned short waitSeconds)
    {
        return writeBytes(COMMAND_REINITIALIZE, 0, 0);
    }

    // Set the duty cycle and period for the device's PWM.
    // NOTE: You probably shouldn't use this.
    bool MaestroCommsInterface::setPWM(unsigned short dutyCycle, unsigned short period)
    {
        return writeBytes(COMMAND_SET_PWM, dutyCycle, period);
    }

}
