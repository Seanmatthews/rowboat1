#include <cstddef>
#include <ctime>
#include <time.h>
#include <iostream>
#include <boost/filesystem.hpp>
#include "logger/logger.hpp"

using namespace ros;
using namespace std;


namespace rowboat1
{
    /// Constructor
    Logger::Logger(NodeHandle nh) :
        nh_(nh),
        it_(nh),
        cameraRecordingStopped_(false)
    {
    }

    /// Destructor
    Logger::~Logger()
    {
        bag_.close();
    }

    /// Obtain launch file params, initialize bagfile, start storage check timer
    void Logger::init()
    {
        // Mandatory launch file vars
        bool paramsPresent = true;
        paramsPresent &= nh_.getParam("goproTopic", goproTopic_);
        paramsPresent &= nh_.getParam("navCamTopic", navCamTopic_);
        paramsPresent &= nh_.getParam("thrustCmdTopic", thrustCmdTopic_);
        paramsPresent &= nh_.getParam("odomTopic", odomTopic_);
        paramsPresent &= nh_.getParam("minStorageBytesThresh", minStorageBytesThresh_);

        // Optional launch file vars
        int storageCheckInterval;
        string bagFilenameFormat;
        nh_.param<int>("storageCheckInterval", storageCheckInterval, 10);
        nh_.param<string>("bagFilenameFormat", bagFilenameFormat, "%Y%m%d%H%M.bag");
        
        // Open "unique" bag name
        time_t now = time(nullptr);
        char bagfile[16];
        strftime(bagfile, 16, bagFilenameFormat.c_str(), localtime(&now));
        bag_.open(bagfile, rosbag::bagmode::Write);

        // Check storage at intervals
        storageCheckTimer_ =
            nh_.createTimer(ros::Duration(storageCheckInterval), &Logger::timedStorageCheck, this);        
    }

    /// Fired at intervals to ensure enough storage remains
    void Logger::timedStorageCheck(const TimerEvent& event)
    {
        auto bytesLeft = storageBytesRemaining();
        ROS_INFO_STREAM(bytesLeft << " storage bytes remaining");

        // Report bytes through diagnostics 

        if (bytesLeft < minStorageBytesThresh_ && !cameraRecordingStopped_)
        {
            ROS_FATAL_STREAM("Too little storage remaining. Halting camera recording.");
            
            // Tell arbiter, as we'll have to take action
            // Set diagnostic status?
            
            // Halt camera recording
            stopCameraRecording();
        }
    }
    
    /// Setup and start recording
    void Logger::start()
    {
        // start subscribers
        if (!cameraRecordingStopped_)
        {
            goproCamSub_ = it_.subscribe(goproTopic_, 1, &Logger::goproImageCB, this);
            navCamSub_ = it_.subscribe(navCamTopic_, 1, &Logger::navCamImageCB, this);
        }
        thrustCmdSub_ = nh_.subscribe(thrustCmdTopic_, 1000, &Logger::thrustCmdCB, this);
        odomSub_ = nh_.subscribe(odomTopic_, 1000, &Logger::odomCB, this);
    }

    /// Pause recording but don't shutdown
    void Logger::stop()
    {
        if (!cameraRecordingStopped_)
        {
            goproCamSub_.shutdown();
            navCamSub_.shutdown();
        }
        thrustCmdSub_.shutdown();
        odomSub_.shutdown();
    }

    /// Stop only camera image recording
    void Logger::stopCameraRecording()
    {
        cameraRecordingStopped_ = true;
        goproCamSub_.shutdown();
        navCamSub_.shutdown();
    }


    /// Obtain percent of local storage exhausted
    /// \return storage percent used
    uint8_t Logger::storagePercentUsed()
    {
//        using namespace boost::filesystem;
//        space_info si = space(".");
//        return (si.capacity - si.available) / si.capacity * 100;
        return 0;
    }

    /// Obtain number of bytes remaining
    /// \return total bytes remaining
    long long Logger::storageBytesRemaining()
    {
//        using namespace boost::filesystem;
//        space_info si = space(".");
//        ROS_DEBUG_STREAM(si.available << " bytes available out of " << si.capacity);
//        return si.available;
        return 0;
    }

    /// Receive image messages from gopro
    /// \param msg typical non-compressed image message
    /// \note Change to compressed if there are throughput problems
    void Logger::goproImageCB(const sensor_msgs::ImageConstPtr& msg)
    {
        bag_.write(goproTopic_, msg->header.stamp, msg);
    }

    /// Receive image messages from navigation camera
    /// \param msg typical non-compressed image message
    /// \note Change to compressed if there are throughput problems    
    void Logger::navCamImageCB(const sensor_msgs::ImageConstPtr& msg)
    {
        bag_.write(navCamTopic_, msg->header.stamp, msg);
    }    

    /// Receive commands going to the PWM thruster control board
    /// \param msg list of 8bit ints
    void Logger::thrustCmdCB(const rowboat_msgs::ControlPWMListConstPtr& msg)
    {
        bag_.write(thrustCmdTopic_, msg->header.stamp, msg);
    }

    /// Receive odometry information from central source
    /// \msg standard ros odometry message
    void Logger::odomCB(const nav_msgs::OdometryConstPtr& msg)
    {
        bag_.write(odomTopic_, msg->header.stamp, msg);
    }
}
