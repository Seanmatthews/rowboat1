#ifndef LOGGER_HPP_
#define LOGGER_HPP_

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <ros/timer.h>
#include <rosbag/bag.h>
#include <rowboat_msgs/ControlPWMList.h>




namespace rowboat1
{
    class Logger {
    public:
        Logger(ros::NodeHandle nh);
        ~Logger();
        
        void init();
        void start();
        void stop();
        
    private:
        uint8_t storagePercentUsed();
        long long storageBytesRemaining();
        void timedStorageCheck(const ros::TimerEvent& event);
        void stopCameraRecording();
        
        // Callbacks
        void navCamImageCB(const sensor_msgs::ImageConstPtr& msg);
        void goproImageCB(const sensor_msgs::ImageConstPtr& msg);        
        void thrustCmdCB(const rowboat_msgs::ControlPWMListConstPtr& msg);
        void odomCB(const nav_msgs::OdometryConstPtr& msg);

        ros::NodeHandle nh_;
        rosbag::Bag bag_;
        ros::Timer storageCheckTimer_;

        image_transport::ImageTransport it_;
        image_transport::Subscriber navCamSub_;    // ptgrey camera images
        image_transport::Subscriber goproCamSub_;  // gopro camera images
        ros::Subscriber odomSub_;                  // all aggregated odom info
        ros::Subscriber thrustCmdSub_;             // commands to thrusters

        bool cameraRecordingStopped_;
        int minStorageBytesThresh_;
        std::string goproTopic_, navCamTopic_, thrustCmdTopic_, odomTopic_;
    };
}


#endif
