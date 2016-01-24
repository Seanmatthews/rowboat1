#ifndef MAESTRO_ROS_LAYER_H_
#define MAESTRO_ROS_LAYER_H_

#include <ros/ros.h>
#include <vector>
#include "std_msgs/Empty.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include "rowboat_msgs/ChannelValue.h"
#include "rowboat_msgs/ControlPWMList.h"
#include "pololu_maestro/maestro_comms_interface.h"

namespace rowboat1 
{
    class MaestroRosLayer 
    {
      public:
        MaestroRosLayer(ros::NodeHandle nh);
        ~MaestroRosLayer();
        void start();

      private:
        void mainLoop();

        // Movement CBs
        void controlAllCB(const rowboat_msgs::ControlPWMList::ConstPtr& msg);
        void controlPWMCB(const rowboat_msgs::ChannelValue::ConstPtr& msg);
        void goHomeCB(const std_msgs::Empty::ConstPtr& msg);


        // Admin & Settings CBs
        void killCB(const std_msgs::String::ConstPtr& msg);
        void reinitializeCB(const std_msgs::UInt8::ConstPtr& msg);
        void setSpeedCB(const rowboat_msgs::ChannelValue::ConstPtr& msg);
        void setAccelerationCB(const rowboat_msgs::ChannelValue::ConstPtr& msg);
        
        // Services
        bool firmwareSrvCB(std_srvs::Trigger::Request& req,
                           std_srvs::Trigger::Response& res);
        
        ros::NodeHandle nh_;
        ros::Publisher heartbeatPub_;
        ros::Publisher pwmInfoPub_;
        ros::Subscriber controlPWMSub_;
        ros::Subscriber controlAllSub_;
        ros::Subscriber killSub_;
        ros::Subscriber speedSub_;
        ros::Subscriber accelerationSub_;
        ros::Subscriber reinitializeSub_;
        ros::Subscriber goHomeSub_;
        unsigned short loopRateHz_;
        MaestroCommsInterface* comms;
        
    };
    
}


#endif // MAESTRO_ROS_LAYER_H_
