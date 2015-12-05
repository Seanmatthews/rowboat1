#ifndef MAESTRO_ROS_LAYER_H_
#define MAESTRO_ROS_LAYER_H_

#include <ros/ros.h>
#include <vector>
#include "std_msgs/String.h"
#include "pololu_maestro/ControlPWMList.h"
#include "pololu_maestro/maestro_comms_interface.h"

namespace navigator 
{
    class MaestroRosLayer 
    {
      public:
        MaestroRosLayer(ros::NodeHandle nh);
        ~MaestroRosLayer();
        void start();
        void killCB(const std_msgs::String::ConstPtr& msg);
        

      private:
        void mainLoop();
        void controlAllCB(const pololu_maestro::ControlPWMList::ConstPtr& msg);
        
        ros::NodeHandle nh_;
        ros::Publisher infoPub_;
        ros::Publisher heartbeatPub_;
        ros::Subscriber controlAllSub_;
        ros::Subscriber killSub_;
        unsigned short loopRateHz_;
        MaestroCommsInterface* comms;
        
    };
    
}


#endif // MAESTRO_ROS_LAYER_H_
