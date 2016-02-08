#ifndef CONTROLLER_TRANSLATOR_H_
#define CONTROLLER_TRANSLATOR_H_

#include <ros/ros.h>
#include "sensor_msgs/Joy.h"
#include "rowboat_msgs/ControlPWMList.h"
#include <vector>

namespace rowboat1
{
    class ControllerTranslator
    {
      public:
        ControllerTranslator(int argc, char** argv);
        ~ControllerTranslator();

      private:
        void controlReceivedCB(const sensor_msgs::Joy::ConstPtr& msg);
        rowboat_msgs::ControlPWMList::Ptr translateJoy(const sensor_msgs::Joy::ConstPtr& msg);
        float calculateCommandPercent(float axis);
        
        ros::NodeHandle n_;
        ros::Subscriber controlSub_;
        ros::Publisher controlPub_;
        
        float minJoyStick_, maxJoyStick_;
        float minTrigger_, maxTrigger_;
        float minPWMCommand_, maxPWMCommand_;
        float powerScale_;

        // thruster# -> PWM port 
        std::vector<int> thrusterMap_;
       
        std::vector<int> buttons_;
        std::vector<float> axes_;
              
    };
    
} // namespace rowboat1


#endif // CONTROLLER_TRANSLATOR_H_
