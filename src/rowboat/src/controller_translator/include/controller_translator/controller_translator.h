#ifndef CONTROLLER_TRANSLATOR_H_
#define CONTROLLER_TRANSLATOR_H_

#include <ros/ros.h>
#include "sensor_msgs/Joy.h"

namespace rowboat1
{
    class ControllerTranslator
    {
      public:
        ControllerTranslator(ros::NodeHandle nh);
        ~ControllerTranslator();
        void start();

      private:
        void mainLoop();

        // TEMPORARY-- this should receive geometry_msgs/Twist
        void xBoxControlReceivedCB(const sensor_msgs::Joy::ConstPtr& msg);
//        void controlReceivedCB(const geometry_msgs::TwistStamped::ConstPtr& msg);

        std::vector<signed char> translateJoy(const sensor_msgs::Joy msg);
        
        
        ros::NodeHandle nh_;
       ros::Subscriber controlSub_;
        ros::Publisher controlPub_;
        ros::Publisher homePub_;
        
        unsigned short loopRateHz_;
        float minJoyStick_, maxJoyStick_;
        float minTrigger_, maxTrigger_;
        // geometry_msgs::TwistStamped
        std::vector<sensor_msgs::Joy> controlMsgBuffer_;
        

        // TODO: put all translation into a config.
        // This is sloppily specific.
        unsigned char forward_left_channel;
        unsigned char forward_right_channel;

        // COMMENTED OUT TO SIMPLIFY FOR HACKDAY
//        unsigned char up_left_channel;
//        unsigned char up_right_channel;
//        unsigned char side_front_channel;
//        unsigned char side_back_channel;
              
    };
    
} // namespace rowboat1


#endif // CONTROLLER_TRANSLATOR_H_
