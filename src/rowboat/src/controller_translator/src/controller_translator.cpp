#include "controller_translator/controller_translator.h"
#include "rowboat_msgs/ControlPWMList.h"
#include "std_msgs/Empty.h"

// The node translates incoming joystick commands to PWM control commands.

namespace rowboat1
{
    ControllerTranslator::ControllerTranslator(ros::NodeHandle nh)
    {
        nh_ = nh;
        loopRateHz_ = nh.param<int>("loopRateHz", 50);
        //minJoyStick_ = nh.param<float>("minJoyStick", 0);
        //maxJoyStick_ = nh.param<float>("maxJoyStick", 0);
        //minTrigger_ = nh.param<float>("minTrigger", 0);
        //maxTrigger_ = nh.param<float>("maxTrigger", 0);
    }

    ControllerTranslator::~ControllerTranslator() {}

    
    void ControllerTranslator::start()
    {
        homePub_ = nh_.advertise<std_msgs::Empty>("goHome", 10);
        controlPub_ = nh_.advertise<rowboat_msgs::ControlPWMList>("controlAllPWM", 10);
        controlSub_ = nh_.subscribe("joy", 1000, &ControllerTranslator::xBoxControlReceivedCB, this);
        ROS_INFO("Almost started");
        mainLoop();
    }

    // The main loop runs asynchronously against the inputs, so that
    // we can guarantee the output rate regardless of input rate.
    void ControllerTranslator::mainLoop()
    {
        ros::Rate loopRate(loopRateHz_);
        
        while (ros::ok())
        {
            // Analyze the buffer and account for high buffering rate
            // TODO

            if (controlMsgBuffer_.size() > 0)
            {
                // If buffer has a value, get it, translate, and publish.
                rowboat_msgs::ControlPWMList msg;
                sensor_msgs::Joy joyMsg = controlMsgBuffer_.back();
                controlMsgBuffer_.pop_back();
                controlMsgBuffer_.clear();
                msg.targets = translateJoy(joyMsg);

                if (joyMsg.buttons[4] == 1 || joyMsg.buttons[5] == 1)
                {
                    std_msgs::Empty homeMsg;
                    homePub_.publish(homeMsg);
                }
                else
                {
                    controlPub_.publish(msg);
                }
            }
            ros::spinOnce();
            loopRate.sleep();
        }
    }

    // Translates from XBox controller to PWM control
    void ControllerTranslator::xBoxControlReceivedCB(const sensor_msgs::Joy::ConstPtr& msg)
    {
        // Buffer incoming joy commands
        // TEMP -- one message at a time, and only if a button was pressed
        if (controlMsgBuffer_.size() == 0)
        {
            std::vector<int> v(msg->buttons);
            for (std::vector<int>::iterator it=v.begin(); it != v.end(); ++it)
            {
                if (*it > 0) controlMsgBuffer_.push_back(*msg);
            }
        }
    }

    // For now, we can use buttons to go forward/back left/right. We're only using the
    // first two PWM ports, and a static speed. Basically, this whole method is foe testing.
    std::vector<signed char> ControllerTranslator::translateJoy(const sensor_msgs::Joy msg)
    {
        std::vector<signed char> targets(8, 0);
        int ON = 1;
        
        if (msg.buttons[2] == ON) // left
        {
            targets[0] = 25;
            targets[1] = -25;
        }
        else if (msg.buttons[1] == ON) // right
        {
            targets[0] = -25;
            targets[1] = 25;
        }
        else if (msg.buttons[3] == ON) // forward
        {
            targets[0] = 25;
            targets[1] = 25;
        }
        else if (msg.buttons[0] == ON) // back
        {
            targets[0] = -25;
            targets[1] = -25;
        }
        else 
        {
            targets[0] = 0;
            targets[1] = 0;
        }

        return targets;
    }
    
}






