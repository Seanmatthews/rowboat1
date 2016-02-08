#include "controller_translator/controller_translator.h"

// The node translates incoming joystick commands to PWM control commands.
// SImply put, the left joy stick controls front and left thrusters, the right
// joystick controls right and back thrusters.
const int CONTROLLER_BUTTONS_NUM = 10;
const int CONTROLLER_AXES_NUM = 7;
const int NUM_THRUSTERS = 6;

namespace rowboat1
{
    ControllerTranslator::ControllerTranslator(int argc, char** argv)
    {
        buttons_.resize(CONTROLLER_BUTTONS_NUM);
        axes_.resize(CONTROLLER_AXES_NUM);
        
        minJoyStick_ = n_.param<float>("minJoyStick", -1.0);
        maxJoyStick_ = n_.param<float>("maxJoyStick", 1.0);
        minTrigger_ = n_.param<float>("minTrigger", -1.0);
        maxTrigger_ = n_.param<float>("maxTrigger", 1.0);
        powerScale_ = n_.param<float>("powerScale", 0.5);
        minPWMCommand_ = n_.param<int>("minPWMCommand", -100);
        maxPWMCommand_ = n_.param<int>("maxPWMCommand", 100);

        // Construct the thrusterMap_. This is currently a simplification,
        // as we'll have two different xml specifications-- one for
        // thruster# -> joystick subscript, and another for
        // thruster# -> PWM port. Then we'll combine them here.
        // Actually, no need to overcomplicate this..
        n_.getParam("thrusterMap", thrusterMap_);
        
        controlPub_ = n_.advertise<rowboat_msgs::ControlPWMList>("controlAllPWM", 10);
        controlSub_ = n_.subscribe("joy", 1000, &ControllerTranslator::controlReceivedCB, this);        
    }

    ControllerTranslator::~ControllerTranslator() {}

    // The callback will indeed block on incomng messages, so it's imperative that
    // we 1) process messages quickly, and 2) don't buffer incoming messages. Any
    // buffered messages will introduce a latency based on the buffer size.
    void ControllerTranslator::controlReceivedCB(const sensor_msgs::Joy::ConstPtr& msg)
    {
        rowboat_msgs::ControlPWMList::Ptr cmdMsg = translateJoy(msg);
        controlPub_.publish(*cmdMsg);
    }
    
    rowboat_msgs::ControlPWMList::Ptr ControllerTranslator::translateJoy(const sensor_msgs::Joy::ConstPtr& msg)
    {
        rowboat_msgs::ControlPWMList::Ptr cmdMsg(new rowboat_msgs::ControlPWMList());

        // Don't operate on pointer
        this->buttons_.assign(msg->buttons.begin(), msg->buttons.end());
        this->axes_.assign(msg->axes.begin(), msg->axes.end());

//        for (std::map<int,int>::iterator it = thrusterMap_.begin(); it != thrusterMap_.end(); ++it)
//        {
//            cmdMsg->targets[it->first] = calculateCommandPercent(this.axes[it->second]);
//        }

        cmdMsg->targets[thrusterMap_[0]] = calculateCommandPercent(this->axes_[1]); // u/d left stick
        cmdMsg->targets[thrusterMap_[1]] = calculateCommandPercent(this->axes_[4]); // u/d right stick
        cmdMsg->targets[thrusterMap_[2]] = calculateCommandPercent(this->axes_[0]); // l/r left stick
        cmdMsg->targets[thrusterMap_[3]] = calculateCommandPercent(this->axes_[3]); // l/r right stick
        cmdMsg->targets[thrusterMap_[4]] = calculateCommandPercent(this->axes_[2]); // left trigger
        cmdMsg->targets[thrusterMap_[5]] = calculateCommandPercent(this->axes_[5]); // right trigger        
        
        return cmdMsg;
    }

    float ControllerTranslator::calculateCommandPercent(float axis)
    {
        return (axis-minJoyStick_) / (maxJoyStick_-minJoyStick_) * (maxPWMCommand_-minPWMCommand_) + minPWMCommand_;
    }

} // namespace rowboat1
