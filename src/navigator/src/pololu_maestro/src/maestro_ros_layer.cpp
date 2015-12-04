#include "pololu_maestro/maestro_ros_layer.h"
#include "pololu_maestro/maestro_usb.h"

#include "std_msgs/Empty.h"
#include "pololu_maestro/Info.h"

namespace navigator {

    MaestroRosLayer::MaestroRosLayer(ros::NodeHandle nh)
    {
        nh_ = nh;
        loopRateHz_ = nh.param<int>("loopRateHz", 50);
    }

    MaestroRosLayer::~MaestroRosLayer() {}

    // Blocking function to be called from main to start the node
    void MaestroRosLayer::start()
    {
        // Attempt to connect to a Maestro device
        comms = new MaestroUsb();
        if (!comms->connect())
        {
            ROS_ERROR_STREAM("Could not connect to a Maestro device");
        }
        ROS_INFO_STREAM("Connected to a Maestro device!");
        
        // Init pubs, subs, and srvs
        infoPub_ = nh_.advertise<pololu_maestro::Info>("info", 0);
        heartbeatPub_ = nh_.advertise<std_msgs::Empty>("heartbeat", 0);
        killSub_ = nh_.subscribe("kill", 10, &MaestroRosLayer::killCB, this);
        controlAllSub_ = nh_.subscribe("controlAllPWM", 1000, &MaestroRosLayer::controlAllCB, this);
        
        mainLoop();
    }

    // Kills this ROS node
    void MaestroRosLayer::killCB(const std_msgs::String::ConstPtr& msg)
    {
        ROS_INFO_STREAM("put the killer's name here");
        ros::shutdown();
    }

    // Send a message to the Maestro to control all the PWM ports at the same time
    void MaestroRosLayer::controlAllCB(const pololu_maestro::ControlPWMList::ConstPtr& msg)
    {
        // Is it faster to copy the elements into a new vector?
        comms->setAllTargets(msg->targets);
    }

    // All it does is spin. Stupid loop.
    void MaestroRosLayer::mainLoop()
    {
        ros::Rate loopRate(loopRateHz_);
        
        while (ros::ok())
        {
            ros::spinOnce();
            loopRate.sleep();
        }
    }
            
}
