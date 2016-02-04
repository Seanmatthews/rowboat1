#include "pololu_maestro/maestro_ros_layer.h"
#include "pololu_maestro/maestro_usb.h"
#include "rowboat_msgs/MaestroInfo.h"
#include "rowboat_msgs/Heartbeat.h"

namespace rowboat1 {

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
        ROS_INFO_STREAM("cool we're here");
        
        // Init pubs, subs, and srvs
        pwmInfoPub_ = nh_.advertise<rowboat_msgs::MaestroInfo>("info", 0);
        heartbeatPub_ = nh_.advertise<rowboat_msgs::Heartbeat>("heartbeat", 0);
        killSub_ = nh_.subscribe("kill", 10, &MaestroRosLayer::killCB, this);
        controlAllSub_ = nh_.subscribe("controlAllPWM", 1000, &MaestroRosLayer::controlAllCB, this);
        controlPWMSub_ = nh_.subscribe("controlPWM", 1000, &MaestroRosLayer::controlPWMCB, this);
        speedSub_ = nh_.subscribe("setSpeed", 10, &MaestroRosLayer::setSpeedCB, this);
        accelerationSub_ = nh_.subscribe("setAcceleration", 10, &MaestroRosLayer::setAccelerationCB, this);
        reinitializeSub_ = nh_.subscribe("reinitialize", 10, &MaestroRosLayer::reinitializeCB, this);
        goHomeSub_ = nh_.subscribe("goHome", 1000, &MaestroRosLayer::goHomeCB, this);
        mainLoop();
    }

    // Kills this ROS node
    void MaestroRosLayer::killCB(const std_msgs::String::ConstPtr& msg)
    {
        ROS_INFO_STREAM("put the killer's name here");
        ros::shutdown();
    }

    // Send a message to the Maestro to control all the PWM channels.
    // This is done to remove latency from sending several ROS messages.
    void MaestroRosLayer::controlAllCB(const rowboat_msgs::ControlPWMList::ConstPtr& msg)
    {
        // Is it faster to copy the elements into a new vector?
        bool success = true;
        for (int i=0; i<msg->targets.size(); ++i)
        {
            success &= comms->setTarget(i, msg->targets[i]);
        }
        if (!success)
        {
            ROS_ERROR_STREAM("Could not write targets to maestro");
        }
    }

    // Control one PWM port
    void MaestroRosLayer::controlPWMCB(const rowboat_msgs::ChannelValue::ConstPtr& msg)
    {
        if (!comms->setTarget(msg->channel, msg->value))
        {
            ROS_ERROR_STREAM("Could not write target to maestro");
        }
    }

    // Send all channels to neutral
    void MaestroRosLayer::goHomeCB(const std_msgs::Empty::ConstPtr& msg)
    {
        if (!comms->goHome())
        {
            ROS_ERROR_STREAM("Could not stop servos");
        }
    }

    // Callback for rebooting the maestro
    void MaestroRosLayer::reinitializeCB(const std_msgs::UInt8::ConstPtr& msg)
    {
        if (!comms->reinitialize(msg->data))
        {
            ROS_ERROR_STREAM("Could not reinitalize Maestro");
        }
    }

    // Callback for setting max speed at which PWM will reach its target value
    void MaestroRosLayer::setSpeedCB(const rowboat_msgs::ChannelValue::ConstPtr& msg)
    {
        if (!comms->setMaxSpeed(msg->channel, msg->value))
        {
            ROS_ERROR_STREAM("Could not set speed");
        }
    }

    // Callback for setting max acceleration at which PWM will approach its target value
    void MaestroRosLayer::setAccelerationCB(const rowboat_msgs::ChannelValue::ConstPtr& msg)
    {
        if (!comms->setMaxAcceleration(msg->channel, msg->value))
        {
            ROS_ERROR_STREAM("Could not set acceleration");
        }
    }

    // Callback for obtaining Maestro firmware version
    bool MaestroRosLayer::firmwareSrvCB(std_srvs::Trigger::Request& req,
                                        std_srvs::Trigger::Response& res)
    {
        std::string version = comms->getFirmwareVersion();
        if (version == "")
        {
            res.success = false;
            ROS_ERROR_STREAM("Could not get firmware version");
            return false;
        }
        res.message = version;
        res.success = true;
        return true;
    }

    // All it does is spin. Stupid loop.
    void MaestroRosLayer::mainLoop()
    {
        ros::Rate loopRate(loopRateHz_);
        rowboat_msgs::Heartbeat hbMsg;
        rowboat_msgs::MaestroInfo infoMsg;

        hbMsg.pkgName = "pololu_maestro";
        infoMsg.numServos = comms->getNumChannels();
        
        while (ros::ok())
        {
            // Fill heartbeat
            hbMsg.status = "OK";

            // Fill PWM info 
            std::vector<ServoStatus> pwmInfo = comms->getAllPWMInfo();
            infoMsg.positions.clear();
            infoMsg.targets.clear();
            infoMsg.speeds.clear();
            infoMsg.accelerations.clear();

            for (std::vector<ServoStatus>::iterator it = pwmInfo.begin(); it != pwmInfo.end(); ++it)
            {
                infoMsg.positions.push_back(it->position);
                infoMsg.targets.push_back(it->target);
                infoMsg.speeds.push_back(it->speed);
                infoMsg.accelerations.push_back(it->acceleration);
            }
            
            pwmInfoPub_.publish(infoMsg);
            heartbeatPub_.publish(hbMsg);
            
            ros::spinOnce();
            loopRate.sleep();
        }
    }
            
}
