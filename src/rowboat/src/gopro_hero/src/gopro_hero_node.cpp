#include "gopro_hero/gopro_hero_node.hpp"
#include "sensor_msgs/Image.h"

using namespace std;
using namespace ros;

/**
 * The node does not have many primary functions-- triggering video & photo,
 * streaming video, and adjusting camera settings. However, there exist a
 * large number of settings. While the interal gopro_hero lib accesses these
 * settings individually, allowing such access through the ROS interface
 * would be cumbersome to develop and messy in a system of ROS nodes with
 * various params. To compartmentalize the camera params, users may set them
 * in groups of "camera", "video", "photo", and "multishot", using a
 * specialized hashmap-like message type.
 */

namespace rowboat1
{

    GoProHeroNode::GoProHeroNode(NodeHandle nh) :
        nh_(nh)
    {

    }

    GoProHeroNode::~GoProHeroNode() { }

    void GoProHeroNode::init()
    {
        modeSub_ = nh_.subscribe("mode", 1, &GoProHeroNode::modeCB, this);
        cameraSettingsSub_ = nh_.subscribe("camera_settings", 1, &GoProHeroNode::cameraSettingsCB, this);
        shutterTriggerSrv_ = nh_.advertiseService("trigger_shutter", &GoProHeroNode::triggerShutterCB, this);
    }
    
    void GoProHeroNode::start()
    {

    }

    void GoProHeroNode::mainLoop()
    {

        while (ros::ok())
        {
            // Publish new image if we're streaming
            if (gp_.isStreaming())
            {
                
            }

            // Adjust loop to coincide with GoPro FPS
//            ros::sleep(50);
            ros::spinOnce();
        }
    }

    void GoProHeroNode::modeCB(const std_msgs::Int8::ConstPtr& msg)
    {
        gp_.setMode(static_cast<GoProHero::Mode>(msg->data));
    }

    // NOTE no checks for proper enum value-- cast will occur regardless
    void GoProHeroNode::cameraSettingsCB(const gopro_hero_msgs::SettingsMap::ConstPtr& msg)
    {
        for (auto s : msg->settings)
        {
            auto val = s.id;
            auto name = s.name;
            
            if ("shutter" == name) gp_.shutter(val);
            else if ("orientation" == name) gp_.orientation(static_cast<Orientation>(val));
            else if("ledBlink" == name) gp_.ledBlink(static_cast<LEDBlink>(val));
            else if("beep" == name) gp_.beep(static_cast<Beep>(val));
            else if("lcdDisplay" == name) gp_.lcdDisplay(val);
            else if("onScreenDisplay" == name) gp_.onScreenDisplay(val); 
            else if("lcdBrightness" == name) gp_.lcdBrightness(static_cast<LCDBrightness>(val));
            else if("lcdLock" == name) gp_.lcdLock(val);
            else if("lcdSleepTimeout" == name) gp_.lcdSleepTimeout(static_cast<LCDSleepTimeout>(val));
            else if( "autoOffTime" == name) gp_.autoOffTime(static_cast<AutoOffTime>(val));

            else if("streamBitRate" == name) gp_.streamBitRate(static_cast<StreamBitRate>(val)); 

                //
                // MORE TO COME
                //
        }
    }


    // A convenience service for triggering the shutter, switching mode beforehand,
    // and receiving mode-specific outputs.
    bool GoProHeroNode::triggerShutterCB(gopro_hero_msgs::Shutter::Request& req,
                                         gopro_hero_msgs::Shutter::Response& rsp)
    {
        vector<vector<unsigned char> > images;
        gp_.setMode(req.multishot ? GoProHero::Mode::MULTISHOT : GoProHero::Mode::PHOTO);
        gp_.shutter(true);
        gp_.currentImages(images);
        
        // Convert image bytes to sensor_msgs/Image
        for (auto i : images)
        {
            sensor_msgs::Image rosImg;
            copy(i.begin(), i.end(), back_inserter(rosImg.data));
            rsp.images.push_back(rosImg);
        }
        
        return true;
    }

}
