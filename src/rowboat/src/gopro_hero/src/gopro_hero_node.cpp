#include "gopro_hero/gopro_hero_node.hpp"

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

    void GoProHeroNode::GoProHeroNode(NodeHandle nh) :
        nh_(nh)
    {
        funcMap_["WhiteBalance"] = &GoProHero::whiteBalance;
    }

    void GoProHeroNode::~GoProHeroNode() { }

    void GoProHeroNode::init()
    {
        modeSub_ = nh_.subscribe("mode", 1, &GoProHeroNode::modeCB, this);
        cameraSettingsSub_ = nh_.subscribe("camera_settings", 1, &GoProHeroNode::cameraSettingsCB, this);
        shutterTriggerSub_ = nh_.subscribe("trigger_shutter", 10, &GoProHeroNode::triggerShutterCB, this);
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
            ros::sleep(50);
            ros::spinOnce();
        }
    }

    void GoProHeroNode::modeCB(const std_msgs::Int::ConstPtr& msg)
    {
        gp_.setMode(static_cast<Mode>(msg->data));
    }

    // NOTE no checks for proper enum value-- cast will occur regardless
    void GoProHeroNode::cameraSettingsCB(const gopro_hero_msgs::SettingsMap::ConstPtr& msg)
    {
        for (auto s : msg->settings)
        {
            auto val = s.id;
            switch(s.name)
            {
            case "shutter": gp_.shutter(val); break;
            case "orientation": gp_.orientation(static_cast<Orientation>(val)); break;
            case "ledBlink": gp_.ledBlink(static_cast<LEDBlink>(val)); break;
            case "beep": gp_.beep(static_cast<Beep>(val)); break;
            case "lcdDisplay": gp_.lcdDisplay(val); break;
            case "onScreenDisplay": gp_.onScreenDisplay(val); break;
            case "lcdBrightness": gp_.lcdBrightness(static_cast<LCDBrightness>(val)); break;
            case "lcdLock": gp_.lcdLock(val); break;
            case "lcdSleepTimeout": gp_.lcdSleepTimeout(static_cast<LCDSleepTimeout>(val)); break;
            case "autoOffTime": gp_.autoOffTime(static_cast<AutoOffTime>(val)); break;

            case "streamBitRate": gp_.streamBitRate(static_cast<StreamBitRate>(val)); break;

                //
                // MORE TO COME
                //
                
            default: break;
            }
        }
    }


    // A convenience service for triggering the shutter, switching mode beforehand,
    // and receiving mode-specific outputs.
    void GoProHeroNode::triggerShutterCB(const gopro_hero_msgs::Shutter::Request& req,
                                         const gopro_hero_msgs::Shutter::Response& rsp)
    {
        gp_.setMode(req.multi ? Mode::MULTI : Mode::PHOTO);
        gp_.shutter(true);
        rsp.images = gp_.currentImageList();
        return true;
    }

}
