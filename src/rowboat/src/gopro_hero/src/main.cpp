#include "ros/ros.h"
#include "gopro_hero/gopro_hero_ros.h"

using namespace rowboat1;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "GoProHero");
    ros::NodeHandle nh;
    GoProHeroRos* node = new GoProHeroRos(nh);
    node->start();
    return 0;
}
