#include "ros/ros.h"
#include "gopro_hero/gopro_hero_node.hpp"

using namespace rowboat1;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "GoProHero");
    ros::NodeHandle nh;
    GoProHeroNode* node = new GoProHeroNode(nh);
    node->start();
    return 0;
}
