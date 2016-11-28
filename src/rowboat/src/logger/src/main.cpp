#include <ros/ros.h>
#include "logger/logger.hpp"

using namespace rowboat1;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "Logger");
    ros::NodeHandle nh;
    Logger* node = new Logger(nh);
    ros::spin();
    return 0;
}
