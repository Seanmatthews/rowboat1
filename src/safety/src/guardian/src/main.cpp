#include "ros/ros/h"
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "guardian");
    ros::NodeHandle n;
    rowboat::Guardian g(n);
    ros::spin();
    return 0;
}
