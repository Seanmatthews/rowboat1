#include "ros/ros.h"
#include "pololu_maestro/maestro_ros_layer.h"

using namespace navigator;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "PololuMaestro");
    ros::NodeHandle nh;
    MaestroRosLayer* mrl = new MaestroRosLayer(nh);
    mrl->start();
    
    return 0;
}
