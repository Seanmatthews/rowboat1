#include "ros/ros.h"
#include "controller_translator/controller_translator.h"

using namespace rowboat1;

// Nodes we make are not usually structured like this, but this node
// passes messages directly through at the received rate.
// Typically, we'll control the rate of spin.
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "controller_translator");
    ControllerTranslator obj(argc, argv);
    ros::spin();
    return 0;
}
