#include "ros/ros.h"
#include "controller_translator/controller_translator.h"

using namespace rowboat1;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ControllerTranslator");
    ros::NodeHandle nh;
    ControllerTranslator* ct = new ControllerTranslator(nh);
    //ct->start();
    
    return 0;
}
