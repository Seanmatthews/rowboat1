#ifndef GUARDIAN_H_ 
#define GUARDIAN_H_

#include <string>
#include <stdarg>

#include <ros/ros.h>
#include <ros/package.h>

namespace rowboat1
{
    class Guardian
    {
    public:
        Guardian(ros::NodeHandle &nh);
        ~Guardian();
    
    private:
        void subscribeToHeartbeats();
        void subscribeToStatuses();

        ros::NodeHandle n_;
        ros::Subscriber sub_;
        
    }
} // end namespace rowboat1

#endif
