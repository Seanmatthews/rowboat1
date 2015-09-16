#include "guardian.h"

namespace rowboat1 {	

	Guardian::Guardian(ros::NodeHandle nh)
	{
		ROS_INFO( "Starting Guardian.");
	}

	Guardian::~Guardian()
	{
		ROS_INFO("Safely shutting down Guardian.");
	}

	/**
	 *  Start the node
	 */
	bool Guardian::init()
	{
		heartbeatSubs = new std::vector<std::string>();
		heartbeatAlerts = new std::map<std::string,std::map<int,std::string> >();
		
		// Print list of params for node to debug
		XmlRpc::XmlRpcValue paramSpace
 		if (nh_.getParam("/guardian", paramSpace))
		{
			ROS_INFO_STREAM(paramSpace);
		}

        // Load params explicitly and set default values if necessary
		if (!nh_.param<int>("/guardian/loopRate", loopRate_, 20))
		{
			ROS_WARN_STREAM("Setting loopRate_ to default value " << loopRate_);
		}
		if (!nh_.param<std::vector<std::string> >("/guardian/guardedNodes", guardedNodes_, new std::vector<std::string>()))
		{
			ROS_WARN_STREAM("Setting guardedNodes_ to default value " << guardedNodes_);
		}
		if (!nh_.param<int>("/guardian/heartbeatBufferMax ", heartbeatBufferMax_, 50))
		{
			ROS_WARN_STREAM("Setting heartbeatBufferMax_ to default value " << heartbeatBufferMax_):
		}
		
		return true;
	}

	/**
	 * Load heartbeat topics from a config file
	 */
	void Guardian::subscribeToHeartbeats()
	{
		for (std::vector<std::string>::iterator it = guardedNodes_.begin(); it != guardedNodes_.end(); ++it)
		{
			heartbeatSubs.push_back(nh_.subscribe(*it, heartbeatBufferMax_, Guardian::heartbeatCB));
		}
	}

	/**
	 *  Load statuses from a config file
	 */
	void Guardian::subscrubeToStatuses()
	{
	
	}

	/**
	 *  Called when heartbeats come in from observed nodes.
	 */
	void Guardian::heartbeatsCB()
	{

	}

	void Guardian::mainLoop()
	{
		while (ros::ok)
		{
			// loop through each topic and compare the delay time against
			// the times listed for the topic in the config file (ok, warn, critical, dead)

			ros::spinOnce();
			loopRate_.sleep();
		}
	}

}
