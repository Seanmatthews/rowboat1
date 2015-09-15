#include "guardian.h"

namespace rowboat1 {	

	Guardian::Guardian(ros::NodeHandle nh)
	{
		ROS_DEBUG( "Starting Guardian.\n");
	}

	Guardian::~Guardian()
	{
		
	}

	/**
	 *  Start the node
	 */
	bool Guardian::init()
	{
		// Load params
	}

	/**
	 * Load heartbeat topics from a config file
	 */
	void Guardian::subscribeToHeartbeats()
	{
		
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
