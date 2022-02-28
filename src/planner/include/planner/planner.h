#ifndef PLANNER_H
#define PLANNER_H

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
g#include "gazebo/LinkStates.h"

class Planner {
	public:
		Planner();
		virtual ~Planner();
		void handleGazeboState( const gazebo::LinkStates::ConstPtr& msg );

	protected:
		geometry_msg::Pose _pose;

};

#endif /* PLANNER_H */
