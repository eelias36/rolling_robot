#ifndef PLANNER_H
#define PLANNER_H

#include "ros/ros.h"
#include <Eigen/Dense>
#include "std_msgs/Int8.h"
#include "geometry_msgs/Pose.h"
#include "gazebo_msgs/LinkStates.h"

class Planner {
	public:
		Planner();
		virtual ~Planner();
		void handleGazeboState( const gazebo_msgs::LinkStates::ConstPtr& msg );
		void findFaceState(void);
		std_msgs::Int8 faceState_msg;

	protected:
		geometry_msgs::Pose _pose;
		Eigen::Vector3d _face_norm_vectors_initial[8];
		Eigen::Vector3d _face_norm_vectors[8];
		double _face_angles[8];
		

};

#endif /* PLANNER_H */
