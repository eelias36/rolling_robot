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
		Eigen::Vector3d _face_norm_vectors_initial[14];
		Eigen::Vector3d _face_norm_vectors[14];
		Eigen::Vector3d _face_fwd_vectors_initial[14];
		Eigen::Vector3d _dir_vectors[4];
		double _cos_face_angles[14];
		

};

#endif /* PLANNER_H */
