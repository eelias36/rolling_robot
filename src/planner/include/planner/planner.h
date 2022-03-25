#ifndef PLANNER_H
#define PLANNER_H

#include "ros/ros.h"
#include <Eigen/Dense>
#include <map>
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose.h"
#include "gazebo_msgs/LinkStates.h"

class Planner {
	public:
		Planner();
		virtual ~Planner();
		void handleGazeboState( const gazebo_msgs::LinkStates::ConstPtr& msg );
		void findFaceState(void);
		void handle_cmd_dir(const std_msgs::Int8::ConstPtr& msg);
		std_msgs::Int8 faceState_msg(void);
		ros::Publisher heading_publisher;

	protected:
		geometry_msgs::Pose _pose;
		Eigen::Vector3d _face_norm_vectors_initial[14];
		Eigen::Vector3d _face_norm_vectors[14];
		Eigen::Vector3d _face_fwd_vectors_initial[14];
		Eigen::Vector3d _dir_vectors[4];
		double _cos_face_angles[14];
		std::map<int, int> _vector_direction_map[14];
		std::map<int, int> _inv_vector_direction_map[14];
		int _cmd_dir;
		int _face_state;
		double _heading;

		

};

#endif /* PLANNER_H */
