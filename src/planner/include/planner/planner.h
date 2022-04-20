#ifndef PLANNER_H
#define PLANNER_H

#include "ros/ros.h"
#include <Eigen/Dense>
#include <cmath>
#include <map>
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
// #include "gazebo_msgs/LinkStates.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"

class Planner {
	public:
		Planner();
		virtual ~Planner();
		// void handleGazeboState( const gazebo_msgs::LinkStates::ConstPtr& msg );
		void findFaceState(void);
		void handle_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg);
		void handleOrientation(const sensor_msgs::Imu::ConstPtr& msg);
		void handlePosition(const geometry_msgs::PoseStamped::ConstPtr& msg);
		void handle_rolling_msg( const std_msgs::Bool::ConstPtr& msg );
		void handle_goal_msg( const geometry_msgs::Point::ConstPtr& msg );
		void handle_roll_to_goal_msg( const std_msgs::Bool::ConstPtr& msg );
		std_msgs::Int8 faceState_msg(void);
		ros::Publisher heading_publisher;
		void command_update(void);
		ros::Publisher cmd_publisher;

	protected:
		geometry_msgs::Pose _pose;
		Eigen::Vector3d _face_norm_vectors_initial[14];
		Eigen::Vector3d _face_norm_vectors[14];
		Eigen::Vector3d _face_fwd_vectors_initial[14];
		Eigen::Vector3d _dir_vectors[4];
		double _cos_face_angles[14];
		std::map<int, int> _vector_direction_map[14];
		std::map<int, int> _inv_vector_direction_map[14];
		int _pred_face_LUT[16][4];
		int _cmd_dir;
		int _face_state;
		double _heading;
		bool _rolling_state;
		bool _roll_to_goal;
		ros::Time _time_at_roll_finish;
		double _roll_wait_secs;
		Eigen::Vector2d _goal;
		int _prev_face;

		

};

#endif /* PLANNER_H */
