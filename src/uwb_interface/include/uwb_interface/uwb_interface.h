#ifndef UWB_INTERFACE_H
#define UWB_INTERFACE_H

#include "ros/ros.h"
#include <Eigen/Dense>
#include "gazebo_msgs/LinkStates.h"
#include "geometry_msgs/PoseArray.h"


class Uwb_interface {
	public:
		Uwb_interface();
		virtual ~Uwb_interface();
		void handleGazeboState( const gazebo_msgs::LinkStates::ConstPtr& msg );
		geometry_msgs::Point uwb_pos_msg(const int uwb_id) const;
		geometry_msgs::PoseArray uwb_pose_msg(void) const;

	protected:
		Eigen::Vector3d _uwb_pos[3];

};

#endif /* UWB_INTERFACE_H */











