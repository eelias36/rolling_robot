#include <iostream>
#include <cmath>
#include "state_estimation/state_estimation.h"


using namespace std;

State_Estimation::State_Estimation() {

}

State_Estimation::~State_Estimation() {

}

void State_Estimation::handle_uwb_msg( const geometry_msgs::PoseArray::ConstPtr& msg ) {

	_uwb_vec[0].x() = msg->poses[1].position.x - msg->poses[0].position.x;
	_uwb_vec[0].y() = msg->poses[1].position.y - msg->poses[0].position.y;
	_uwb_vec[0].z() = msg->poses[1].position.z - msg->poses[0].position.z;

	_uwb_vec[1].x() = msg->poses[2].position.x - msg->poses[0].position.x;
	_uwb_vec[1].y() = msg->poses[2].position.y - msg->poses[0].position.y;
	_uwb_vec[1].z() = msg->poses[2].position.z - msg->poses[0].position.z;

	cout << "_____________________" << endl;
	cout << _uwb_vec[0] << endl;
	cout << _uwb_vec[1] << endl;


	return;
}

// void State_Estimation::handle_imu_msg( const sensor_msgs::MagneticField::ConstPtr& msg ) {
// 	_imu_msg_no_mag = *msg;
// 	return;
// }

sensor_msgs::MagneticField State_Estimation::mag_field_msg(void) const {

	Eigen::Vector3d x_axis;
	Eigen::Vector3d y_axis;
	Eigen::Vector3d z_axis;

	Eigen::Vector3d x_mag;
	Eigen::Vector3d y_mag;
	Eigen::Vector3d z_mag;

	Eigen::Vector3d B_field(1,0,0);


	// find 3 axes in world frame using vectors between UWB boards
	x_axis = _uwb_vec[0].normalized();
	z_axis = _uwb_vec[0].cross(_uwb_vec[1]).normalized();
	y_axis = -x_axis.cross(z_axis).normalized();

	sensor_msgs::MagneticField msg;

	// simulate magnetometer readings using axes and magnetic field vector
	msg.magnetic_field.x = x_axis.dot(B_field);
	msg.magnetic_field.y = y_axis.dot(B_field);
	// msg.magnetic_field.y = NAN;
	msg.magnetic_field.z = z_axis.dot(B_field);
	// msg.magnetic_field.z = NAN;

	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "UWB_boards";

	return msg;
}