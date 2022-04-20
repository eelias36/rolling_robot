#include <iostream>
#include <cmath>
#include "uwb_interface/uwb_interface.h"


using namespace std;

Uwb_interface::Uwb_interface() {

}

Uwb_interface::~Uwb_interface() {

}

void Uwb_interface::handleGazeboState( const gazebo_msgs::LinkStates::ConstPtr& msg ) {

	_uwb_pos[0].x() = msg->pose[19].position.x;
	_uwb_pos[0].y() = msg->pose[19].position.y;
	_uwb_pos[0].z() = msg->pose[19].position.z;

	_uwb_pos[1].x() = msg->pose[20].position.x;
	_uwb_pos[1].y() = msg->pose[20].position.y;
	_uwb_pos[1].z() = msg->pose[20].position.z;

	_uwb_pos[2].x() = msg->pose[21].position.x;
	_uwb_pos[2].y() = msg->pose[21].position.y;
	_uwb_pos[2].z() = msg->pose[21].position.z;

	// cout << "____________________" << endl;
	// cout << msg->name[19] << endl;
	// cout<< _uwb_pos[0] << endl << endl;
	// cout << msg->name[20] << endl;
	// cout<< _uwb_pos[1] << endl << endl;
	// cout << msg->name[21] << endl;
	// cout<< _uwb_pos[2] << endl << endl;

	return;
}

geometry_msgs::Point Uwb_interface::uwb_pos_msg(const int uwb_id) const{
	// geometry_msgs::PoseArray msg;
	// msg.poses.resize(3);

	// msg.header.stamp = ros::Time::now();
	// for(int i=0; i<3; i++) {
	// 	msg.poses[i].position.x = _uwb_pos[i].x();
	// 	msg.poses[i].position.y = _uwb_pos[i].y();
	// 	msg.poses[i].position.z = _uwb_pos[i].z();
	// }

	geometry_msgs::Point msg;

	msg.x = _uwb_pos[uwb_id].x();
	msg.y = _uwb_pos[uwb_id].y();
	msg.z = _uwb_pos[uwb_id].z();

	return msg;
}




