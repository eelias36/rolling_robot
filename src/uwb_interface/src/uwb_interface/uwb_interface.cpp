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
	_uwb_pos[1].x() = msg->pose[20].position.x;
	_uwb_pos[1].y() = msg->pose[20].position.y;

	cout << "____________________" << endl;
	cout << msg->name[19] << endl;
	cout << "x1: " << _uwb_pos[0].x() << endl;	
	cout << "y1: " << _uwb_pos[0].y() << endl << endl;
	cout << msg->name[20] << endl;
	cout << "x2: " << _uwb_pos[1].x() << endl;	
	cout << "y2: " << _uwb_pos[1].y() << endl << endl;	

	return;
}

geometry_msgs::PoseArray Uwb_interface::uwb_pose_msg(void) const{
	geometry_msgs::PoseArray msg;
	msg.poses.resize(2);

	msg.header.stamp = ros::Time::now();
	for(int i=0; i<2; i++) {
		msg.poses[i].position.x = _uwb_pos[i].x();
		msg.poses[i].position.y = _uwb_pos[i].y();
	}

	return msg;
}




