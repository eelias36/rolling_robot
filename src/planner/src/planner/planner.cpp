#include <iostream>
#include <cmath>
#include "planner/planner.h"


using namespace std;

Planner::Planner() {
	face_norm_vectors_initial[0] << 0, 0, -1;
	face_norm_vectors_initial[1] << 0, 1, -1;
	face_norm_vectors_initial[2] << 0, 1, 0;
	face_norm_vectors_initial[3] << 0, 1, 1;
	face_norm_vectors_initial[4] << 0, 0, 1;
	face_norm_vectors_initial[5] << 0, -1, 1;
	face_norm_vectors_initial[6] << 0, -1, 0;
	face_norm_vectors_initial[7] << 0, -1, -1;

	for (int i = 0; i < 8; i++) {
		face_norm_vectors_initial[i].normalize();
	}
}

Planner::~Planner() {

}

void Planner::handleGazeboState( const gazebo_msgs::LinkStates::ConstPtr& msg ) {
	
	// update pose class variable
	_pose = msg->pose[1];
	cout << "____________________" << endl;
	cout << _pose << endl << endl;	

	// convert geometry_msgs/quaternion to eigen quaternion
	static Eigen::Quaterniond eigen_quat;
	eigen_quat.x() = _pose.orientation.x;
	eigen_quat.y() = _pose.orientation.y;
	eigen_quat.z() = _pose.orientation.z;
	eigen_quat.w() = _pose.orientation.w;
/*
	cout << "x: " << eigen_quat.x() << endl;
	cout << "y: " << eigen_quat.y() << endl;
	cout << "z: " << eigen_quat.z() << endl;
	cout << "w: " << eigen_quat.w() << endl;
*/
	
	// update face normal vectors using new pose and find angle to <0, 0, -1>
	static Eigen::Vector3d v(0, 0, -1);
	for (int i = 0; i < 8; i++) {
		face_norm_vectors[i] = eigen_quat * face_norm_vectors_initial[i];
		//cout << "Face Normal Vector" << i << ": " << face_norm_vectors[i] << endl;

		_face_angles[i] = acos(face_norm_vectors[i].dot(v));
		cout << "Face Normal Angle " << i << ": " << _face_angles[i] << endl;
		
	}

	return;
}
