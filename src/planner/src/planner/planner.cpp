#include <iostream>
#include <cmath>
#include "planner/planner.h"


using namespace std;

Planner::Planner() {
	faceState_msg.data = -1;
	_face_norm_vectors_initial[0] << 0, 0, -1;
	_face_norm_vectors_initial[1] << 0, 1, -1;
	_face_norm_vectors_initial[2] << 0, 1, 0;
	_face_norm_vectors_initial[3] << 0, 1, 1;
	_face_norm_vectors_initial[4] << 0, 0, 1;
	_face_norm_vectors_initial[5] << 0, -1, 1;
	_face_norm_vectors_initial[6] << 0, -1, 0;
	_face_norm_vectors_initial[7] << 0, -1, -1;
	_face_norm_vectors_initial[8] << 1, 0, -1;
	_face_norm_vectors_initial[9] << 1, 0, 0;
	_face_norm_vectors_initial[10] << 1, 0, 1;
	_face_norm_vectors_initial[11] << -1, 0, 1;
	_face_norm_vectors_initial[12] << -1, 0, 0;
	_face_norm_vectors_initial[13] << -1, 0, -1;


	for (int i = 0; i < 14; i++) {
		_face_norm_vectors_initial[i].normalize();
	}
}

Planner::~Planner() {

}

void Planner::handleGazeboState( const gazebo_msgs::LinkStates::ConstPtr& msg ) {
	// update pose class variable
	_pose = msg->pose[1];
	cout << "____________________" << endl;
	cout << _pose << endl << endl;	
	findFaceState();

	return;
}

void Planner::findFaceState (void) {
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
	static int _faceState_temp = -1;

	for (int i = 0; i < 14; i++) {
		_face_norm_vectors[i] = eigen_quat * _face_norm_vectors_initial[i];
		//cout << "Face Normal Vector" << i << ": " << face_norm_vectors[i] << endl;

		_face_angles[i] = acos(_face_norm_vectors[i].dot(v));
		cout << "Face Normal Angle " << i << ": " << _face_angles[i] << endl;

		// check if angle is less than 20 degrees
		if(_face_angles[i] < 0.349066) {
			_faceState_temp = i;
			break;
		}
		
	}
	
	faceState_msg.data = _faceState_temp;
	cout << "Face State: " << (int) faceState_msg.data << endl;

	return;
}
