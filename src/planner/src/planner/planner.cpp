#include <iostream>
#include <cmath>
#include <iterator>
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

	// map array entry: face state
	// key: face norm vector index
	// value: commanded direction [forward, back, left, right]
	_vector_direction_map[0].insert(pair<int,int>(2,0));
	_vector_direction_map[0].insert(pair<int,int>(8,1));
	_vector_direction_map[0].insert(pair<int,int>(12,2));
	_vector_direction_map[0].insert(pair<int,int>(9,3));
	_vector_direction_map[1].insert(pair<int,int>(3,0));
	_vector_direction_map[1].insert(pair<int,int>(7,1));
	_vector_direction_map[2].insert(pair<int,int>(4,0));
	_vector_direction_map[2].insert(pair<int,int>(0,1));
	_vector_direction_map[3].insert(pair<int,int>(5,0));
	_vector_direction_map[3].insert(pair<int,int>(1,1));
	_vector_direction_map[4].insert(pair<int,int>(6,0));
	_vector_direction_map[4].insert(pair<int,int>(2,1));
	_vector_direction_map[4].insert(pair<int,int>(12,2));
	_vector_direction_map[4].insert(pair<int,int>(9,3));
	_vector_direction_map[5].insert(pair<int,int>(7,0));
	_vector_direction_map[5].insert(pair<int,int>(3,1));
	_vector_direction_map[6].insert(pair<int,int>(0,0));
	_vector_direction_map[6].insert(pair<int,int>(4,1));
	_vector_direction_map[7].insert(pair<int,int>(1,0));
	_vector_direction_map[7].insert(pair<int,int>(5,1));
	_vector_direction_map[8].insert(pair<int,int>(13,2));
	_vector_direction_map[8].insert(pair<int,int>(10,3));
	_vector_direction_map[9].insert(pair<int,int>(0,2));
	_vector_direction_map[9].insert(pair<int,int>(4,3));
	_vector_direction_map[10].insert(pair<int,int>(8,2));
	_vector_direction_map[10].insert(pair<int,int>(11,3));
	_vector_direction_map[11].insert(pair<int,int>(10,2));
	_vector_direction_map[11].insert(pair<int,int>(13,3));
	_vector_direction_map[12].insert(pair<int,int>(4,2));
	_vector_direction_map[12].insert(pair<int,int>(0,3));
	_vector_direction_map[13].insert(pair<int,int>(11,2));
	_vector_direction_map[13].insert(pair<int,int>(8,3));

	// map array entry: face state
	// key: commanded direction [forward, back, left, right]
	// value: face norm vector index
	for (int i = 0; i < 14; i++) {
		for (itr = _vector_direction_map.begin(); itr != _vector_direction_map.end(); ++itr) {
			_inv_vector_direction_map[i].insert(pair<int,int>( itr->second, itr->first ));
    	}
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

		_cos_face_angles[i] = _face_norm_vectors[i].dot(v);
		cout << "Cos of Face Normal Angle " << i << ": " << _cos_face_angles[i] << endl;

		// check if cos(20 deg) is less than cos(angle between face normal and down)
		if(0.9396926207859083840541 < _cos_face_angles[i]) {
			_faceState_temp = i;
			break;
		}
		
	}
	
	_face_state = _faceState_temp;
	cout << "Face State: " << _face_state << endl;

	return;
}

std_msgs::Int8 Planner::faceState_msg(void) {
	std_msgs::Int8 msg
	msg.data = _face_state;
	return msg;
}

void Planner::handle_cmd_dir(const std_msgs::Int8::ConstPtr& msg) {
	_cmd_dir = msg->data;
	return;
}

std_msgs::Float64 Planner::heading_msg(void) {
	std_msgs::Float64 msg;

	Eigen::Vector2d v(1, 0);
	Eigen::Vector2d heading_vec = _face_norm_vectors[_inv_vector_direction_map[_face_state][_cmd_dir]].head<2>();

	double heading = acos(heading_vec.dot(v)); // find angle between heading vector and x-axis

	cout << "heading: " << heading << endl;

	msg.data = heading;
}