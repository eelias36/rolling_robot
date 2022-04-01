#include <iostream>
#include <cmath>
#include <iterator>
#include "planner/planner.h"


using namespace std;

Planner::Planner() {
	_face_state = -1;
	_roll_wait_secs = 2;
	_roll_to_goal = false;
	ros::Time t(0);
	_time_at_roll_finish = t;

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
		_face_norm_vectors[i] = _face_norm_vectors_initial[i];
	}

	// map array index: face state
	// key: face norm vector index
	// value: commanded direction [forward, back, left, right]
	_vector_direction_map[0].insert(pair<int,int>(2,0));
	_vector_direction_map[0].insert(pair<int,int>(6,1));
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

	// map array index: face state
	// key: commanded direction [forward, back, left, right]
	// value: face norm vector index
	std::map<int, int>::iterator itr;
	for (int i = 0; i < 14; i++) {
		for (itr = _vector_direction_map[i].begin(); itr != _vector_direction_map[i].end(); ++itr) {
			_inv_vector_direction_map[i].insert(pair<int,int>( itr->second, itr->first ));
    	}
	}

	// first index: current face
	// second index: commanded direction [forward, back, left, right]
	// entry: predicted face
	for (int i = 0; i < 14; i++){
		for (int j = 0; j < 4; j++){
			_pred_face_LUT[i][j] = -1;
		}
	}
	_pred_face_LUT[0][0] = 1;
	_pred_face_LUT[0][1] = 7;
	_pred_face_LUT[0][2] = 13;
	_pred_face_LUT[0][3] = 8;
	_pred_face_LUT[1][0] = 2;
	_pred_face_LUT[1][1] = 0;
	_pred_face_LUT[2][0] = 3;
	_pred_face_LUT[2][1] = 1;
	_pred_face_LUT[3][0] = 4;
	_pred_face_LUT[3][1] = 2;
	_pred_face_LUT[4][0] = 5;
	_pred_face_LUT[4][1] = 3;
	_pred_face_LUT[4][2] = 11;
	_pred_face_LUT[4][3] = 10;
	_pred_face_LUT[5][0] = 6;
	_pred_face_LUT[5][1] = 4;
	_pred_face_LUT[6][0] = 7;
	_pred_face_LUT[6][1] = 5;
	_pred_face_LUT[7][0] = 0;
	_pred_face_LUT[7][1] = 6;
	_pred_face_LUT[8][2] = 0;
	_pred_face_LUT[8][3] = 9;
	_pred_face_LUT[9][2] = 8;
	_pred_face_LUT[9][3] = 10;
	_pred_face_LUT[10][2] = 9;
	_pred_face_LUT[10][3] = 4;
	_pred_face_LUT[11][2] = 4;
	_pred_face_LUT[11][3] = 12;
	_pred_face_LUT[12][2] = 11;
	_pred_face_LUT[12][3] = 13;
	_pred_face_LUT[13][2] = 12;
	_pred_face_LUT[13][3] = 0;

}

Planner::~Planner() {

}

void Planner::handleOrientation(const sensor_msgs::Imu::ConstPtr& msg) {
	_pose.orientation = msg->orientation;
	// cout << "____________________" << endl;
	// cout << _pose << endl << endl;	
	findFaceState();
	return;
}

void Planner::handlePosition(const geometry_msgs::Point::ConstPtr& msg){
	_pose.position = *msg;
	return;
}

void Planner::handleGazeboState( const gazebo_msgs::LinkStates::ConstPtr& msg ) {
	// update pose class variable
	_pose = msg->pose[1];
	// cout << "____________________" << endl;
	// cout << _pose << endl << endl;	
	findFaceState();

	return;
}

void Planner::handle_rolling_msg( const std_msgs::Bool::ConstPtr& msg) {
	_rolling_state = msg->data;
	return;
}

void Planner::handle_goal_msg( const geometry_msgs::Point::ConstPtr& msg ) {
	_goal.x() = msg->x;
	_goal.y() = msg->y;
	return;
}

void Planner::handle_roll_to_goal_msg( const std_msgs::Bool::ConstPtr& msg ) {
	_roll_to_goal = msg->data;
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
		//cout << "Cos of Face Normal Angle " << i << ": " << _cos_face_angles[i] << endl;

		// check if cos(20 deg) is less than cos(angle between face normal and down)
		if(0.9396926207859083840541 < _cos_face_angles[i]) {
			_faceState_temp = i;
			break;
		}
		
	}
	
	_face_state = _faceState_temp;
	//cout << "Face State: " << _face_state << endl;

	return;
}

std_msgs::Int8 Planner::faceState_msg(void) {
	std_msgs::Int8 msg;
	msg.data = _face_state;
	return msg;
}

void Planner::handle_cmd_dir(const std_msgs::Int8::ConstPtr& msg) {
	_cmd_dir = msg->data;
	
	double heading;
	if(_cmd_dir != -1) {
		std_msgs::Float64 msg;

		Eigen::Vector2d v(1, 0);
		Eigen::Vector3d heading_vec_init = _face_norm_vectors[_inv_vector_direction_map[_face_state][_cmd_dir]];
		Eigen::Vector2d heading_vec;
		
		heading_vec.x() = heading_vec_init.x();
		heading_vec.y() = heading_vec_init.y();
		heading_vec.normalize();

		// cout << "__________" << endl;
		// cout << "Face: " << _face_state << endl;
		// cout << "Cmd Dir: " << _cmd_dir << endl;
		// cout << "Index: " << _inv_vector_direction_map[_face_state][_cmd_dir] << endl;
		// cout << "At Index: " << _face_norm_vectors[_inv_vector_direction_map[_face_state][_cmd_dir]] << endl;
		// cout << "Heading vector: " << heading_vec << endl;
		
		// find angle between heading vector and x-axis
		double angle = acos(heading_vec.dot(v)); 

		double heading;

		// positive heading if vector in quadrants I/II, negative if in III/IV
		if (heading_vec.y() >= 0) {
			heading = angle;
		} else {
			heading = -angle;
		}


		msg.data = heading;

		heading_publisher.publish(msg);
	}


	return;
}


geometry_msgs::Twist Planner::command_msg(void) {

	geometry_msgs::Twist msg;

	if ( _rolling_state == true ) {
		msg.linear.x = 0;
		msg.linear.y = 0;
		msg.linear.z = 0;

		_time_at_roll_finish = ros::Time::now();

	} else if ( (_rolling_state == false) && (((ros::Time::now()) - _time_at_roll_finish).toSec() > _roll_wait_secs) && (_roll_to_goal == true) )  {
		// if not rolling and needs to roll

		// iterate through vector direction map to evaluate each direction of motion given the current face
		double min_angle = M_PI;
		int best_cmd_dir;
		Eigen::Vector2d test_vec;
		Eigen::Vector2d goal_vec;
		goal_vec.x() = _goal.x() - _pose.position.x;
		goal_vec.y() = _goal.y() - _pose.position.y;
		double goal_angle = atan2(goal_vec.y(), goal_vec.x());

		std::map<int, int>::iterator itr;
		cout << "-----------" << endl;
		cout << _pose << endl;
        cout << "Goal is " << _goal << endl;
        cout << "Goal angle is " << goal_angle << endl;

		double vec_angle;
		double angle;

		cout << "Evaluating " << _vector_direction_map[_face_state].size() << " directions..." << endl << endl;

		for (itr = _vector_direction_map[_face_state].begin(); itr != _vector_direction_map[_face_state].end(); ++itr) {


		        test_vec.x() = _face_norm_vectors[itr->first].x();
		        test_vec.y() = _face_norm_vectors[itr->first].y();

		        // find angle of vector from x-axis
		        vec_angle = atan2(test_vec.y(), test_vec.x());

		        // find angle between vector and goal
		        angle = vec_angle - goal_angle;
				if (angle > M_PI)        { angle -= 2 * M_PI; }
				else if (angle <= -M_PI) { angle += 2 * M_PI; }
				angle = abs(angle);
		        
		        if ( angle < min_angle) {
		        	min_angle = angle;
		        	best_cmd_dir = itr->second;
		        }

		        //cout << "Vector angle is " << vec_angle << endl;
		        cout << "Face norm vec " << itr->first << " is at an angle of " << angle << " for direction " << itr->second << endl;
	     }

	     cout << "Commanded direction: " << best_cmd_dir << endl;

	     if (best_cmd_dir == 0) {
	     	msg.linear.x = 1;
	     	msg.linear.y = 0;
	     	msg.linear.z = 0;
	     } else if (best_cmd_dir == 1) {
	     	msg.linear.x = -1;
	     	msg.linear.y = 0;
	     	msg.linear.z = 0;
	     } else if (best_cmd_dir == 2) {
	     	msg.linear.x = 0;
	     	msg.linear.y = 1;
	     	msg.linear.z = 0;
	     } else if (best_cmd_dir == 3) {
	     	msg.linear.x = 0;
	     	msg.linear.y = -1;
	     	msg.linear.z = 0;
	     }

     }

	return msg;

}