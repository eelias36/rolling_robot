#include <iostream>
#include <cmath>
#include "state_estimation/state_estimation.h"


using namespace std;

State_Estimation::State_Estimation(const Eigen::Vector3d& alpha) :  _u(),
																	_mu( Eigen::Vector2d::Zero( 2 ) ),
																	_sigma( Eigen::Matrix2d::Zero( 2, 2 ) ),
																	_alpha( alpha ){

	_pos_estimate << 0, 0, 0.5;

}

State_Estimation::~State_Estimation() {

}

void State_Estimation::handle_heading_msg( const std_msgs::Float64::ConstPtr& msg ) {
	_u = (*msg).data;
	return;
}

void State_Estimation::handle_rolling_msg( const std_msgs::Bool::ConstPtr& msg) {
	_rolling = (*msg).data;
	return;
}


void State_Estimation::handle_uwb_msg( const geometry_msgs::PoseArray::ConstPtr& msg ) {

	Eigen::Vector3d new_pos[3];

	new_pos[0].x() = msg->poses[0].position.x;
	new_pos[0].y() = msg->poses[0].position.y;
	new_pos[0].z() = msg->poses[0].position.z;

	new_pos[1].x() = msg->poses[1].position.x;
	new_pos[1].y() = msg->poses[1].position.y;
	new_pos[1].z() = msg->poses[1].position.z;

	new_pos[2].x() = msg->poses[2].position.x;
	new_pos[2].y() = msg->poses[2].position.y;
	new_pos[2].z() = msg->poses[2].position.z;

	_uwb_pos[0].push_back(new_pos[0]);
	_uwb_pos[1].push_back(new_pos[1]);
	_uwb_pos[2].push_back(new_pos[2]);


	// keep rolling average only of past 5 measurements
	if (_uwb_pos[0].size() > 5) {
		_uwb_pos[0].pop_front();
		_uwb_pos[1].pop_front();
		_uwb_pos[2].pop_front();
	}

	find_pos();

	// _uwb_vec[0].x() = msg->poses[1].position.x - msg->poses[0].position.x;
	// _uwb_vec[0].y() = msg->poses[1].position.y - msg->poses[0].position.y;
	// _uwb_vec[0].z() = msg->poses[1].position.z - msg->poses[0].position.z;

	// _uwb_vec[1].x() = msg->poses[2].position.x - msg->poses[0].position.x;
	// _uwb_vec[1].y() = msg->poses[2].position.y - msg->poses[0].position.y;
	// _uwb_vec[1].z() = msg->poses[2].position.z - msg->poses[0].position.z;

	// cout << "_____________________" << endl;
	// cout << _uwb_vec[0] << endl;
	// cout << _uwb_vec[1] << endl;


	return;
}

void State_Estimation::handle_imu_msg( const sensor_msgs::Imu::ConstPtr& msg ) {
	_orientation = *msg;
	return;
}


sensor_msgs::MagneticField State_Estimation::mag_field_msg(void) const {

	Eigen::Vector3d x_axis;
	Eigen::Vector3d y_axis;
	Eigen::Vector3d z_axis;

	Eigen::Vector3d x_mag;
	Eigen::Vector3d y_mag;
	Eigen::Vector3d z_mag;

	Eigen::Vector3d B_field(1,0,0);

	Eigen::Vector3d uwb_vec[2];

	// find vectors between most recent UWB positions
	if (_uwb_pos[0].size() > 0) {
		uwb_vec[0].x() = _uwb_pos[1].back().x() - _uwb_pos[0].back().x();
		uwb_vec[0].y() = _uwb_pos[1].back().y() - _uwb_pos[0].back().y();
		uwb_vec[0].z() = _uwb_pos[1].back().z() - _uwb_pos[0].back().z();

		uwb_vec[1].x() = _uwb_pos[2].back().x() - _uwb_pos[0].back().x();
		uwb_vec[1].y() = _uwb_pos[2].back().y() - _uwb_pos[0].back().y();
		uwb_vec[1].z() = _uwb_pos[2].back().z() - _uwb_pos[0].back().z();
	}


	// find 3 axes in world frame using vectors between UWB boards
	x_axis = uwb_vec[0].normalized();
	z_axis = uwb_vec[0].cross(uwb_vec[1]).normalized();
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

void State_Estimation::find_pos ( void ) {

	Eigen::Vector3d running_total(0,0,0);



	for (int i = 0; i < _uwb_pos[0].size(); i++) {
		running_total = running_total + (_uwb_pos[1].at(i)+ _uwb_pos[2].at(i))/2;
		// running_total.x() = running_total.x() + (it[1].x() + it[2].x())/2;
		// running_total.y() = running_total.y() + (it[1].y() + it[2].y())/2;
		// running_total.z() = running_total.z() + (it[1].z() + it[2].z())/2;
	}


	_pos_estimate = running_total/_uwb_pos[0].size();

	return;
}

geometry_msgs::Point State_Estimation::pos_msg( void ) const {
	geometry_msgs::Point msg;

	msg.x = _pos_estimate.x();
	msg.y = _pos_estimate.y();
	msg.z = _pos_estimate.z();

	return msg;
}

void State_Estimation::step( void ) {

	// distance travelled with each roll
	double d = 0.25;

	// prediction step
	Eigen::Vector2d _mu_pred;
	Eigen::Matrix2d _sigma_pred;

	if (_rolling == false && (_EKF_roll_step_complete == true)) {
		//cout << "state prediction staying the same" << endl;
		// if not rolling, predicted state is the same as the current state
		_mu_pred = _mu;

	} else if(_rolling == true) {
		//cout << "state prediction staying the same" << endl;
		// if rolling, do not run prediction step
		_EKF_roll_step_complete = false;

	} else if ((_rolling == false) && (_EKF_roll_step_complete == false)) {
		// cout << "propagating state with roll" << endl;
		// if just finished rolling, propagate state using the heading of the roll

		_mu_pred[0] = _mu[0] + d*cos(_u); // propagate x
		_mu_pred[1] = _mu[1] + d*sin(_u); // propagate y

		// propagate covariance
		Eigen::Matrix3d M = _alpha.asDiagonal();

		Eigen::MatrixXd V(2,3);
		V << 	1, 0, -d*sin(_u),
				0, 1, d*cos(_u);

		// Map noise from x,y,heading in body frame to x,y state space
		Eigen::Matrix2d R = V*M*V.transpose();

		_sigma_pred = _sigma + R;

		_EKF_roll_step_complete = true;

		// cout << _mu_pred << endl;
		// cout << _sigma_pred << endl;

	}


	return;
}