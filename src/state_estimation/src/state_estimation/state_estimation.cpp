#include <iostream>
#include <cmath>
#include "state_estimation/state_estimation.h"


using namespace std;

State_Estimation::State_Estimation(const Eigen::Vector3d& alpha) :  _u(),
																	_mu( Eigen::Vector2d::Zero( 2 ) ),
																	_sigma( Eigen::Matrix2d::Zero( 2, 2 ) ),
																	_alpha( alpha ){

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

	_uwb_vec[0].x() = msg->poses[1].position.x - msg->poses[0].position.x;
	_uwb_vec[0].y() = msg->poses[1].position.y - msg->poses[0].position.y;
	_uwb_vec[0].z() = msg->poses[1].position.z - msg->poses[0].position.z;

	_uwb_vec[1].x() = msg->poses[2].position.x - msg->poses[0].position.x;
	_uwb_vec[1].y() = msg->poses[2].position.y - msg->poses[0].position.y;
	_uwb_vec[1].z() = msg->poses[2].position.z - msg->poses[0].position.z;

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
		cout << "propagating state with roll" << endl;
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

		cout << _mu_pred << endl;
		cout << _sigma_pred << endl;

	}


	return;
}