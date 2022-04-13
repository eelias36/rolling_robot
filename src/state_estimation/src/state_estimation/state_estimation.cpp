#include <iostream>
#include <cmath>
#include "state_estimation/state_estimation.h"


using namespace std;


State_Estimation::State_Estimation(const Eigen::Vector3d& alpha) :  _u(),
																	_particle_pos_estimate( Eigen::Vector3d::Zero( 3 ) ),
																	_sigma( Eigen::Matrix2d::Zero( 2, 2 ) ),
																	_alpha( alpha ) {

	_pos_estimate << 0, 0, 0.5;

	// create grid of particles 10x10
	for (int i=0; i<P_COUNT; i++) {
		_particles[i].x() = i / (P_COUNT/10) - 5;
		_particles[i].y() = i % (P_COUNT/10) - 5;
		_particles[i].z() = 0.3;

	}

}

State_Estimation::~State_Estimation() {

}

void State_Estimation::find_particle_avg( void ) {
	
	double x_avg = 0;
	double y_avg = 0;
	double z_avg = 0;

	for (int i=0; i<P_COUNT; i++) {
		x_avg += _particles[i].x();
		y_avg += _particles[i].y();
		z_avg += _particles[i].z();
	}
	x_avg = x_avg/P_COUNT;
	y_avg = y_avg/P_COUNT;
	z_avg = z_avg/P_COUNT;

	_particle_pos_estimate.x() = x_avg;
	_particle_pos_estimate.y() = y_avg;
	_particle_pos_estimate.z() = z_avg;

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

sensor_msgs::PointCloud State_Estimation::particle_msg( void ) const {
	sensor_msgs::PointCloud msg;

	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "base_link";

	for (int i=0; i<P_COUNT; i++) {
		msg.points.push_back( geometry_msgs::Point32() );
		msg.points.back().x = _particles[i].x();
		msg.points.back().y = _particles[i].y();
		msg.points.back().z = _particles[i].z();
	}

	return msg;
}

void State_Estimation::callback(const ros::TimerEvent& event) {
	particle_step();
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

geometry_msgs::PoseStamped State_Estimation::pose_msg( void ) const {
	geometry_msgs::PoseStamped msg;


	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "base_link";
	msg.pose.orientation = _orientation.orientation;
	msg.pose.position.x = _particle_pos_estimate.x();
	msg.pose.position.y = _particle_pos_estimate.y();
	msg.pose.position.z = _particle_pos_estimate.z();

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

void State_Estimation::particle_step( void ) {


	if(_rolling == true) {
		//cout << "state prediction staying the same" << endl;
		// if rolling, do not run filter
		_motion_step_complete = false;

	} else if (_rolling == false) {
		// cout << "propagating state with roll" << endl;
		// if just finished rolling, propagate state using the heading of the roll

		
		std::random_device rd{};
	    std::mt19937 gen{rd()};

	    // values near the mean are the most likely
	    // standard deviation affects the dispersion of generated values from the mean
	    std::normal_distribution<> d{0,1};

	   	Eigen::Vector3d particles_bar[P_COUNT];


	    if(_motion_step_complete == false) {

	    	double d = 0.25;	// distance travelled with each roll

	    	// when rolling, add distance rolled to particles
	    	// NEED TO ADD NOISE
			for (int i=0; i<P_COUNT; i++) {
				particles_bar[i].x() = _particles[i].x() + d*cos(_u); // propagate x;
				particles_bar[i].y() = _particles[i].y() + d*sin(_u); // propagate y;
				particles_bar[i].z() = _particles[i].z(); // propagate z

			}
			_motion_step_complete = true;


	    } else {

	    	// if not rolling, predict state to stay the same
			for (int i=0; i<P_COUNT; i++) {
				particles_bar[i] = _particles[i];

			}

	    	// add noise
			for (int i=0; i<P_COUNT; i++) {
			particles_bar[i].x() = particles_bar[i].x() + d(gen)/50;
			particles_bar[i].y() = particles_bar[i].y() + d(gen)/50;
			particles_bar[i].z() = particles_bar[i].z() + d(gen)/500;
			//cout << d(gen) << endl;

			}

	    }


	    // find weights

		// find expected UWB positions
		Eigen::Vector3d uwb1_init(-0.21, -0.21, 0);
		Eigen::Vector3d uwb2_init(0.21, -0.21, 0);
		Eigen::Vector3d uwb3_init(-0.21, 0.21, 0);

		Eigen::Quaterniond q;
		q.w() = _orientation.orientation.w;
		q.x() = _orientation.orientation.x;
		q.y() = _orientation.orientation.y;
		q.z() = _orientation.orientation.z;

		uwb1_init = q * uwb1_init;
		uwb2_init = q * uwb2_init;
		uwb3_init = q * uwb3_init;


		// Eigen::Vector3d uwb1_hat = uwb1_init + particle_avg_bar;
		// Eigen::Vector3d uwb2_hat = uwb2_init + particle_avg_bar;	
		// Eigen::Vector3d uwb3_hat = uwb3_init + particle_avg_bar;

		double prob_x1;
		double prob_y1;
		double prob_z1;
		double prob_x2;
		double prob_y2;
		double prob_z2;
		double prob_x3;
		double prob_y3;
		double prob_z3;
		double var_x = 0.1;
		double var_y = 0.1;
		double var_z = 0.005;
		Eigen::Vector3d uwb1_hat;
		Eigen::Vector3d uwb2_hat;
		Eigen::Vector3d uwb3_hat;

		double weights[P_COUNT];


		for (int i=0; i<P_COUNT; i++) {

			// find predicted uwb position for each particle
			uwb1_hat = uwb1_init + particles_bar[i];
			uwb2_hat = uwb2_init + particles_bar[i];
			uwb3_hat = uwb3_init + particles_bar[i];

			//cout << "meas: " << _uwb_pos[0].back().x() << endl << "pred: " << uwb1_hat.x() << endl;

			// find probability of each predicted vs measured position
			prob_x1 = exp(-0.5 * (_uwb_pos[0].back().x() - uwb1_hat.x())*(_uwb_pos[0].back().x() - uwb1_hat.x()) / var_x);
			prob_y1 = exp(-0.5 * (_uwb_pos[0].back().y() - uwb1_hat.y())*(_uwb_pos[0].back().y() - uwb1_hat.y()) / var_x);
			prob_z1 = exp(-0.5 * (_uwb_pos[0].back().z() - uwb1_hat.z())*(_uwb_pos[0].back().z() - uwb1_hat.z()) / var_x);
			prob_x2 = exp(-0.5 * (_uwb_pos[1].back().x() - uwb2_hat.x())*(_uwb_pos[1].back().x() - uwb2_hat.x()) / var_x);
			prob_y2 = exp(-0.5 * (_uwb_pos[1].back().y() - uwb2_hat.y())*(_uwb_pos[1].back().y() - uwb2_hat.y()) / var_x);
			prob_z2 = exp(-0.5 * (_uwb_pos[1].back().z() - uwb2_hat.z())*(_uwb_pos[1].back().z() - uwb2_hat.z()) / var_x);
			prob_x3 = exp(-0.5 * (_uwb_pos[2].back().x() - uwb3_hat.x())*(_uwb_pos[2].back().x() - uwb3_hat.x()) / var_x);
			prob_y3 = exp(-0.5 * (_uwb_pos[2].back().y() - uwb3_hat.y())*(_uwb_pos[2].back().y() - uwb3_hat.y()) / var_x);
			prob_z3 = exp(-0.5 * (_uwb_pos[2].back().z() - uwb3_hat.z())*(_uwb_pos[2].back().z() - uwb3_hat.z()) / var_x);

			weights[i] = prob_x1 * prob_y1 * prob_z1 * prob_x2 * prob_y2 * prob_z2 * prob_x3 * prob_y3 * prob_z3;

		}

		// create cumulative weights
		double cum_weights[P_COUNT];
		double running_cum = 0;
		//cout <<endl<< "Cum. Weights: " << endl;
		for (int i=0; i<P_COUNT; i++) {
			running_cum += weights[i];
			cum_weights[i] = running_cum;
			//cout << cum_weights[i]<<endl;
		}

		//normalize
		for (int i=0; i<P_COUNT; i++) {
			cum_weights[i] = cum_weights[i] / cum_weights[P_COUNT-1];
			//cout << cum_weights[i]<<endl;
		}

		//cout << cum_weights[99] << endl;

		std::uniform_real_distribution<double> dist(0.0, 1.0);

		double random_num;
		int j;

		// draw new particles
		for (int i=0; i<P_COUNT; i++) {
			random_num = dist(gen);
			//cout << random_num << endl;
			for (j=0; j<P_COUNT; j++) {
				if(cum_weights[j] > random_num) {break;}
			}

			// add drawn particles to list
			_particles[i] = particles_bar[j];
			
			//cout << particles_bar[j] <<endl<<endl;
		}

		find_particle_avg();

	 
	}
	return;
}

void State_Estimation::step( void ) {

	// distance travelled with each roll
	double d = 0.25;

	// prediction step
	Eigen::Vector2d _mu_pred;
	Eigen::Matrix2d _sigma_pred;

	if (_rolling == false && (_motion_step_complete == true)) {
		//cout << "state prediction staying the same" << endl;
		// if not rolling, predicted state is the same as the current state
		_mu_pred = _mu;

	} else if(_rolling == true) {
		//cout << "state prediction staying the same" << endl;
		// if rolling, do not run prediction step
		_motion_step_complete = false;

	} else if ((_rolling == false) && (_motion_step_complete == false)) {
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

		_motion_step_complete = true;

		// cout << _mu_pred << endl;
		// cout << _sigma_pred << endl;

	}


	return;
}