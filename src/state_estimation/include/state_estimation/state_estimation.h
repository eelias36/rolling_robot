#ifndef STATE_LOCALIZATION_H
#define STATE_LOCALIZATION_H

#include <iostream>
#include <Eigen/Dense>
#include "sensor_msgs/Imu.h"

class State_Localization {
	public:
		State_Localization( const Eigen::VectorXd& mu = Eigen::VectorXd::Zero( 6 ),
		const Eigen::MatrixXd& q = Eigen::MatrixXd::Zero( 3, 3 ) );
		virtual ˜EKF_Localization();
		void handle_imu_msgs( const sensor_msgs::Imu::ConstPtr& msg );
		void handle_uwb_msgs( const nav_msgs::Odometry::ConstPtr& msg );
		void handle_direction_msgs( const int msg );
		void step( const double& dt );
		nav_msgs::Odometry estimated_odometry( void )const;

	protected:
		geometry_msgs::Twist _u;
		perception::Observations _z;
		Eigen::Vector2d _mu_xy;
		Eigen::Quaterniond _mu_quat;
		Eigen::MatrixXd _sigma;
		Eigen::VectorXd _alpha;
		Eigen::MatrixXd _q;

};

#endif /* STATE_LOCALIZATION_H */
