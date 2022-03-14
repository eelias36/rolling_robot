#ifndef STATE_ESTIMATION_H
#define STATE_ESTIMATION_H

#include <iostream>
#include <Eigen/Dense>
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/PoseArray.h"


class State_Estimation {
	public:
		State_Estimation();
		// State_Estimation( const Eigen::VectorXd& mu = Eigen::VectorXd::Zero( 6 ),
		// 	const Eigen::MatrixXd& q = Eigen::MatrixXd::Zero( 3, 3 ) );
		virtual ~State_Estimation();
		// void handle_imu_msg( const sensor_msgs::Imu::ConstPtr& msg );
		void handle_uwb_msg( const geometry_msgs::PoseArray::ConstPtr& msg );
		// void handle_direction_msgs( const int msg );
		// void step( const double& dt );
		// nav_msgs::Odometry estimated_odometry( void )const;
		sensor_msgs::MagneticField mag_field_msg(void) const;

	protected:
		// geometry_msgs::Twist _u;
		// perception::Observations _z;
		// Eigen::Vector2d _mu_xy;
		// Eigen::Quaterniond _mu_quat;
		// Eigen::MatrixXd _sigma;
		// Eigen::VectorXd _alpha;
		// Eigen::MatrixXd _q;
		Eigen::Vector3d _uwb_vec[2];
		// sensor_msgs::Imu _imu_msg_no_mag;

};

#endif /* STATE_ESTIMATION_H */
