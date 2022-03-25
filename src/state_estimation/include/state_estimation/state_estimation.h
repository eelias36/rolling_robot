#ifndef STATE_ESTIMATION_H
#define STATE_ESTIMATION_H

#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include "std_msgs/Float64.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseArray.h"


class State_Estimation {
	public:
		//State_Estimation();
		State_Estimation(const Eigen::Vector3d& alpha);
		virtual ~State_Estimation();
		void handle_imu_msg( const sensor_msgs::Imu::ConstPtr& msg );
		void handle_uwb_msg( const geometry_msgs::PoseArray::ConstPtr& msg );
		void handle_heading_msg( const std_msgs::Float64::ConstPtr& msg );
		// void handle_direction_msgs( const int msg );
		void step( void );
		// nav_msgs::Odometry estimated_odometry( void )const;
		sensor_msgs::MagneticField mag_field_msg(void) const;

	protected:
		double _u;
		// perception::Observations _z;
		Eigen::Vector2d _mu;
		Eigen::Matrix2d _sigma;
		Eigen::Vector3d _alpha;
		Eigen::Matrix2d _q;
		Eigen::Vector3d _uwb_vec[2];
		bool _rolling;
		bool _EKF_roll_step_complete;
		sensor_msgs::Imu _orientation;

};

#endif /* STATE_ESTIMATION_H */
