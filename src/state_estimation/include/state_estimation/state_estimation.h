#ifndef STATE_ESTIMATION_H
#define STATE_ESTIMATION_H

#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <random>
#include "ros/ros.h"
#include <deque>
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Point32.h"

#define P_COUNT 1000


class State_Estimation {
	public:
		//State_Estimation();
		State_Estimation(const Eigen::Vector3d& alpha);
		virtual ~State_Estimation();
		void handle_imu_msg( const sensor_msgs::Imu::ConstPtr& msg );
		void handle_uwb_msg( const geometry_msgs::PoseArray::ConstPtr& msg );
		void handle_heading_msg( const std_msgs::Float64::ConstPtr& msg );
		void handle_rolling_msg( const std_msgs::Bool::ConstPtr& msg );
		void callback(const ros::TimerEvent& event);
		// void handle_direction_msgs( const int msg );
		void step( void );
		void particle_step( void );
		void find_pos ( void ) ;
		// nav_msgs::Odometry estimated_odometry( void )const;
		sensor_msgs::MagneticField mag_field_msg( void ) const;
		geometry_msgs::Point pos_msg( void ) const;
		sensor_msgs::PointCloud particle_msg( void ) const;
		geometry_msgs::PoseStamped pose_msg( void ) const;


	protected:
		double _u;
		// perception::Observations _z;
		Eigen::Vector3d _particle_pos_estimate;	
		Eigen::Vector2d _mu;
		Eigen::Matrix2d _sigma;
		Eigen::Vector3d _alpha;
		Eigen::Matrix2d _q;
		//Eigen::Vector3d _uwb_vec[2];
		bool _rolling;
		bool _EKF_roll_step_complete;
		sensor_msgs::Imu _orientation;
		std::deque <Eigen::Vector3d> _uwb_pos[3];
		Eigen::Vector3d _pos_estimate;
		Eigen::Vector3d _particles[P_COUNT];
		void find_particle_avg( void );

};

#endif /* STATE_ESTIMATION_H */
