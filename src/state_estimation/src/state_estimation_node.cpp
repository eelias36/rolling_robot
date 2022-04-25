#include <iostream>
#include "ros/ros.h"
#include "state_estimation/state_estimation.h"

using namespace std;

int
main( int argc, char* argv[] ){
	Eigen::Vector3d alpha(0.1,0.1,0.1);
	State_Estimation state_estimation(alpha);

	ros::init( argc, argv, "state_estimation_node" );
	ros::NodeHandle node_handle;

	// ros::Subscriber uwb_subscriber = node_handle.subscribe( "/rolling_robot/uwb_poses",1, &State_Estimation::handle_uwb_msg, &state_estimation );
	ros::Subscriber uwb1_subscriber = node_handle.subscribe( "uwb1",1, &State_Estimation::handle_uwb1_msg, &state_estimation );
	ros::Subscriber uwb2_subscriber = node_handle.subscribe( "uwb2",1, &State_Estimation::handle_uwb2_msg, &state_estimation );
	ros::Subscriber uwb3_subscriber = node_handle.subscribe( "uwb3",1, &State_Estimation::handle_uwb3_msg, &state_estimation );

	ros::Subscriber imu_subscriber = node_handle.subscribe( "/imu/data",1, &State_Estimation::handle_imu_msg, &state_estimation );
	ros::Subscriber heading_subscriber = node_handle.subscribe( "/cmd_heading",1, &State_Estimation::handle_heading_msg, &state_estimation );
	ros::Subscriber rolling_subscriber = node_handle.subscribe( "/rolling_state",1, &State_Estimation::handle_rolling_msg, &state_estimation );
	
	ros::Publisher mag_publisher = node_handle.advertise< sensor_msgs::MagneticField >( "/imu/mag", 1, true );
	ros::Publisher pos_publisher = node_handle.advertise< geometry_msgs::Point >( "/position", 1, true );
	ros::Publisher particle_publisher = node_handle.advertise< sensor_msgs::PointCloud >( "/particles", 1, true );
	ros::Publisher pose_publisher = node_handle.advertise< geometry_msgs::PoseStamped >( "/pose", 1, true );	
	ros::Publisher bad_pos_error_publisher = node_handle.advertise< std_msgs::Float64 >( "/bad_pos_error", 1, true );	
	ros::Publisher good_pos_error_publisher = node_handle.advertise< std_msgs::Float64 >( "/good_pos_error", 1, true );	

	// ros::Timer filter_timer = node_handle.createTimer(ros::Duration(0.1), &State_Estimation::callback, &state_estimation);


	double frequency = 10;
	ros::Rate timer( frequency );

	while( ros::ok() ){
		// mag_publisher.publish( state_estimation.mag_field_msg() );
		// pos_publisher.publish( state_estimation.pos_msg() );
		// particle_publisher.publish( state_estimation.particle_msg() );
		pose_publisher.publish( state_estimation.pose_msg() );
		// bad_pos_error_publisher.publish( state_estimation.bad_estimate_error_msg() );
		// good_pos_error_publisher.publish( state_estimation.particle_estimate_error_msg() );


		ros::spinOnce();
		timer.sleep();
	}
	return 0;
}
