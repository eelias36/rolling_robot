#include <iostream>
#include "ros/ros.h"
#include "state_estimation/state_estimation.h"

using namespace std;

int
main( int argc, char* argv[] ){
	State_Estimation state_estimation;

	ros::init( argc, argv, "state_estimation_node" );
	ros::NodeHandle node_handle;

	ros::Subscriber uwb_subscriber = node_handle.subscribe( "/rolling_robot/uwb_poses",1, &State_Estimation::handle_uwb_msg, &state_estimation );
	ros::Publisher mag_publisher = node_handle.advertise< sensor_msgs::MagneticField >( "/imu/mag", 1, true );

	double frequency = 10;
	ros::Rate timer( frequency );

	while( ros::ok() ){
		mag_publisher.publish( state_estimation.mag_field_msg() );

		ros::spinOnce();
		timer.sleep();
	}
	return 0;
}
