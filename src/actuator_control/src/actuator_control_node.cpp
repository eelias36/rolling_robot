#include <iostream>
#include <string>
#include "ros/ros.h"
#include "actuator_control/actuator_control.h"

using namespace std;

int
main( int argc, char* argv[] ){
	Actuators actuators;

	ros::init( argc, argv, "actuators_node" );
	ros::NodeHandle node_handle;

	ros::Publisher joint_command_publishers[16];
	//actuators.cmd_dir_publisher = node_handle.advertise< std_msgs::Int8 >( "/cmd_dir", 1, true );
	ros::Publisher rolling_publisher = node_handle.advertise< std_msgs::Bool >( "/rolling_state", 1, true );
	ros::Publisher relay_states_publisher = node_handle.advertise< std_msgs::ByteMultiArray >( "/relay_states", 1, true );
	ros::Publisher driver_speed_publisher = node_handle.advertise< std_msgs::Float32 >( "/driver_speed", 1, true );
	ros::Subscriber command_subscriber = node_handle.subscribe( "/cmd_vel", 1, &Actuators::handle_command, &actuators );
	ros::Subscriber faceState_subscriber = node_handle.subscribe( "rolling_robot/face_state", 1, &Actuators::handle_faceState, &actuators );


	double frequency = 10;
	ros::Rate timer( frequency );


	ros::Duration(2.0).sleep();

	while( ros::ok() ){
		//cmd_dir_publisher.publish( actuators.cmd_dir_msg() );
		rolling_publisher.publish( actuators.rolling_msg() );
		ros::spinOnce();
		timer.sleep();
	}
	return 0;
}
