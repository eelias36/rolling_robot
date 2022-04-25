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
	ros::Subscriber extended_switches_subscriber = node_handle.subscribe( "/extended_switches", 1, &Actuators::handle_extended_switches, &actuators );
	ros::Subscriber retracted_switches_subscriber = node_handle.subscribe( "/retracted_switches", 1, &Actuators::handle_retracted_switches, &actuators );



	double frequency = 10;
	ros::Rate timer( frequency );


	ros::Duration(2.0).sleep();

	while( ros::ok() ){
		actuators.home(10);
		relay_states_publisher.publish( actuators.relay_msg() );
		driver_speed_publisher.publish( actuators.driver_speed_msg() );
		//cmd_dir_publisher.publish( actuators.cmd_dir_msg() );
		// rolling_publisher.publish( actuators.rolling_msg() );

		ros::spinOnce();
		timer.sleep();
	}
	return 0;
}
