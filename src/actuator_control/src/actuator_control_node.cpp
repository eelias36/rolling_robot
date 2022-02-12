#include <iostream>
#include "ros/ros.h"
#include "actuator_control/actuator_control.h"

using namespace std;

int
main( int argc, char* argv[] ){
	Actuators actuators;

	ros::init( argc, argv, "actuators_node" );
	ros::NodeHandle node_handle;

	ros::Publisher joint1_command_publisher = node_handle.advertise< std_msgs::Float64 >( "rolling_robot/joint1_position_controller/command", 1);
	
	double frequency = 0.1;
	ros::Rate timer( frequency );

	while( ros::ok() ){
		actuators.update_commands();
		actuators.update_command_msgs();
		joint1_command_publisher.publish( actuators.command_msgs[0] );

		ros::spinOnce();
		timer.sleep();
	}
	return 0;
}
