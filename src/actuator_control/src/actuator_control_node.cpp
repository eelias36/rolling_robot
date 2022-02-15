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
	ros::Publisher joint2_command_publisher = node_handle.advertise< std_msgs::Float64 >( "rolling_robot/joint2_position_controller/command", 1);
	ros::Publisher joint3_command_publisher = node_handle.advertise< std_msgs::Float64 >( "rolling_robot/joint3_position_controller/command", 1);
	ros::Publisher joint4_command_publisher = node_handle.advertise< std_msgs::Float64 >( "rolling_robot/joint4_position_controller/command", 1);
	ros::Publisher joint5_command_publisher = node_handle.advertise< std_msgs::Float64 >( "rolling_robot/joint5_position_controller/command", 1);
	ros::Publisher joint6_command_publisher = node_handle.advertise< std_msgs::Float64 >( "rolling_robot/joint6_position_controller/command", 1);
	
	double frequency = 30;
	ros::Rate timer( frequency );

	while( ros::ok() ){
		actuators.update_commands();
		actuators.update_command_msgs();
		joint1_command_publisher.publish( actuators.command_msgs[0] );
		joint2_command_publisher.publish( actuators.command_msgs[1] );
		joint3_command_publisher.publish( actuators.command_msgs[2] );
		joint4_command_publisher.publish( actuators.command_msgs[3] );
		joint5_command_publisher.publish( actuators.command_msgs[4] );
		joint6_command_publisher.publish( actuators.command_msgs[5] );

		ros::spinOnce();
		timer.sleep();
	}
	return 0;
}
