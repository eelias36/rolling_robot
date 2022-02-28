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

	ros::Publisher joint_command_publishers[8];

	for(int i = 1; i < 9; i++) {
		string topicName = "rolling_robot/joint" + to_string(i) + "_position_controller/command";
		joint_command_publishers[i] = node_handle.advertise< std_msgs::Float64 >( topicName, 1);
	}

	/*
	ros::Publisher joint1_command_publisher = node_handle.advertise< std_msgs::Float64 >( "rolling_robot/joint1_position_controller/command", 1);
	ros::Publisher joint2_command_publisher = node_handle.advertise< std_msgs::Float64 >( "rolling_robot/joint2_position_controller/command", 1);
	ros::Publisher joint3_command_publisher = node_handle.advertise< std_msgs::Float64 >( "rolling_robot/joint3_position_controller/command", 1);
	ros::Publisher joint4_command_publisher = node_handle.advertise< std_msgs::Float64 >( "rolling_robot/joint4_position_controller/command", 1);
	ros::Publisher joint5_command_publisher = node_handle.advertise< std_msgs::Float64 >( "rolling_robot/joint5_position_controller/command", 1);
	ros::Publisher joint6_command_publisher = node_handle.advertise< std_msgs::Float64 >( "rolling_robot/joint6_position_controller/command", 1);
	ros::Publisher joint7_command_publisher = node_handle.advertise< std_msgs::Float64 >( "rolling_robot/joint7_position_controller/command", 1);
	ros::Publisher joint8_command_publisher = node_handle.advertise< std_msgs::Float64 >( "rolling_robot/joint8_position_controller/command", 1);
	*/

	double frequency = 10;
	ros::Rate timer( frequency );

	actuators.home();
	actuators.update_command_msgs();

	ros::Duration(2.0).sleep();

	while( ros::ok() ){

		actuators.update_command_msgs();
		for(int i = 0; i < 8; i++) {
			joint_command_publishers[i+1].publish( actuators.command_msgs[i] );
		}
/*
		joint1_command_publisher.publish( actuators.command_msgs[0] );
		joint2_command_publisher.publish( actuators.command_msgs[1] );
		joint3_command_publisher.publish( actuators.command_msgs[2] );
		joint4_command_publisher.publish( actuators.command_msgs[3] );
		joint5_command_publisher.publish( actuators.command_msgs[4] );
		joint6_command_publisher.publish( actuators.command_msgs[5] );
*/
		ros::spinOnce();
		timer.sleep();
	}
	return 0;
}
