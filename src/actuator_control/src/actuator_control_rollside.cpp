#include <iostream>
#include <string>
#include "ros/ros.h"
#include "actuator_control/actuator_control.h"

using namespace std;

int
main( int argc, char* argv[] ){
	Actuators actuators;

	ros::init( argc, argv, "control_rollside" );
	ros::NodeHandle node_handle;

	ros::Publisher joint_command_publishers[16];

	for(int i = 1; i < 17; i++) {
		string topicName = "rolling_robot/joint" + to_string(i) + "_position_controller/command";
		joint_command_publishers[i] = node_handle.advertise< std_msgs::Float64 >( topicName, 1);
	}

	double frequency = 10;
	ros::Rate timer( frequency );

	actuators.home();
	actuators.update_command_msgs();

	ros::Duration(2.0).sleep();

	while( ros::ok() ){
		actuators.roll_side_update();
		actuators.update_command_msgs();

		for(int i = 0; i < 16; i++) {
			joint_command_publishers[i+1].publish( actuators.command_msgs[i] );
		}

		ros::spinOnce();
		timer.sleep();
	}
	return 0;
}
