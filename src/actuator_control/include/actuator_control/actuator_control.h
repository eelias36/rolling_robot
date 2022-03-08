#ifndef ACTUATOR_CONTROL_H
#define ACTUATOR_CONTROL_H

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"

class Actuators {
	public:
		Actuators();
		virtual ~Actuators();
		void update_command_msgs(void);
		void handle_command(const geometry_msgs::Twist::ConstPtr& msg);
		void handle_faceState(const std_msgs::Int8::ConstPtr& msg);
		void roll_fwd_update(void);
		void roll_side_update(void);
		void home(void);
		void actuator_position_update(void);
		std_msgs::Float64 command_msgs[16];

	protected:
		void evaluate_command();


		double _commanded_pos[16];
		float _counter;
		int _actuator_command_matrix[16][4];
		int _commanded_actuator;
		bool _rolling;
		geometry_msgs::Vector3 _commanded_vel;
		int _faceState;
		float _maxStroke;

};

#endif /* ACTUATOR_CONTROL_H */
