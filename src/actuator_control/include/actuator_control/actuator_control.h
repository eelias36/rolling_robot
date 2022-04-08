#ifndef ACTUATOR_CONTROL_H
#define ACTUATOR_CONTROL_H

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "std_msgs/ByteMultiArray.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"

class Actuators {
	public:
		Actuators();
		virtual ~Actuators();
		void update_command_msgs_sim(void);
		void handle_command(const geometry_msgs::Twist::ConstPtr& msg);
		void handle_faceState(const std_msgs::Int8::ConstPtr& msg);
		void roll_fwd_update_sim(void);
		void roll_side_update_sim(void);
		void home_sim(void);
		void home(const float& speed);
		void update(void);
		void actuator_position_update_sim(void);
		std_msgs::Float64 command_msgs[16];
		std_msgs::Bool rolling_msg(void);
		ros::Publisher cmd_dir_publisher;
		std_msgs::ByteMultiArray relay_msg(void);
		std_msgs::Float32 driver_speed_msg(void);

	protected:
		void evaluate_command();

		double _commanded_pos[16];
		float _counter;
		int _actuator_command_LUT[16][4];
		int _commanded_actuator;
		bool _rolling;
		geometry_msgs::Vector3 _commanded_vel;
		int _faceState;
		float _maxStroke;
		int _cmd_dir;
		bool _relay_state[16];
		bool _in_switch_state[16];
		bool _out_switch_state[16];
		float _driver_speed;
		bool _retracting;
		bool _homing_complete;

};

#endif /* ACTUATOR_CONTROL_H */
