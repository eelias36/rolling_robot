#ifndef ACTUATOR_CONTROL_H
#define ACTUATOR_CONTROL_H

#include "ros/ros.h"
#include "std_msgs/Float64.h"

class Actuators {
	public:
		Actuators();
		virtual ~Actuators();
		void update_command_msgs(void);
		void roll_fwd_update(void);
		void home(void);
		std_msgs::Float64 command_msgs[16];

	protected:
		double _commanded_pos[16];
		float _counter;

};

#endif /* ACTUATOR_CONTROL_H */
