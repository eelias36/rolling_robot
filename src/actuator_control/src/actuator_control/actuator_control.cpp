#include <iostream>
#include <string>
#include "std_msgs/String.h"
#include "actuator_control/actuator_control.h"

using namespace std;

Actuators::Actuators() {
	_counter = 0;
	_faceState = -1;
	_maxStroke = 0.15;
	_rolling = false;
	_commanded_actuator = -1;
	_cmd_dir = -1;
	_retracting = false;
	_homing_complete = false;

	for (int i=0; i<16; i++) {
		_relay_state[i] = 0;
	}
	_driver_speed = 0;

	// first index: current face
	// second index: commanded direction [forward, back, left, right]
	// entry: actuator to be commanded
	for (int i = 0; i < 14; i++){
		for (int j = 0; j < 4; j++){
			_actuator_command_LUT[i][j] = -1;
		}
	}
	_actuator_command_LUT[0][0] = 0;
	_actuator_command_LUT[0][1] = 1;
	_actuator_command_LUT[0][2] = 9;
	_actuator_command_LUT[0][3] = 8;
	_actuator_command_LUT[1][0] = 1;
	_actuator_command_LUT[1][1] = 2;
	_actuator_command_LUT[2][0] = 2;
	_actuator_command_LUT[2][1] = 3;
	_actuator_command_LUT[3][0] = 3;
	_actuator_command_LUT[3][1] = 4;
	_actuator_command_LUT[4][0] = 4;
	_actuator_command_LUT[4][1] = 5;
	_actuator_command_LUT[4][2] = 12;
	_actuator_command_LUT[4][3] = 13;
	_actuator_command_LUT[5][0] = 5;
	_actuator_command_LUT[5][1] = 6;
	_actuator_command_LUT[6][0] = 6;
	_actuator_command_LUT[6][1] = 7;
	_actuator_command_LUT[7][0] = 7;
	_actuator_command_LUT[7][1] = 0;
	_actuator_command_LUT[8][2] = 10;
	_actuator_command_LUT[8][3] = 9;
	_actuator_command_LUT[9][2] = 11;
	_actuator_command_LUT[9][3] = 10;
	_actuator_command_LUT[10][2] = 12;
	_actuator_command_LUT[10][3] = 11;
	_actuator_command_LUT[11][2] = 14;
	_actuator_command_LUT[11][3] = 13;
	_actuator_command_LUT[12][2] = 15;
	_actuator_command_LUT[12][3] = 14;
	_actuator_command_LUT[13][2] = 8;
	_actuator_command_LUT[13][3] = 15;
}

Actuators::~Actuators() {

}

void Actuators::handle_command(const geometry_msgs::Twist::ConstPtr& msg) {
	_commanded_vel = msg->linear;

	if (_faceState != -1 && _rolling == false) {
		evaluate_command();
	}
	return;
}

void Actuators::handle_faceState(const std_msgs::Int8::ConstPtr& msg) {
	_faceState = msg->data;
	return;
}

std_msgs::ByteMultiArray Actuators::relay_msg(void) {
	std_msgs::ByteMultiArray msg;

	for (int i=0; i<16; i++) {
		msg.data.push_back( _relay_state[i] );
	}

	return msg;
}

std_msgs::Float32 Actuators::driver_speed_msg(void) {
	std_msgs::Float32 msg;

	msg.data = _driver_speed;

	return msg;
}

void Actuators::home(const float& speed){

	if (!_retracting && !_homing_complete) {

		bool _test_retracting = true;

		// extend actuators until reach end of travel
		for (int i=0; i<16; i++) {
			if( _out_switch_state[i] == true ) {
				// not at end of travel
				_relay_state[i] = 1;
				_test_retracting = false;
			} else {
				// reaches end of travel
				_relay_state[i] = 0;
			}
		}

		if (_test_retracting) {
			_retracting = true;
		}

		_driver_speed = speed;
	}

	if(_retracting && !_homing_complete) {
		
		_driver_speed = -speed;

		bool _test_complete = true;

		// retract actuators until reach end of travel
		for (int i=0; i<16; i++) {
			if( _in_switch_state[i] == false ) {
				// not at end of travel
				_relay_state[i] = 1;
				_test_complete = false;
			} else {
				// reaches end of travel
				_relay_state[i] = 0;
			}
		}

		if (_test_complete) {
			_homing_complete = true;
		}
	}
	return;
}

void Actuators::update(void){


	if (_commanded_actuator != -1) {

		for (int i=0; i<16; i++) {
			_relay_state[i] = 0;
		}
		
		_relay_state[_commanded_actuator] = 1;

		if ( !_retracting && _out_switch_state[_commanded_actuator] == true ) {
			_driver_speed = 20;
		} else if ( !_retracting &&  _out_switch_state[_commanded_actuator] == false ) {
			_retracting = true;
			_driver_speed = -20;
		} else if ( _retracting && _in_switch_state[_commanded_actuator] == false ) {
			_driver_speed = -20;
		} else if ( _retracting && _in_switch_state[_commanded_actuator] == true ) {
			_driver_speed = 0;
			_rolling = false;
			_commanded_actuator = -1;
		}

	} else {
		for (int i=0; i<16; i++) {
			_relay_state[i] = 0;
		}
		_driver_speed = 0;
	}

	return;
}

void Actuators::evaluate_command(void) {
	if (_commanded_vel.x > 0) {
		_cmd_dir = 0;
		_commanded_actuator = _actuator_command_LUT[_faceState][0];
	} else if (_commanded_vel.x < 0) {
		_cmd_dir = 1;
		_commanded_actuator = _actuator_command_LUT[_faceState][1];
	} else if (_commanded_vel.y > 0) {
		_cmd_dir = 2;
		_commanded_actuator = _actuator_command_LUT[_faceState][2];
	} else if(_commanded_vel.y < 0) {
		_cmd_dir = 3;
		_commanded_actuator = _actuator_command_LUT[_faceState][3];
	}

	_rolling = (_commanded_actuator != -1);


	if (_rolling) {
		std_msgs::Int8 msg;

		msg.data = _cmd_dir;

		//cmd_dir_publisher.publish( msg );
	}

	if (!_rolling) {
		_cmd_dir = -1;
	}
	return;
}

void Actuators::actuator_position_update_sim(void) {

	// command all other actuators to 0
	for(int i = 0; i < 16; i++) {
		_commanded_pos[i] = 0;
	}

	// cout << "____________________________" << endl;
	// cout << "Face State: " << _faceState << endl;
	// cout << "Rolling: " << _rolling << endl;
	// cout << "Commanded Actuator: " << _commanded_actuator << endl;
	
	if (_rolling == true) {
		if ( (_counter > 60) | (_counter < 0) ) {
			_rolling = false;
			_commanded_actuator = -1;
			_counter = 0;
		} else if ( _counter < 30 ) {
			_commanded_pos[_commanded_actuator] = _counter/30.0*_maxStroke;
			_counter++;
		} else if ( _counter <= 60 ){
			_commanded_pos[_commanded_actuator] = _maxStroke - (_counter-30)/30.0*_maxStroke;
			_counter++;
		}
	}
}

void Actuators::home_sim(void) {
	for( int i=0;i<16;i++ ) {
		_commanded_pos[i] = 0;
	}
	return;
}

void Actuators::update_command_msgs_sim(void) {
	for( int i=0;i<16;i++ ) {
		command_msgs[i].data = (float) _commanded_pos[i];
	}
	return;
}

void Actuators::roll_fwd_update_sim(void) {
	float static stroke = 0.5;

	if ( (_counter > 540) | (_counter < 0) ) {
		_counter = 0;
	} else if ( _counter < 60 ) {
		_commanded_pos[0] = _counter/60.0*stroke;
	} else if ( _counter < 120 ){
		_commanded_pos[0] = stroke - (_counter-60)/60.0*stroke;
		_commanded_pos[1] = (_counter-60)/60.0*stroke;
	} else if ( _counter < 180 ){
		_commanded_pos[1] = stroke - (_counter-120)/60.0*stroke;
		_commanded_pos[2] = (_counter-120)/60.0*stroke;
	} else if ( _counter < 240 ){
		_commanded_pos[2] = stroke - (_counter-180)/60.0*stroke;
		_commanded_pos[3] = (_counter-180)/60.0*stroke;
	} else if ( _counter < 300 ){
		_commanded_pos[3] = stroke - (_counter-240)/60.0*stroke;
		_commanded_pos[4] = (_counter-240)/60.0*stroke;
	} else if ( _counter < 360 ){
		_commanded_pos[4] = stroke - (_counter-300)/60.0*stroke;
		_commanded_pos[5] = (_counter-300)/60.0*stroke;
	} else if ( _counter < 420 ){
		_commanded_pos[5] = stroke - (_counter-360)/60.0*stroke;
		_commanded_pos[6] = (_counter-360)/60.0*stroke;
	} else if ( _counter < 480 ){
		_commanded_pos[6] = stroke - (_counter-420)/60.0*stroke;
		_commanded_pos[7] = (_counter-420)/60.0*stroke;
	} else {
		_commanded_pos[7] = stroke - (_counter-480)/60.0*stroke;
	}

	for( int i=0;i<16;i++ ) {
		cout<< "actuator " + to_string(i+1) + ": " + to_string(_commanded_pos[i]) << endl;
	}
	cout << endl;

	_counter++;
	return;
}

void Actuators::roll_side_update_sim(void) {
	float static stroke = 0.5;

	if ( (_counter > 540) | (_counter < 0) ) {
		_counter = 0;
	} else if ( _counter < 60 ) {
		_commanded_pos[8] = _counter/60.0*stroke;
	} else if ( _counter < 120 ){
		_commanded_pos[8] = stroke - (_counter-60)/60.0*stroke;
		_commanded_pos[9] = (_counter-60)/60.0*stroke;
	} else if ( _counter < 180 ){
		_commanded_pos[9] = stroke - (_counter-120)/60.0*stroke;
		_commanded_pos[10] = (_counter-120)/60.0*stroke;
	} else if ( _counter < 240 ){
		_commanded_pos[10] = stroke - (_counter-180)/60.0*stroke;
		_commanded_pos[11] = (_counter-180)/60.0*stroke;
	} else if ( _counter < 300 ){
		_commanded_pos[11] = stroke - (_counter-240)/60.0*stroke;
		_commanded_pos[12] = (_counter-240)/60.0*stroke;
	} else if ( _counter < 360 ){
		_commanded_pos[12] = stroke - (_counter-300)/60.0*stroke;
		_commanded_pos[13] = (_counter-300)/60.0*stroke;
	} else if ( _counter < 420 ){
		_commanded_pos[13] = stroke - (_counter-360)/60.0*stroke;
		_commanded_pos[14] = (_counter-360)/60.0*stroke;
	} else if ( _counter < 480 ){
		_commanded_pos[14] = stroke - (_counter-420)/60.0*stroke;
		_commanded_pos[15] = (_counter-420)/60.0*stroke;
	} else {
		_commanded_pos[15] = stroke - (_counter-480)/60.0*stroke;
	}

	for( int i=0;i<16;i++ ) {
		cout<< "actuator " + to_string(i+1) + ": " + to_string(_commanded_pos[i]) << endl;
	}
	cout << endl;

	_counter++;
	return;
}

std_msgs::Bool Actuators::rolling_msg(void) {
	std_msgs::Bool msg;

	msg.data = _rolling;
	return msg;
}