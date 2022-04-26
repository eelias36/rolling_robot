#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point

import serial
import time

'''
uwb_reader.py
Alex Elias

Sends UWB LEP readings to ROS ropic as a string

Parameters:
    serial_port: e.g. '/dev/ttyS0'
    topic_name:  e.g. 'uwb_serial_front'
'''


class Uwb_reader:
    def __init__(self):
        rospy.init_node('uwb_reader', anonymous=True, disable_signals=True)
        self.serial_port = rospy.get_param('~serial_port')
        topic_name = rospy.get_param('~topic_name')

        # self.serial_port = '/dev/ttyACM0'
        # topic_name = 'uwb_1'

        self.ser = None

        self.pub = rospy.Publisher(topic_name, Point, queue_size=1)
        
        rospy.on_shutdown(self.close_serial_if_active)

    def close_serial_if_active(self):
        if(not(self.ser == None)):
            self.ser.close()

    def start_lep_mode(self):
       # Reset UWB tag so that we're in a known state
        self.ser.write('reset\r'.encode())
        self.ser.write('reset\r'.encode())

        time.sleep(0.1)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        
        ser_bytes = self.ser.readline()
        while not 'dwm>'.encode() in ser_bytes:
            rospy.loginfo('waiting for dwm>')
            rospy.loginfo(ser_bytes)
            self.ser.write('\r\r'.encode())
            ser_bytes = self.ser.readline()
            time.sleep(0.1)
            while(self.ser.in_waiting):
                rospy.loginfo('waiting for dwm> (ser.in_waiting)')
                ser_bytes = self.ser.readline()
                time.sleep(0.1)

        # Tell UWB tag to give us distance readings
        self.ser.write("lep\r".encode())

        ser_bytes = self.ser.readline() 

        # Throw out first reading (has extra "dwm> ")
        ser_bytes = self.ser.readline() 

    def start_reading(self):
        while not rospy.is_shutdown():
            try:
                if(self.ser == None):
                    rospy.loginfo("Trying to reconnect to serial")
                    self.ser = serial.Serial(self.serial_port, 115200, timeout=1, xonxoff=True)
                    rospy.loginfo("Connected to serial")
                    # self.ser.reset_input_buffer()
                    # self.ser.reset_output_buffer()
                    time.sleep(1)
                    self.start_lep_mode()

                ser_bytes = self.ser.readline()
                if(ser_bytes):
                    try:
                        # self.pub.publish(ser_bytes)
                        # print(ser_bytes.decode())

                        # create list of x,y,z,QF
                        pos_list_str = ser_bytes.decode().split(",")
                        pos_list = list()
                        for i in range(1,5):
                            pos_list.append( float( pos_list_str[i] ) )
                        # print( pos_list )

                        # publish msg
                        msg = Point()
                        msg.x = pos_list[0]
                        msg.y = pos_list[1]
                        msg.z = pos_list[2]

                        self.pub.publish( msg )
                    except:
                        rospy.logwarn("Error parsing serial message")

                else:
                    rospy.logwarn("Serial timeout occured")

            except serial.serialutil.SerialException:
                if(not(self.ser == None)):
                    self.ser.close()
                    self.ser = None
                    rospy.logwarn("Disconnecting from serial")
                rospy.logwarn("Serial disconnected")
                time.sleep(0.25)


if __name__ == '__main__':
    uwb_reader = Uwb_reader()
    uwb_reader.start_reading()
