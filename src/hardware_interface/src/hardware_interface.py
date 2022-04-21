import rospy
from std_msgs.msg import String, ByteMultiArray, Float32
from sensor_msgs.msg import Imu



class Hardware_interface:
    def __init__(self):
        rospy.init_node('hardware_interface_node', anonymous=True, disable_signals=True)

        rospy.Subscriber("/relay_states", ByteMultiArray, self.relay_states_callback, queue_size=1)
        rospy.Subscriber("/driver_speed", Float32, self.driver_speed_callback, queue_size=1)

        self.imu_pub = rospy.Publisher("/imu/data_raw", Imu, queue_size=1)



    def relay_states_callback(self, data):
        return

    def driver_speed_callback(self, data):
        return
        
    def start_running(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.imu_pub.publish( self.imu_msg() )
            rate.sleep()


# def talker():
#     pub = rospy.Publisher('chatter', String, queue_size=10)
#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         hello_str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()

if __name__ == '__main__':
    hardware_interface = Hardware_interface()
    hardware_interface.start_running()