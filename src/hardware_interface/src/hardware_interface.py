import rospy
from std_msgs.msg import String, ByteMultiArray, Float32
from sensor_msgs.msg import Imu
import numpy as np
from pyquaternion import Quaternion
from MinIMU_v5_pi import MinIMU_v5_pi
import time



class Hardware_interface:
    def __init__(self):
        rospy.init_node('hardware_interface_node', anonymous=True, disable_signals=True)

        rospy.Subscriber("/relay_states", ByteMultiArray, self.relay_states_callback, queue_size=1)
        rospy.Subscriber("/driver_speed", Float32, self.driver_speed_callback, queue_size=1)

        self.imu_pub = rospy.Publisher("/imu/data_raw", Imu, queue_size=1)
        self.IMU = MinIMU_v5_pi()
        self.imu_q = Quaternion(-0.2705981, -0.2705981, -0.6532815, -0.6532815)



    def relay_states_callback(self, data):
        return

    def driver_speed_callback(self, data):
        return
        
    def start_running(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.imu_pub.publish( self.get_imu_msg() )
            rate.sleep()

    def get_imu_msg(self):
        imu_msg = Imu()

        accel = np.array( IMU.readAccelerometer() )
        accel = imu_q.rotate(accel)
        # print(accel)
        # print("Gyro: ")
        gyro = np.array( IMU.readGyro() )
        gyro = imu_q.rotate(gyro)
        # print(gyro)

        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = 'base_link'
        imu_msg.angular_velocity.x = gyro[0]
        imu_msg.angular_velocity.y = gyro[1]
        imu_msg.angular_velocity.z = gyro[2]

        imu_msg.linear_acceleration.x = accel[0]
        imu_msg.linear_acceleration.y = accel[1]
        imu_msg.linear_acceleration.z = accel[2]

        return imu_msg


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