#!/usr/bin/
#This code was written by Caleb G. Teague in 2019

import numpy as np
from pyquaternion import Quaternion
from MinIMU_v5_pi import MinIMU_v5_pi
import time


def main():
	#Setup the MinIMU_v5_pi
	IMU = MinIMU_v5_pi()
	
	#Initiate tracking of Yaw on the IMU
	# IMU.trackYaw()
	# (0.2705981, -0.2705981, 0.6532815, -0.6532815)
	q = Quaternion(-0.2705981, -0.2705981, -0.6532815, -0.6532815)
	# q = Quaternion()

	while True: #Main loop             
		time.sleep(0.1)
		# yaw = IMU.prevYaw[0]
		# print (yaw)
		print("------------")
		print("Accel: ")
		accel = np.array( IMU.readAccelerometer() )
		accel = q.rotate(accel)
		print(accel)
		# print( IMU.readAccelerometer() )
		print("Gyro: ")
		gyro = np.array( IMU.readGyro() )
		gyro = q.rotate(gyro)
		print(gyro)


if __name__ == "__main__":
	main()