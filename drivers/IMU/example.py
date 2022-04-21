#!/usr/bin/
#This code was written by Caleb G. Teague in 2019


from MinIMU_v5_pi import MinIMU_v5_pi
import time


def main():
	#Setup the MinIMU_v5_pi
	IMU = MinIMU_v5_pi()
	
	#Initiate tracking of Yaw on the IMU
	# IMU.trackYaw()

	while True: #Main loop             
		time.sleep(0.1)
		# yaw = IMU.prevYaw[0]
		# print (yaw)
		print("------------")
		print("Accel: ")
		print( IMU.readAccelerometer() )
		print("Gyro: ")
		print( IMU.readGyro() )


if __name__ == "__main__":
	main()


