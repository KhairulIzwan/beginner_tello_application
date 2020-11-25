#!/usr/bin/env python

################################################################################
## {Description}: 
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

# import the necessary Python packages
from __future__ import print_function
import sys
import cv2
import time
import numpy as np
import imutils
import random
import apriltag

# import the necessary ROS packages
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from std_msgs.msg import UInt8
from std_msgs.msg import Empty

from geometry_msgs.msg import Twist

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Imu

from nav_msgs.msg import Odometry

from tello_driver.msg import TelloStatus

from beginner_tello_application.msg import objCenter as objCoord
from beginner_tello_application.msg import apriltagData

from beginner_tello_application.pid import PID
from beginner_tello_application.makesimpleprofile import map as mapped

import rospy

class AprilTagTracking:
	def __init__(self):
		self.telloCmdVel = Twist()
		self.land = Empty()
		self.flip = UInt8()

		self.isApriltag_received = False

		self.stateTwo = False
		self.stateFlip = False
		self.stateLand = False

		self.MAX_LIN_VEL = 1.5
		self.MAX_LIN_VEL_S2 = 0.4
		self.MAX_LIN_VEL_S = 0.6
		self.MAX_ANG_VEL = 1.0

		# set PID values for pan
		self.panP = 3
		self.panI = 0
		self.panD = 0.001

		# set PID values for tilt
		self.tiltP = 3
		self.tiltI = 0
		self.tiltD = 0.001

		# set PID values for yaw
		self.yawP = 3
		self.yawI = 0
		self.yawD = 0.001

		# set PID values for distance
		self.distanceP = 3
		self.distanceI = 0
		self.distanceD = 0.001

		# set PID values for height_m
		self.heightP = 3
		self.heightI = 0
		self.heightD = 0.001

		# create a PID and initialize it
		self.panPID = PID(self.panP, self.panI, self.panD)
		self.tiltPID = PID(self.tiltP, self.tiltI, self.tiltD)
		self.yawPID = PID(self.yawP, self.yawI, self.yawD)
		self.distancePID = PID(self.distanceP, self.distanceI, self.distanceD)
		self.heightPID = PID(self.heightP, self.heightI, self.heightD)

		self.panPID.initialize()
		self.tiltPID.initialize()
		self.yawPID.initialize()
		self.distancePID.initialize()
		self.heightPID.initialize()

		rospy.logwarn("AprilTag3 Tracking Node [ONLINE]...")

		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)

		# Subscribe to CompressedImage msg
		self.telloCameraInfo_topic = "/tello/camera/camera_info"
		self.telloCameraInfo_sub = rospy.Subscriber(
						self.telloCameraInfo_topic, 
						CameraInfo, 
						self.cbCameraInfo
						)
						
		# Subscribe to apriltagData msg
		self.apriltagData_topic = "/apriltagData"
		self.apriltagData_sub = rospy.Subscriber(
					self.apriltagData_topic, 
					apriltagData, 
					self.cbAprilTagData
					)
					
		# Subscribe to TelloStatus msg
		self.telloStatus_topic = "/tello/status"
		self.telloStatus_sub = rospy.Subscriber(
					self.telloStatus_topic, 
					TelloStatus, 
					self.cbTelloStatus
					)

		# Subscribe to Odometry msg
		self.telloOdom_topic = "/tello/odom"
		self.telloOdom_sub = rospy.Subscriber(
					self.telloOdom_topic, 
					Odometry, 
					self.cbTelloOdometry
					)

		# Subscribe to PoseWithCovariance msg
		self.telloIMU_topic = "/tello/imu"
		self.telloIMU_sub = rospy.Subscriber(
					self.telloIMU_topic, 
					Imu, 
					self.cbTelloIMU
					)

		# Subscribe to objCenter msg
		self.objCoord_topic = "/apriltagCoord"
		self.objCoord_sub = rospy.Subscriber(
					self.objCoord_topic, 
					objCoord, 
					self.cbObjCoord
					)
					
		# Publish to Twist msg
		self.telloCmdVel_topic = "/tello/cmd_vel"
		self.telloCmdVel_pub = rospy.Publisher(
					self.telloCmdVel_topic, 
					Twist, 
					queue_size=10
					)
					
		# Publish to Empty msg
		self.telloLand_topic = "/tello/land"
		self.telloLand_pub = rospy.Publisher(
					self.telloLand_topic, 
					Empty, 
					queue_size=10)
					
		# Publish to UInt8 msg
		self.telloFlip_topic = "/tello/flip"
		self.telloFlip_pub = rospy.Publisher(
					self.telloFlip_topic, 
					UInt8, 
					queue_size=10)
					
		# Allow up to one second to connection
		rospy.sleep(1)
		
	# Is AprilTag3 Data Received?
	def cbAprilTagData(self, msg):
		try:
			self.apriltagStatus = msg.apriltagStatus
			self.apriltagID = msg.apriltagID
			self.apriltagCenter_x = msg.apriltagCenter_x
			self.apriltagCenter_y = msg.apriltagCenter_y
			self.apriltagHomography_00 =msg.apriltagHomography_00
			self.apriltagHomography_01 = msg.apriltagHomography_01
			self.apriltagHomography_02 = msg.apriltagHomography_02
			self.apriltagHomography_10 = msg.apriltagHomography_10
			self.apriltagHomography_11 = msg.apriltagHomography_11
			self.apriltagHomography_12 = msg.apriltagHomography_12
			self.apriltagHomography_20 = msg.apriltagHomography_20
			self.apriltagHomography_21 = msg.apriltagHomography_21
			self.apriltagHomography_22 = msg.apriltagHomography_22
			self.apriltagCorner_x1 = msg.apriltagCorner_x1
			self.apriltagCorner_y1 = msg.apriltagCorner_y1
			self.apriltagCorner_x2 = msg.apriltagCorner_x2
			self.apriltagCorner_y2 = msg.apriltagCorner_y2
			self.apriltagCorner_x3 = msg.apriltagCorner_x3
			self.apriltagCorner_y3 = msg.apriltagCorner_y3
			self.apriltagCorner_x4 = msg.apriltagCorner_x4
			self.apriltagCorner_y4 = msg.apriltagCorner_y4
			self.apriltagDistance = msg.apriltagDistance
			
		except AttributeError as e:
			print(e)

		if self.apriltagStatus is not None:
			self.apriltagStatus_received = True
		else:
			self.apriltagStatus_received = False
			
	def cbmissionID(self, msg):
		try:
			self.missionID = msg.data
		except AttributeError as e:
			print(e)
			
		if self.missionID is not None:
			self.missionID_received = True
		else:
			self.missionID_received = False
			
	# Convert image to OpenCV format
	def cbCameraInfo(self, msg):
		self.imgWidth = msg.width
		self.imgHeight = msg.height
		
	# Get TelloIMU info
	def cbTelloIMU(self, msg):
		self.orientationXIMU = msg.orientation.x
		self.orientationYIMU = msg.orientation.y
		self.orientationZIMU = msg.orientation.z
		self.orientationWIMU = msg.orientation.w
		
		self.angularXIMU = msg.angular_velocity.x
		self.angularYIMU = msg.angular_velocity.y
		self.angularZIMU = msg.angular_velocity.z
		
		self.linearXIMU = msg.linear_acceleration.x
		self.linearYIMU = msg.linear_acceleration.y
		self.linearZIMU = msg.linear_acceleration.z
		
	# Get TelloOdometry info
	def cbTelloOdometry(self, msg):
		self.poseX = msg.pose.pose.position.x
		self.poseY = msg.pose.pose.position.y
		self.poseZ = msg.pose.pose.position.z
		self.orientationX = msg.pose.pose.orientation.x
		self.orientationY = msg.pose.pose.orientation.y
		self.orientationZ = msg.pose.pose.orientation.z
		self.orientationW = msg.pose.pose.orientation.w
		
		self.linearX = msg.twist.twist.linear.x
		self.linearY = msg.twist.twist.linear.y
		self.linearZ = msg.twist.twist.linear.z
		self.angularX = msg.twist.twist.angular.x
		self.angularY = msg.twist.twist.angular.y
		self.angularZ = msg.twist.twist.angular.z
		
	# Get TelloStatus info
	def cbTelloStatus(self, msg):
		# Non-negative; calibrated to takeoff altitude; auto-calib if 
		# falls below takeoff height; inaccurate near ground
		self.height_m = msg.height_m
		
		self.speed_northing_mps = msg.speed_northing_mps
		self.speed_easting_mps = msg.speed_easting_mps
		self.speed_horizontal_mps = msg.speed_horizontal_mps
		self.speed_vertical_mps = msg.speed_vertical_mps
		
		self.flight_time_sec = msg.flight_time_sec
		
		self.imu_state = msg.imu_state
		self.pressure_state = msg.pressure_state
		self.down_visual_state = msg.down_visual_state
		self.power_state = msg.power_state
		self.battery_state = msg.battery_state
		self.gravity_state = msg.gravity_state
		self.wind_state = msg.wind_state
		
		self.imu_calibration_state = msg.imu_calibration_state
		self.battery_percentage = msg.battery_percentage
		self.drone_fly_time_left_sec = msg.drone_fly_time_left_sec
		self.drone_battery_left_sec = msg.drone_battery_left_sec
		
		self.is_flying = msg.is_flying
		self.is_on_ground = msg.is_on_ground
		# is_em_open True in flight, False when landed
		self.is_em_open = msg.is_em_open
		self.is_drone_hover = msg.is_drone_hover
		self.is_outage_recording = msg.is_outage_recording
		self.is_battery_low = msg.is_battery_low
		self.is_battery_lower = msg.is_battery_lower
		self.is_factory_mode = msg.is_factory_mode
		
		# flymode=1: landed; =6: flying
		self.fly_mode = msg.fly_mode
		self.throw_takeoff_timer_sec = msg.throw_takeoff_timer_sec
		self.camera_state = msg.camera_state
		
		self.electrical_machinery_state = msg.electrical_machinery_state
		
		self.front_in = msg.front_in
		self.front_out = msg.front_out
		self.front_lsc = msg.front_lsc
		
		self.temperature_height_m = msg.temperature_height_m
		
		self.cmd_roll_ratio = msg.cmd_roll_ratio
		self.cmd_pitch_ratio = msg.cmd_pitch_ratio
		self.cmd_yaw_ratio = msg.cmd_yaw_ratio
		self.cmd_vspeed_ratio = msg.cmd_vspeed_ratio
		self.cmd_fast_mode = msg.cmd_fast_mode
		
	# show information callback
	def cbPIDerrCenter(self):
		self.panErr, self.panOut = self.cbPIDprocess(self.panPID, self.objectCoordX, self.imgWidth // 2)
		self.tiltErr, self.tiltOut = self.cbPIDprocess(self.tiltPID, self.objectCoordY, self.imgHeight // 2)
		self.yawErr, self.yawOut = self.cbPIDprocess(self.yawPID, self.objectCoordX, self.imgWidth // 2)
		# TODO: How to fixed the distance
		self.distanceErr, self.distanceOut = self.cbPIDprocess(self.distancePID, self.isApriltagDistance[0], 1.5)

	def cbPIDprocess(self, pid, objCoord, centerCoord):
		# calculate the error
		error = centerCoord - objCoord

		# update the value
		output = pid.update(error)

		return error, output

	def cbAprilTagTracking(self):
		# AprilTag3 Status Received: True
		if self.apriltagStatus_received:
			if self.missionID_received:
				# Find index of missionCount
				self.cbFindList()
				# is AprilTag3 detected? : True
				if self.apriltagStatus:
			
					# is AprilTag3 detected listed? : False
					if not self.apriltagID:
						pass
						
					# is AprilTag3 detected listed? : True
					else:
				
						# is AprilTag3 detected listed it NOT 0 : Takeoff or 1 : Land
						if self.apriltagID[self.index] == 1:
							# Calculate the PID error
							self.cbPIDerrCenter()

							# Mapped the speed according to PID error calculated
							panSpeed = mapped(
								abs(self.panOut), 
								0, 
								self.imgWidth // 2, 
								0, 
								self.MAX_LIN_VEL_S2)

							tiltSpeed = mapped(
								abs(self.tiltOut), 
								0, 
								self.imgHeight // 2, 
								0, 
								self.MAX_LIN_VEL)

							yawSpeed = mapped(
								abs(self.yawOut), 
								0, 
								self.imgWidth // 2, 
								0, 
								self.MAX_ANG_VEL)

							distanceSpeed = mapped(
								abs(self.distanceOut), 
								0, 
								1.5, 
								0, 
								self.MAX_LIN_VEL_S)

							# Constrain the speed : speed in range
							panSpeed = self.constrain(
								panSpeed, 
								-self.MAX_LIN_VEL_S2, 
								self.MAX_LIN_VEL_S2)

							tiltSpeed = self.constrain(
								tiltSpeed, 
								-self.MAX_LIN_VEL, 
								self.MAX_LIN_VEL)

							yawSpeed = self.constrain(
								yawSpeed, 
								-self.MAX_ANG_VEL, 
								self.MAX_ANG_VEL)

							distanceSpeed = self.constrain(
								distanceSpeed, 
								-self.MAX_LIN_VEL_S, 
								self.MAX_LIN_VEL_S)

							if self.panOut < -10:
								self.telloCmdVel.linear.x = panSpeed
							elif self.panOut > 10:
								self.telloCmdVel.linear.x = -panSpeed
							else:
								self.telloCmdVel.linear.x = 0

							if self.tiltOut > 10:
								self.telloCmdVel.linear.z = tiltSpeed
							elif self.tiltOut < -10:
								self.telloCmdVel.linear.z = -tiltSpeed
							else:
								self.telloCmdVel.linear.z = 0

							if self.yawOut > 10:
								self.telloCmdVel.angular.z = -yawSpeed
							elif self.yawOut < -10:
								self.telloCmdVel.angular.z = yawSpeed
							else:
								self.telloCmdVel.angular.z = 0

							if self.distanceOut <= 0:
								self.telloCmdVel.linear.y = distanceSpeed
							else:
								self.telloCmdVel.linear.y = 0
								self.stateTwo = True
								rospy.logerr("ARRIVED...")

	#						self.telloCmdVel.linear.x = 0.0
	#						self.telloCmdVel.linear.y = 0.0
	#						self.telloCmdVel.linear.z = 0.0
						
							self.telloCmdVel.angular.x = 0.0
							self.telloCmdVel.angular.y = 0.0
	#						self.telloCmdVel.angular.z = 0.0

							self.telloCmdVel_pub.publish(self.telloCmdVel)
						else:
							self.telloCmdVel.linear.x = 0.0
							self.telloCmdVel.linear.y = 0.0
							self.telloCmdVel.linear.z = 0.0
						
							self.telloCmdVel.angular.x = 0.0
							self.telloCmdVel.angular.y = 0.0
							self.telloCmdVel.angular.z = 0.0
						
							self.telloCmdVel_pub.publish(self.telloCmdVel)
							
				# is AprilTag3 detected? : False
				else:
					pass
			else:
				pass
		# AprilTag3 Status Received: False
		else:
			pass
			
	def constrain(self, input, low, high):
		if input < low:
			input = low
		elif input > high:
			input = high
		else:
			input = input
			
		return input
		
	# Find index of missionCount
	def cbFindList(self):
		try:
			self.index = self.apriltagID.index(self.missionID)
		except ValueError:
			pass
			
	# rospy shutdown callback
	def cbShutdown(self):
		rospy.logerr("AprilTag3 Tracking Node [OFFLINE]...")
		
if __name__ == '__main__':

	# Initialize
	rospy.init_node('apriltag_tracking', anonymous=False)
	at = AprilTagTracking()
		
	r = rospy.Rate(10)
	
	# Camera preview
	while not rospy.is_shutdown():
		at.cbAprilTagTracking()
		r.sleep()
