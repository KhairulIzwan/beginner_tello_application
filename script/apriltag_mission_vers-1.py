#!/usr/bin/env python

################################################################################
## {Description}: AprilTag3 Mission
## {Description}: What will Tello action if designated AprilTag3 were found
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

# import the necessary Python packages
import time

# import the necessary ROS packages
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int64
from std_msgs.msg import Empty

from tello_driver.msg import TelloStatus
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

from beginner_tello_application.msg import apriltagData

import rospy

class ApriltagMission:
	def __init__(self):
		# Initialization
#		self.missionID = Int64()
		self.takeoff = Empty()
		self.land = Empty()
		self.missionSearch = Bool()
		
		self.apriltagStatus_received = False
		self.missionON = False
		self.missionComplete = False
		
		self.missionCount = 0
		self.missionLoc = -1
		
		self.gateList = [4, 1, 2, 3, 0]
		
		rospy.logwarn("Apriltag3 Mission Node [ONLINE]...")
		
		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)
		
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
					self.cbTelloOdometry)
					
		# Subscribe to PoseWithCovariance msg
		self.telloIMU_topic = "/tello/imu"
		self.telloIMU_sub = rospy.Subscriber(
					self.telloIMU_topic, 
					Imu, 
					self.cbTelloIMU)
					
		# Subscribe to apriltagData msg
		self.apriltagData_topic = "/apriltagData"
		self.apriltagData_sub = rospy.Subscriber(
					self.apriltagData_topic, 
					apriltagData, 
					self.cbAprilTagData
					)
					
#		# Publish to Int64 msg
#		self.missionID_topic = "/missionID"
#		self.missionID_pub = rospy.Publisher(
#					self.missionID_topic, 
#					Int64, 
#					queue_size=10)
					
		# Publish to Bool msg
		self.missionSearch_topic = "/missionSearch"
		self.missionSearch_pub = rospy.Publisher(
					self.missionSearch_topic, 
					Bool, 
					queue_size=10)
					
		# Publish to Empty msg
		self.telloTakeoff_topic = "/tello/takeoff"
		self.telloTakeoff_pub = rospy.Publisher(
					self.telloTakeoff_topic, 
					Empty, 
					queue_size=10)
					
		# Publish to Empty msg
		self.telloLand_topic = "/tello/land"
		self.telloLand_pub = rospy.Publisher(
					self.telloLand_topic, 
					Empty, 
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

	def cbAprilTagTakeOffLand(self):
		# AprilTag3 Status Received: True
		if self.apriltagStatus_received:

			# Find the gates of AprilTag3 need to be processed
			self.gates = self.gateList[self.missionCount]
			
			# Find the gates indexs in apriltagData msg
			self.index = self.cbFindList(self.gates)
			
			# TODO: If exist, set missionON
			if self.index != -1 and self.missionComplete == False:
				self.missionON = True
			elif self.index == -1 and self.missionComplete == True:
				self.missionON = False
			
#			self.missionID.data = self.index
#			self.missionID_pub.publish(self.missionID)

#			rospy.logwarn("Index: %s" % self.index)
##			rospy.logwarn("Mission No: %s" % self.missionLoc)
#			rospy.logwarn("Mission Count: %s" % self.missionCount)
#			rospy.logwarn("Next Gate No: %s" % self.gateList[self.missionCount])
#			rospy.logwarn("On Mission: %s" % self.missionON)
			
			# AprilTag3 List: Empty
			if not self.apriltagID:
				# TODO: What the drone should do if apriltag detected
				# not what as required
				pass
			
			# TODO: Mission Executed
			if self.missionON:
				self.missionSearch.data = False
				
				# Mission: 0 : Gate 0 : Takeoff
				if self.gates == 4:
#					self.telloTakeoff_pub.publish(self.takeoff)
					rospy.loginfo("Takeoff")
					
					self.missionCount += 1
					self.missionComplete = False
					self.missionON = False
					
				# Mission: 1 : Gate 1 : Land
				elif self.gates == 1:
#					self.telloLand_pub.publish(self.land)
					rospy.loginfo("Gate 1 Mission")
					
					self.missionCount += 1
					self.missionComplete = False
					self.missionON = False
					
				# Mission: 2 : Gate 2 : Land
				elif self.gates == 2:
#					self.telloLand_pub.publish(self.land)
					rospy.loginfo("Land")
					
					self.missionCount += 1
					self.missionComplete = False
					self.missionON = False
					
			else:
				# TODO: Ask the drone to rotate searching for gates
#				rospy.logwarn("Searching for Gates: %s" % self.gates)
				self.missionSearch.data = True
				
			self.missionSearch_pub.publish(self.missionSearch)

		# AprilTag3 Status Received: False
		else:
			# TODO: Information related node not yet running
			rospy.logwarn("AprilTag3 Detection Node [OFFLINE]...")
			
	# Find the index of AprilTag3 Detected
	def cbFindList(self, gates):
		try:
			return self.apriltagID.index(gates)
		except ValueError:
			return -1
			
	# rospy shutdown callback
	def cbShutdown(self):
		rospy.logerr("Apriltag3 Mission Node [OFFLINE]...")

if __name__ == '__main__':

	# Initialize
	rospy.init_node('apriltag_mission', anonymous=False)
	m = ApriltagMission()
	
	r = rospy.Rate(10)
	
	# Camera preview
	while not rospy.is_shutdown():
		m.cbAprilTagTakeOffLand()
		r.sleep()
