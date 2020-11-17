#!/usr/bin/env python

################################################################################
## {Description}: Tello Pose
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

# import the necessary Python packages
import tf
import math

from tello_driver.msg import TelloStatus
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from common_tello_application.msg import Pose

import rospy

class TelloPose:
	def __init__(self):
	
		self.dronePose = Pose()
		rospy.logwarn("Tello Pose Node [ONLINE]...")
		
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
				self.cbTelloOdometry
				)
				
		# Subscribe to PoseWithCovariance msg
		self.telloIMU_topic = "/tello/imu"
		self.telloIMU_sub = rospy.Subscriber(
				self.telloIMU_topic, Imu, 
				self.cbTelloIMU
				)
				
		# Publish to Pose msg
		self.dronePose_topic = "/dronePose"
		self.dronePose_pub = rospy.Publisher(
					self.dronePose_topic, 
					Pose, 
					queue_size=10
					)
					
		# Allow up to one second to connection
		rospy.sleep(1)
		
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
	
		# POSE
		self.poseX = msg.pose.pose.position.x
		self.poseY = msg.pose.pose.position.y
		self.poseZ = msg.pose.pose.position.z
		
		# QUATERNION
		self.orientationX = msg.pose.pose.orientation.x
		self.orientationY = msg.pose.pose.orientation.y
		self.orientationZ = msg.pose.pose.orientation.z
		self.orientationW = msg.pose.pose.orientation.w
		
		self.quaternion = (self.orientationX, 
				self.orientationY,
				self.orientationZ,
				self.orientationW)
				
		self.euler = tf.transformations.euler_from_quaternion(self.quaternion)
		self.rollRad = self.euler[0]	# radians
		self.pitchRad = self.euler[1]	# radians
		self.yawRad = self.euler[2]	# radians
		
		self.rollDeg = math.degrees(self.rollRad)	# degrees
		self.pitchDeg = math.degrees(self.pitchRad)	# degrees
		self.yawDeg = math.degrees(self.yawRad)	# degrees
		
		#
		self.linearX = msg.twist.twist.linear.x
		self.linearY = msg.twist.twist.linear.y
		self.linearZ = msg.twist.twist.linear.z
		
		#
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
		
	def cbPose(self):
		self.dronePose.x = self.poseX
		self.dronePose.y = self.poseY
		self.dronePose.z = self.poseZ
		
		self.dronePose.phi = self.rollRad
		self.dronePose.theta = self.pitchRad
		self.dronePose.psi = self.yawRad
		
		self.dronePose.linear_velocity_x = self.linearX
		self.dronePose.linear_velocity_y = self.linearY
		self.dronePose.linear_velocity_z = self.linearZ
		
		self.dronePose.angular_velocity_x = self.angularX
		self.dronePose.angular_velocity_y = self.angularY
		self.dronePose.angular_velocity_z = self.angularZ
		
		self.dronePose_pub.publish(self.dronePose)
		
	# rospy shutdown callback
	def cbShutdown(self):
		rospy.logerr("Tello Pose Node [OFFLINE]...")

if __name__ == '__main__':
	# Initialize
	rospy.init_node('tello_pose', anonymous=False)
	t = TelloPose()
	
	r = rospy.Rate(10)
	
	# Camera preview
	while not rospy.is_shutdown():
		t.cbPose()
		r.sleep()
