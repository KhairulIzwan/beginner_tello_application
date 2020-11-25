#!/usr/bin/env python

################################################################################
## {Description}: Recognizing Apriltag (Detecting Single AprilTag Only!)
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

"""
Image published (CompressedImage) from tello originally size of 960x720 pixels
We will try to resize it using imutils.resize (with aspect ratio) to width = 320
and then republish it as Image
"""

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
from std_msgs.msg import Int64
from std_msgs.msg import Bool
from tello_driver.msg import TelloStatus
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import CameraInfo

from beginner_tello_application.msg import objCenter as objCoord
from beginner_tello_application.msg import apriltagData

import rospy

class AprilTagCenter:
	def __init__(self):
		# Initialization
		self.objectCoord = objCoord()
		
		self.apriltagStatus_received = False
		self.missionID_received = False
		
		rospy.logwarn("AprilTag3 Center Node [ONLINE]...")
		
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
					
		# Subscribe to Int64 msg
		self.missionID_topic = "/missionID"
		self.missionID_sub = rospy.Subscriber(
					self.missionID_topic, 
					Int64, 
					self.cbmissionID
					)
					
		# Publish to objCenter msg
		self.objCoord_topic = "/apriltagCoord"
		self.objCoord_pub = rospy.Publisher(
					self.objCoord_topic, 
					objCoord, 
					queue_size=10
					)
					
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
		
	def cbAprilTagCenter(self):
		# AprilTag3 Status Received: True
		if self.apriltagStatus_received:
			if self.missionID_received:
				# Find index of missionCount
				self.cbFindList()
			
				# AprilTag3 Status: True
				if self.apriltagStatus:
			
					# AprilTag3 List: Empty
					# Object Coordinate will be set at Center of Image
					if not self.apriltagID:
						self.objectCoord.centerX = self.imgWidth // 2
						self.objectCoord.centerY = self.imgHeight // 2
					
					# AprilTag3 List: Not Empty	
					else:
						try:
							self.objectCoord.centerX = int(self.apriltagCenter_x[self.index])
							self.objectCoord.centerY = int(self.apriltagCenter_y[self.index])
						except AttributeError as e:
							pass
					
				# Is AprilTag3 Detected: False
				# Object Coordinate will be set at Center of Image
				else:
					self.objectCoord.centerX = self.imgWidth // 2
					self.objectCoord.centerY = self.imgHeight // 2
				
				self.objCoord_pub.publish(self.objectCoord)
			
			else:
				pass
			
		# Is AprilTag3 Received: False
		# Ignored
		else:
			pass
		
	# Find index of missionCount
	def cbFindList(self):
		try:
			self.index = self.apriltagID.index(self.missionID)
		except ValueError:
			pass
				
	# rospy shutdown callback
	def cbShutdown(self):
		rospy.logerr("AprilTag3 Center Node [OFFLINE]...")

if __name__ == '__main__':

	# Initialize
	rospy.init_node('apriltag_center', anonymous=False)
	c = AprilTagCenter()
	
	r = rospy.Rate(10)
	
	# Camera preview
	while not rospy.is_shutdown():
		c.cbAprilTagCenter()
		r.sleep()
