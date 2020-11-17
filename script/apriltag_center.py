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
from std_msgs.msg import String, Float32, Bool
from sensor_msgs.msg import Image, CameraInfo, CompressedImage

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

from tello_driver.msg import TelloStatus

from nav_msgs.msg import Odometry

from sensor_msgs.msg import Imu

from common_tello_application.msg import objCenter as objCoord

import rospy

from common_tello_application.msg import apriltagN as apriltagList
from common_tello_application.msg import apriltagC as apriltagCenter

class AprilTagCenter:
	def __init__(self):
		self.objectCoord = objCoord()
		
		self.isApriltag_received = False
		
		rospy.logwarn("AprilTag Center Node [ONLINE]...")
		
		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)
		
		# Publish to Bool msg
		self.isApriltag_topic = "/isApriltag"
		self.isApriltag_sub = rospy.Subscriber(
					self.isApriltag_topic, 
					Bool, 
					self.cbIsApriltag
					)
					
		# Subscribe to apriltagCenter msg
		self.apriltagCenterX_topic = "/isApriltag/Center/X"
		self.apriltagCenterX_sub = rospy.Subscriber(
					self.apriltagCenterX_topic, 
					apriltagCenter, 
					self.cbIsApriltagCenterX
					)
					
		# Subscribe to apriltagCenter msg
		self.apriltagCenterY_topic = "/isApriltag/Center/Y"
		self.apriltagCenterY_sub = rospy.Subscriber(
					self.apriltagCenterY_topic, 
					apriltagCenter, 
					self.cbIsApriltagCenterY
					)
					
		# Publish to objCenter msg
		self.objCoord_topic = "/isApriltag/objCoord"
		self.objCoord_pub = rospy.Publisher(
					self.objCoord_topic, 
					objCoord, 
					queue_size=10
					)
					
		# Allow up to one second to connection
		rospy.sleep(1)
		
	# Is AprilTag3 Detected?
	def cbIsApriltag(self, msg):
		try:
			self.isApriltag = msg.data
			
		except AttributeError as e:
			print(e)
			
		if self.isApriltag is not None:
			self.isApriltag_received = True
		else:
			self.isApriltag_received = False
			
	# What AprilTag3 Detected?
	def cbIsApriltagN(self, msg):
		self.isApriltagN = msg.apriltagN
		
	# Center X AprilTag3 Detected?
	def cbIsApriltagCenterX(self, msg):
		self.isApriltagCenterX = msg.apriltagC
		
	# Center Y AprilTag3 Detected?
	def cbIsApriltagCenterY(self, msg):
		self.isApriltagCenterY = msg.apriltagC
		
	# Convert image to OpenCV format
	def cbCameraInfo(self, msg):
		self.imgWidth = msg.width
		self.imgHeight = msg.height
		
	def cbAprilTagCenter(self):
		# Is AprilTag3 Received Status: True
		if self.isApriltag_received:
			# Is AprilTag3 Detected: True
			if self.isApriltag:
				# AprilTag3 List: Empty
				# Object Coordinate will be set at Center of Image
				if not self.isApriltagN:
					self.objectCoord.centerX = self.imgWidth // 2
					self.objectCoord.centerY = self.imgHeight // 2
				# AprilTag3 List: Not Empty	
				else:
					self.objectCoord.centerX = int(self.isApriltagCenterX[0])
					self.objectCoord.centerY = int(self.isApriltagCenterY[0])
			# Is AprilTag3 Detected: False
			# Object Coordinate will be set at Center of Image
			else:
				self.objectCoord.centerX = self.imgWidth // 2
				self.objectCoord.centerY = self.imgHeight // 2
				
			self.objCoord_pub.publish(self.objectCoord)
		# Is AprilTag3 Received: False
		# Ignored
		else:
			pass
			
	# rospy shutdown callback
	def cbShutdown(self):
		rospy.logerr("AprilTag Center Node [OFFLINE]...")

if __name__ == '__main__':

	# Initialize
	rospy.init_node('apriltag_center', anonymous=False)
	c = AprilTagCenter()
	
	r = rospy.Rate(10)
	
	# Camera preview
	while not rospy.is_shutdown():
		c.cbAprilTagCenter()
		r.sleep()
