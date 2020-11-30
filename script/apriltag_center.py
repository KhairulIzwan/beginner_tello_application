#!/usr/bin/env python

################################################################################
## {Description}: Recognizing Apriltag (Detecting Single AprilTag Only!)
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {2}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

"""
Image published (CompressedImage) from tello originally size of 960x720 pixels
We will try to resize it using imutils.resize (with aspect ratio) to width = 320
and then republish it as Image
"""

# import the necessary Python packages

# import the necessary ROS packages
import rospy

from sensor_msgs.msg import CameraInfo

from beginner_tello_application.msg import apriltagData
from beginner_tello_application.msg import missionData
from beginner_tello_application.msg import centerData

class AprilTagCenter:
	def __init__(self):
		# Initialization
		self.center = centerData()
		
		self.missionData_received = False
		
		self.centerStatus = False
		
		rospy.logwarn("AprilTag3 Center Node [ONLINE]...")
		
		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)
		
		# Subscribe to CompressedImage msg
		self.telloCameraInfo_topic = "/tello/camera/camera_info"
#		self.telloCameraInfo_topic = "/cv_camera/camera_info"
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
					
		# Subscribe to missionData msg
		self.missionData_topic = "/missionData"
		self.missionData_sub = rospy.Subscriber(
					self.missionData_topic, 
					missionData, 
					self.cbMissionData
					)
					
		# Publish to objCenter msg
		self.centerData_topic = "/centerData"
		self.centerData_pub = rospy.Publisher(
					self.centerData_topic, 
					centerData, 
					queue_size=10
					)
					
		# Allow up to one second to connection
		rospy.sleep(1)
		
	# Is AprilTag3 Data Received?
	def cbAprilTagData(self, msg):
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
		
	def cbMissionData(self, msg):
		try:
			self.missionList = msg.missionList
			self.missionCount = msg.missionCount
			self.missionGate = msg.missionGate
			self.missionIndex = msg.missionIndex
			self.missionSearch = msg.missionSearch
			self.missionStatus = msg.missionStatus
			
		except AttributeError as e:
			print(e)
			
		if self.missionList is not None:
			self.missionData_received = True
		else:
			self.missionData_received = False
			
	# Convert image to OpenCV format
	def cbCameraInfo(self, msg):
		self.imgWidth = msg.width
		self.imgHeight = msg.height
		
	def cbAprilTagCenter(self):
		# Mission Data Received: True
		if self.missionData_received:
		
			# AprilTag3 Status: True
			if self.missionStatus:
				try:
					self.center.centerStatus = True
					self.center.centerX = int(self.apriltagCenter_x[self.missionIndex])
					self.center.centerY = int(self.apriltagCenter_y[self.missionIndex])
				except IndexError:
					pass
			# Is AprilTag3 Detected: False
			# Object Coordinate will be set at Center of Image
			else:
				self.center.centerStatus = False
				self.center.centerX = self.imgWidth // 2
				self.center.centerY = self.imgHeight // 2
				
			self.centerData_pub.publish(self.center)
			
		# Mission Data Received: False
		else:
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
