#!/usr/bin/env python

################################################################################
## {Description}: Detecting an Apriltag3
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {2}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

# import the necessary Python packages
import cv2
import imutils
import apriltag

# import the necessary ROS packages
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

from beginner_tello_application.msg import apriltagData

import rospy

class AprilTagDetection:
	def __init__(self):
		# Initialization
		# OpenCV -- ROS
		self.bridge = CvBridge()
		
		# AprilTag3 
		self.detector = apriltag.Detector()
		
		# AprilTag3 msg
		self.apriltagData = apriltagData()
		
		# state
		self.image_received = False
		
		# Distance Parameters
		self.knownWidth = 0.135 # meter (m)
		self.perWidth = 120 # pixels
		self.knownDistance = 1 # meter (m)
		self.focalLength = (
			(self.perWidth * self.knownDistance) / self.knownWidth)
			
		# Text and Image parameters configuration
		self.fontFace = cv2.FONT_HERSHEY_PLAIN
		self.fontScale = 0.7
		self.color = (255, 255, 255)
		self.colorPose = (0, 0, 255)
		self.colorIMU = (255, 0, 255)
		self.thickness = 1
		self.lineType = cv2.LINE_AA
		self.bottomLeftOrigin = False # if True (text upside down)
		
		rospy.logwarn("AprilTag3 Detection Node [ONLINE]...")
		
		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)
		
		# Subscribe to CompressedImage msg /cv_camera/image_raw
#		self.telloImage_topic = "/tello/image_raw/compressed"
		self.telloImage_topic = "/cv_camera/image_raw/compressed"
		self.telloImage_sub = rospy.Subscriber(
						self.telloImage_topic, 
						CompressedImage, 
						self.cbImage
						)
						
		# Publish to apriltagData msg
		self.apriltagData_topic = "/apriltagData"
		self.apriltagData_pub = rospy.Publisher(
					self.apriltagData_topic, 
					apriltagData, 
					queue_size=10
					)
					
		# Allow up to one second to connection
		rospy.sleep(1)

	# Convert image to OpenCV format
	def cbImage(self, msg):

		try:
			# direct conversion to cv2
			self.cv_image = self.bridge.compressed_imgmsg_to_cv2(
								msg, 
								"bgr8"
								)
		except CvBridgeError as e:
			print(e)

		if self.cv_image is not None:
			self.image_received = True
		else:
			self.image_received = False
			
	def cbAprilTag(self):
		# Tello Node: ONLINE
		if self.image_received:
			# TODO: Publish related AprilTag3 Data
			
			# An empty array to accomodate AprilTag3 Data (if available)
			self.apriltagData.apriltagID = []
			self.apriltagData.apriltagCenter_x = []
			self.apriltagData.apriltagCenter_y = []
			self.apriltagData.apriltagHomography_00 = []
			self.apriltagData.apriltagHomography_01 = []
			self.apriltagData.apriltagHomography_02 = []
			self.apriltagData.apriltagHomography_10 = []
			self.apriltagData.apriltagHomography_11 = []
			self.apriltagData.apriltagHomography_12 = []
			self.apriltagData.apriltagHomography_20 = []
			self.apriltagData.apriltagHomography_21 = []
			self.apriltagData.apriltagHomography_22 = []
			self.apriltagData.apriltagCorner_x1 = []
			self.apriltagData.apriltagCorner_y1 = []
			self.apriltagData.apriltagCorner_x2 = []
			self.apriltagData.apriltagCorner_y2 = []
			self.apriltagData.apriltagCorner_x3 = []
			self.apriltagData.apriltagCorner_y3 = []
			self.apriltagData.apriltagCorner_x4 = []
			self.apriltagData.apriltagCorner_y4 = []
			self.apriltagData.apriltagDistance = []
		
			# Converting to grayscale
			cv_image_gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)

			# AprilTag3 detection
			result = self.detector.detect(cv_image_gray)

			# AprilTag3 was detected
			if len(result) != 0:
				# TODO: AprilTag3 Data not empty!
				self.apriltagData.apriltagStatus = True
			
				for i in range(len(result)):
					self.apriltagData.apriltagID.append(result[i][1])
					self.apriltagData.apriltagCenter_x.append(result[i][6][0])
					self.apriltagData.apriltagCenter_y.append(result[i][6][1])
					self.apriltagData.apriltagHomography_00.append(result[i][5][0][0])
					self.apriltagData.apriltagHomography_01.append(result[i][5][0][1])
					self.apriltagData.apriltagHomography_02.append(result[i][5][0][2])
					self.apriltagData.apriltagHomography_10.append(result[i][5][1][0])
					self.apriltagData.apriltagHomography_11.append(result[i][5][1][1])
					self.apriltagData.apriltagHomography_12.append(result[i][5][1][2])
					self.apriltagData.apriltagHomography_20.append(result[i][5][2][0])
					self.apriltagData.apriltagHomography_21.append(result[i][5][2][1])
					self.apriltagData.apriltagHomography_22.append(result[i][5][2][2])
					self.apriltagData.apriltagCorner_x1.append(result[i][7][0][0])
					self.apriltagData.apriltagCorner_y1.append(result[i][7][0][1])
					self.apriltagData.apriltagCorner_x2.append(result[i][7][1][0])
					self.apriltagData.apriltagCorner_y2.append(result[i][7][1][1])
					self.apriltagData.apriltagCorner_x3.append(result[i][7][2][0])
					self.apriltagData.apriltagCorner_y3.append(result[i][7][2][1])
					self.apriltagData.apriltagCorner_x4.append(result[i][7][3][0])
					self.apriltagData.apriltagCorner_y4.append(result[i][7][3][1])
					self.apriltagData.apriltagDistance.append(self.distance_to_camera(abs(result[i][7][1][0] - result[i][7][0][0])))
					cv2.putText(
						self.cv_image, 
						"%d" % (result[i][1]), 
						(int(result[i][6][0]), int(result[i][6][1])), 
						self.fontFace, 
						self.fontScale * 5, 
						self.color, 
						self.thickness * 5, 
						self.lineType, 
						self.bottomLeftOrigin)
					
					cv2.line(
						self.cv_image, 
						(int(result[i][7][0][0]), int(result[i][7][0][1])), 
						(int(result[i][7][1][0]), int(result[i][7][1][1])), 
						(0, 0, 255), 
						10)
					
					cv2.line(
						self.cv_image, 
						(int(result[i][7][0][0]), int(result[i][7][0][1])), 
						(int(result[i][7][3][0]), int(result[i][7][3][1])), 
						(0, 255, 0), 
						10)
					
					cv2.line(
						self.cv_image, 
						(int(result[i][7][1][0]), int(result[i][7][1][1])), 
						(int(result[i][7][2][0]), int(result[i][7][2][1])), 
						(255, 0, 0), 
						10)
					
					cv2.line(
						self.cv_image, 
						(int(result[i][7][2][0]), int(result[i][7][2][1])), 
						(int(result[i][7][3][0]), int(result[i][7][3][1])), 
						(255, 0, 0), 
						10)
					
					cv2.circle(
						self.cv_image, 
						(int(result[i][6][0]), int(result[i][6][1])), 
						10, 
						(255, 0, 0), 
						-1)
						
			# No Apriltag3 found
			else:
				# TODO: AprilTag3 Data will be empty!
				self.apriltagData.apriltagStatus = False
			
			self.apriltagData_pub.publish(self.apriltagData)
			
#			cv2.putText(
#				self.cv_image, 
#				"AprilTag3: %s" % (self.apriltagData.apriltagStatus), 
#				(20, 40), 
#				self.fontFace, 
#				self.fontScale * 4, 
#				(0, 0, 255), 
#				self.thickness * 2, 
#				self.lineType, 
#				self.bottomLeftOrigin)
#			
#			cv2.putText(
#				self.cv_image, 
#				"N AprilTag3: %d" % (len(self.apriltagData.apriltagID)), 
#				(20, 80), 
#				self.fontFace, 
#				self.fontScale * 4, 
#				(0, 0, 255), 
#				self.thickness * 2, 
#				self.lineType, 
#				self.bottomLeftOrigin)
#			
#			cv2.putText(
#				self.cv_image, 
#				"AprilTag3 List: %s" % (self.apriltagData.apriltagID), 
#				(20, 120), 
#				self.fontFace, 
#				self.fontScale * 4, 
#				(0, 0, 255), 
#				self.thickness * 2, 
#				self.lineType, 
#				self.bottomLeftOrigin)
		
		# Tello Node: OFFLINE	
		else:
			# TODO: Information from related node not yet running
			rospy.logwarn("Tello Node [OFFLINE]...")
			
	def distance_to_camera(self, perWidth):
		# compute and return the distance from the maker to the camera
		return (self.knownWidth * self.focalLength) / perWidth

	# Show the output frame
	def cbShowImage(self):
		self.cv_image_clone = imutils.resize(
					self.cv_image.copy(), 
					width=320
					)
					
		cv2.imshow("AprilTag3 Detection", self.cv_image_clone)
		cv2.waitKey(1)
		
	# Preview image + info
	def cbDetection(self):
		if self.image_received:
			self.cbAprilTag()
			self.cbShowImage()
		else:
			rospy.logerr("No images recieved")
			
	# rospy shutdown callback
	def cbShutdown(self):
		rospy.logerr("AprilTag3 Detection Node [OFFLINE]...")
		cv2.destroyAllWindows()

if __name__ == '__main__':
	# Initialize
	rospy.init_node('apriltag_detection', anonymous=False)
	a = AprilTagDetection()
	
	r = rospy.Rate(10)
	
	# Camera preview
	while not rospy.is_shutdown():
		a.cbDetection()
		r.sleep()
