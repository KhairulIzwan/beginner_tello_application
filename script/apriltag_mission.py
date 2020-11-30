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

# import the necessary ROS packages
import rospy

from beginner_tello_application.msg import apriltagData
from beginner_tello_application.msg import missionData
from beginner_tello_application.msg import trackingData

class ApriltagMission:
	def __init__(self):
		# Initialization
		self.mission = missionData()
		
		self.apriltagData_received = False
#		self.trackingData_received = False
		
		self.missionStatus = False
		self.missionSearch = True
		self.missionComplete = False
		
		self.missionCount = 0
		
		self.missionList = [0, 6, 1]
		
		rospy.logwarn("Apriltag3 Mission Node [ONLINE]...")
		
		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)
		
		# Subscribe to apriltagData msg
		self.apriltagData_topic = "/apriltagData"
		self.apriltagData_sub = rospy.Subscriber(
					self.apriltagData_topic, 
					apriltagData, 
					self.cbAprilTagData
					)
					
		# Subscribe to trackingData msg
		self.trackingData_topic = "/trackingData"
		self.trackingData_sub = rospy.Subscriber(
					self.trackingData_topic, 
					trackingData, 
					self.cbTrackingData
					)
					
		# Publish to missionData msg
		self.missionData_topic = "/missionData"
		self.missionData_pub = rospy.Publisher(
					self.missionData_topic, 
					missionData, 
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
			self.apriltagData_received = True
		else:
			self.apriltagData_received = False
			
	def cbTrackingData(self, msg):
		self.trackingStatus = msg.trackingStatus

	def cbAprilTagTakeOffLand(self):
		self.mission.missionList = self.missionList
		self.mission.missionSearch = self.missionSearch
		self.mission.missionCount = self.missionCount
		
		# AprilTag3 Status Received: True
		if self.apriltagData_received:
			
			# Find the gates of AprilTag3 need to be processed
			if self.missionCount < len(self.missionList):
				self.mission.missionGate = self.mission.missionList[self.missionCount]
				self.mission.missionDone = False
			else:
				self.mission.missionDone = True
				pass
				
			# Find the gates indexs in apriltagData msg
			self.mission.missionIndex = self.cbFindList(self.mission.missionGate)
		
			# TODO: If exist, set missionON
			if self.mission.missionIndex != -1 and not self.missionComplete:
				self.mission.missionSearch = False
				self.mission.missionStatus = True
			elif self.mission.missionIndex == -1 and self.missionComplete:
				self.mission.missionSearch = True
				self.mission.missionStatus = False
				
			self.missionData_pub.publish(self.mission)
		
			# TODO: Mission Executed
			# TODO: Required trackingStatus
			if self.mission.missionStatus:
#				self.mission.missionSearch = False
				
				if not self.trackingStatus:
					self.missionCount += 1
					self.missionComplete = False
#					self.mission.missionSearch = True
					self.mission.missionStatus = False
				else:
					pass
			else:
				pass
					
		# AprilTag3 Status Received: False
		else:
			pass
			
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
