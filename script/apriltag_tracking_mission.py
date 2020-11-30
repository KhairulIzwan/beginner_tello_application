#!/usr/bin/env python

################################################################################
## {Description}: 
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

# import the necessary Python packages
import time

# import the necessary ROS packages
import rospy

from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

from beginner_tello_application.msg import apriltagData
from beginner_tello_application.msg import missionData
from beginner_tello_application.msg import centerData
from beginner_tello_application.msg import trackingData

from beginner_tello_application.pid import PID
from beginner_tello_application.makesimpleprofile import map as mapped

class AprilTagTrackingMission:
	def __init__(self):
		# Initialization
		self.tracking = trackingData()
		self.telloCmdVel = Twist()
		self.telloTakeoff = Empty()
		self.telloLand = Empty()
		
		self.missionData_received = False
		
		self.missionSet = False
		
		self.trackingStatus = False
		
		self.stateArrived = False
		self.stateStopped = False
		self.stateActivity = False
		
		# speed parameter
		self.MAX_LIN_VEL = 0.50
		self.MAX_LIN_VEL_S2 = 0.20
		self.MAX_LIN_VEL_S = 0.50
		self.MAX_ANG_VEL = 0.50

		# set PID values for pan
		self.panP = 2
		self.panI = 0
		self.panD = 0.001

		# set PID values for tilt
		self.tiltP = 2
		self.tiltI = 0
		self.tiltD = 0.001

		# set PID values for yaw
		self.yawP = 2
		self.yawI = 0
		self.yawD = 0.001

		# set PID values for distance
		self.distanceP = 3
		self.distanceI = 0
		self.distanceD = 0.0001

		# set PID values for height_m
		self.heightP = 2
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
		
		rospy.logwarn("AprilTag3 Tracking Mission Node [ONLINE]...")
		
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
					
		# Subscribe to centerData msg
		self.centerData_topic = "/centerData"
		self.centerData_sub = rospy.Subscriber(
					self.centerData_topic, 
					centerData, 
					self.cbCenterData
					)
					
		# Publish to trackingData msg
		self.trackingData_topic = "/trackingData"
		self.trackingData_pub = rospy.Publisher(
					self.trackingData_topic, 
					trackingData, 
					queue_size=10
					)
					
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
					
		# Publish to Twist msg
		self.telloCmdVel_topic = "/tello/cmd_vel"
		self.telloCmdVel_pub = rospy.Publisher(
					self.telloCmdVel_topic, 
					Twist, 
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
			self.missionDone = msg.missionDone
			
		except AttributeError as e:
			print(e)
			
		if self.missionList is not None:
			self.missionData_received = True
		else:
			self.missionData_received = False
			
	def cbCenterData(self, msg):
		self.centerStatus = msg.centerStatus
		self.centerX = msg.centerX
		self.centerY = msg.centerY
		
	# Convert image to OpenCV format
	def cbCameraInfo(self, msg):	
		self.imgWidth = msg.width
		self.imgHeight = msg.height
		
	# Function to convert number into string 
	# Switcher is dictionary data type here 
	def cbMissionSwitcher(self, argument): 
		switcher = { 
			0: "takeoff", 
			1: "land", 
#			3: "obstacle_3", 
#			4: "obstacle_4", 
#			5: "obstacle_5", 
			6: "obstacle_6", 
#			7: "obstacle_7", 
#			8: "obstacle_8", 
#			9: "obstacle_9", 
			10: "obstacle_10", 
			}
	
		# get() method of dictionary data type returns  
		# value of passed argument if it is present  
		# in dictionary otherwise second argument will 
		# be assigned as default value of passed argument 
		return switcher.get(argument, "nothing")
		
	def cbTelloStop(self):
		self.telloCmdVel.linear.x = 0.00
		self.telloCmdVel.linear.y = 0.00
		self.telloCmdVel.linear.z = 0.00
		
		self.telloCmdVel.angular.x = 0.00
		self.telloCmdVel.angular.y = 0.00
		self.telloCmdVel.angular.z = 0.00
		
	def cbTelloSearch(self):
		self.telloCmdVel.linear.x = 0.00
		self.telloCmdVel.linear.y = 0.00
		self.telloCmdVel.linear.z = 0.00
		
		self.telloCmdVel.angular.x = 0.00
		self.telloCmdVel.angular.y = 0.00
		self.telloCmdVel.angular.z = 0.40
		
	def cbTelloTakeOff(self):	
		self.telloTakeoff_pub.publish(self.telloTakeoff)
		time.sleep(3)
		self.trackingStatus = False
		
	def cbTelloLand(self):
		self.telloLand_pub.publish(self.telloLand)
		time.sleep(3)
		self.trackingStatus = False
		
	# show information callback
	def cbPIDerrCenter(self):
		try:
			self.panErr, self.panOut = self.cbPIDprocess(self.panPID, self.centerX, self.imgWidth // 2)
			self.tiltErr, self.tiltOut = self.cbPIDprocess(self.tiltPID, self.centerY, self.imgHeight // 2)
			self.yawErr, self.yawOut = self.cbPIDprocess(self.yawPID, self.centerX, self.imgWidth // 2)
			# TODO: How to fixed the distance
			self.distanceErr, self.distanceOut = self.cbPIDprocess(self.distancePID, self.apriltagDistance[self.missionIndex], 1.0)
		except IndexError as e:
			pass

	def cbPIDprocess(self, pid, objCoord, centerCoord):
		# calculate the error
		error = centerCoord - objCoord
		
		# update the value
		output = pid.update(error)
		
		return error, output
		
	def constrain(self, input, low, high):
		if input < low:
			input = low
		elif input > high:
			input = high
		else:
			input = input
			
		return input
		
	def cbTelloTarget(self):
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
			self.stateArrived = True
#			rospy.logerr("ARRIVED...")
			
		# self.telloCmdVel.linear.x = 0.0
		# self.telloCmdVel.linear.y = 0.0
		# self.telloCmdVel.linear.z = 0.0
		
		self.telloCmdVel.angular.x = 0.0
		self.telloCmdVel.angular.y = 0.0
		# self.telloCmdVel.angular.z = 0.0
		
	# Find the index of AprilTag3 Detected
	def cbFindList(self, gates):
		try:
			return self.apriltagID.index(gates)
		except ValueError:
			return -1
			
	def cbAprilTagTracking(self):
		self.tracking.trackingStatus = self.trackingStatus
		
		# Mission Data Received: True
		if self.missionData_received:
		
			# Continue mission until missionList completed!
			if not self.missionDone:
			
				# Initialize self.trackingStatus = True
				self.trackingStatus = True
			
				# TODO: Determine mission type
				self.missionSwitcher = self.cbMissionSwitcher(self.missionGate)
#				rospy.loginfo(self.missionSwitcher)
				
				# TODO: Maybe problem here!
				if not self.missionSearch  and self.missionStatus:
					self.cbTelloStop()
					
					self.missionSet = True
					
					if self.missionSwitcher == "takeoff":
						rospy.loginfo("TakeOff...")
						self.cbTelloTakeOff()
						
						time.sleep(2)
						
					elif self.missionSwitcher == "land" and self.missionSet:
						if self.stateArrived == False and self.stateStopped == False:
							rospy.loginfo("Go to Target %s..." % self.missionGate)
							self.cbTelloTarget()
							
							time.sleep(2)

						elif self.stateArrived == True and self.stateStopped == False:
							rospy.loginfo("Stop at Target %s..." % self.missionGate)
							self.cbTelloStop()
							
							self.stateStopped = True
							self.stateActivity = True
							
							time.sleep(2)
							
						elif self.stateActivity == True and self.stateStopped == True:
							rospy.loginfo("Land Target %s..." % self.missionGate)
							self.cbTelloLand()
							
#							time.sleep(2)
							
							self.stateArrived = False
							self.stateStopped = False
							
							self.missionSet = False
							
							time.sleep(2)
							
					elif self.missionSwitcher == "obstacle_6" and self.missionSet:
						if self.stateArrived == False and self.stateStopped == False:
							rospy.loginfo("Go to Target %s..." % self.missionGate)
							self.cbTelloTarget()
							
							time.sleep(2)
							
						elif self.stateArrived == True and self.stateStopped == False:
							rospy.loginfo("Stop at Target %s..." % self.missionGate)
							self.cbTelloStop()
							
							self.stateStopped = True
							self.stateActivity = True
							
							time.sleep(2)
							
						elif self.stateActivity == True and self.stateStopped == True:
							rospy.loginfo("Complete Target %s..." % self.missionGate)
							
#							time.sleep(2)
							
							self.trackingStatus = False
							self.stateArrived = False
							self.stateStopped = False
							
							self.missionSet = False
							
							time.sleep(2)
							
				elif self.missionSearch and not self.missionStatus:
					
					# TODO:
					
					# Find the gates indexs in apriltagData msg
					self.gates = self.cbFindList(self.missionGate)
					
					if not self.apriltagStatus or (self.gates == -1):
#						rospy.logerr("Searching for Gate %s..." % self.missionGate)
						self.cbTelloSearch()
					else:
#						rospy.logwarn("Found Gate %s..." % self.missionGate)
						self.cbTelloStop()
						time.sleep(2)
#						self.missionSet = False
						
				self.trackingData_pub.publish(self.tracking)
				self.telloCmdVel_pub.publish(self.telloCmdVel)
				
			# missionList completed!
			else:
				pass
				
		# Mission Data Received: False
		else:
			pass
			
	# rospy shutdown callback
	def cbShutdown(self):
		rospy.logerr("AprilTag3 Tracking Mission Node [OFFLINE]...")
		
if __name__ == '__main__':

	# Initialize
	rospy.init_node('apriltag_tracking', anonymous=False)
	atm = AprilTagTrackingMission()
		
	r = rospy.Rate(10)
	
	# Camera preview
	while not rospy.is_shutdown():
		atm.cbAprilTagTracking()
		r.sleep()
