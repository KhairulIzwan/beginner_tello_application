#!/usr/bin/env python  

################################################################################
## {Description}: Tello TF Broadcaster
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

# import the necessary Python packages
import rospy
import tf

from common_tello_application.msg import Pose

class TelloPoseBroadcaster:
	def __init__(self):
		self.br = tf.TransformBroadcaster()
		
		rospy.logwarn("Tello Pose Broadcaster Node [ONLINE]...")
		
		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)
		
		# Subscribe to Pose msg
		self.dronePose_topic = "/dronePose"
		self.dronePose_sub = rospy.Subscriber(
					self.dronePose_topic, 
					Pose, 
					self.cbPose
					)
					
		# Allow up to one second to connection
		rospy.sleep(1)
		
	def cbPose(self, msg):
		self.poseX = msg.x
		self.poseY = msg.y
		self.poseZ = msg.z
		
		self.rollRad = msg.phi
		self.pitchRad = msg.theta
		self.yawRad = msg.psi
		
		self.linearX = msg.linear_velocity_x
		self.linearY = msg.linear_velocity_y
		self.linearZ = msg.linear_velocity_z
		
		self.angularX = msg.angular_velocity_x
		self.angularY = msg.angular_velocity_y
		self.angularZ = msg.angular_velocity_z
		
	def cbTransferFunction(self):
		self.br.sendTransform(
			(self.poseX, self.poseY, self.poseZ),
			tf.transformations.quaternion_from_euler(0, 0, self.yawRad),
			rospy.Time.now(),
			"drone",
			"world")
			
	# rospy shutdown callback
	def cbShutdown(self):
		rospy.logerr("Tello Pose Broadcaster Node [OFFLINE]...")

if __name__ == '__main__':
	# Initialize
	rospy.init_node('tello_pose_broadcaster', anonymous=False)
	telloTF = TelloPoseBroadcaster()
	
	r = rospy.Rate(10)
	
	# Camera preview
	while not rospy.is_shutdown():
		telloTF.cbTransferFunction()
		r.sleep()
