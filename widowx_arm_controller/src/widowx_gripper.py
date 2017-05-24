#!/usr/bin/env python
"""
	@file widowx_gripper.py
	
	
	Subscribes:
		- 
		
	Publishes:
		- 
		
	Actions:
		
	
	@author: Robotnik Automation
	
	Software License Agreement (BSD License)
	
	Copyright (c) 2015 Robotnik Automation SLL. All Rights Reserved.

	Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

	1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

	2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer
	   in the documentation and/or other materials provided with the distribution.

	3. The name of the author may not be used to endorse or promote products derived from this software without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY ROBOTNIK AUTOMATION SLL "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
	AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
	OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
	AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
	EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import roslib; roslib.load_manifest('widowx_arm_controller')
import rospy
import actionlib
from control_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import JointState



class WidowxGripper:
	"""
		Class to communicate with the dinamyxel controllers of the arm
	"""	
	def __init__(self):
		
		self.distance = 0.0;
		
		try:
			self.name = rospy.get_param('~name', default = 'widowx')
			self.revolute_command = rospy.get_param('~revolute_command', default = '/gripper_revolute_joint/command')
			self.joint_name = rospy.get_param('~joint_name', default = 'gripper_prismatic_joint_1')
			self.prismatic_command = rospy.get_param('~prismatic_command', default = '/gripper_prismatic_joint/command')
			
			self.gripper_revolute_joint_state_subscriber = rospy.Subscriber('/joint_states', JointState, self.jointStateCb)
			self.gripper_prismatic_joint_subscriber = rospy.Subscriber(self.prismatic_command, Float64, self.gripperPrismaticJointCommandCb)
			self.gripper_prismatic_joint_state_publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)
			self.gripper_revolute_joint_publisher = rospy.Publisher(self.revolute_command, Float64, queue_size=10)
			
			
			self.desired_freq = rospy.get_param('~desired_freq', default = 10.0)
			
		except rospy.ROSException, e:
			rospy.logerr('%s: error getting params %s'%(rospy.get_name(), e))
			exit()

	def jointStateCb(self, msg):
		
		 if 'gripper_revolute_joint' in msg.name:
			index = msg.name.index('gripper_revolute_joint')
			position = msg.position[index]
			velocity = msg.velocity[index]
						
			self.distance = (-0.01154 * position) + 0.03
	
	def gripperPrismaticJointCommandCb(self, msg):
		"""
			Receives joint command and send it to the controller
		"""
		#rospy.loginfo('%s: info getting params'%rospy.get_name())
		
		gripper_goal = Float64()
		
		if(msg.data >= 0 and msg.data <= 0.03):
			self.distance = msg.data;
			rad = (-86.67 * self.distance) + 2.6
			gripper_goal.data = rad
			self.gripper_revolute_joint_publisher.publish(gripper_goal)

		
	def controlLoop(self):
		"""
			Runs the control loop
		"""
		joint_state_gripper = JointState()
		
		joint_state_gripper.header = Header()
		joint_state_gripper.header.stamp = rospy.Time.now()
		joint_state_gripper.name = [self.joint_name]
		joint_state_gripper.position = [self.distance]
		joint_state_gripper.velocity = [0.0]
		joint_state_gripper.effort = [0.0]
		
		self.gripper_prismatic_joint_state_publisher.publish(joint_state_gripper)
		
		t_sleep = 1.0/self.desired_freq
		
		while not rospy.is_shutdown():
			joint_state_gripper.header.stamp = rospy.Time.now()
			joint_state_gripper.position = [self.distance]
			
			self.gripper_prismatic_joint_state_publisher.publish(joint_state_gripper)
			rospy.sleep(t_sleep)
			
	def start(self):
		"""
			Starts the action server and runs spin
		"""	
		try:
			self.controlLoop()
		except rospy.ROSInterruptException:
			rospy.loginfo('%s: Bye!'%rospy.get_name())

def main():

	rospy.init_node('widowx_gripper_node')
		
	widowx_node = WidowxGripper()
	
	widowx_node.start()
	
	
if __name__=='__main__':
	main()
	exit()
	
