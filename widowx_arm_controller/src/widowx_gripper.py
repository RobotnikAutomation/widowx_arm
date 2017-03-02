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

import roslib; roslib.load_manifest('phantomx_reactor_arm_controller')
import rospy
import actionlib
from control_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import JointState



class PhantomXReactor:
	"""
		Class to communicate with the dinamyxel controllers of the arm
	"""	
	def __init__(self):
		
		try:
			self.name = rospy.get_param('~name', default = 'phantomx_reactor')
			self.j1 = rospy.get_param('~j1', default = 'j1')
			self.j2 = rospy.get_param('~j2', default = 'j2')
			self.j3 = rospy.get_param('~j3', default = 'j3')
			self.j4 = rospy.get_param('~j4', default = 'j4')
			self.j5 = rospy.get_param('~j5', default = 'j5')
			self.j6 = rospy.get_param('~j6', default = 'gripper')
			self.controller_1 = rospy.get_param('~controller_1', default = '/arm_1_joint/command')
			self.controller_2 = rospy.get_param('~controller_2', default = '/arm_2_joint/command')
			self.controller_3 = rospy.get_param('~controller_3', default = '/arm_2_1_joint/command')
			self.controller_4 = rospy.get_param('~controller_4', default = '/arm_3_joint/command')
			self.controller_5 = rospy.get_param('~controller_5', default = '/arm_3_1_joint/command')
			self.controller_6 = rospy.get_param('~controller_6', default = '/arm_4_joint/command')
			self.controller_7 = rospy.get_param('~controller_7', default = '/arm_5_jointr/command') # optional
			self.controller_8 = rospy.get_param('~controller_8', default = '/arm_gripper_joint/command')
			
			# j2 has to controllers syncronized (2,3)
			# j3 has to controllers syncronized (4,5)
			self.controller_1_publisher = rospy.Publisher(self.controller_1, Float64, queue_size = 10)
			self.controller_2_publisher = rospy.Publisher(self.controller_2, Float64, queue_size = 10)
			self.controller_3_publisher = rospy.Publisher(self.controller_3, Float64, queue_size = 10)
			self.controller_4_publisher = rospy.Publisher(self.controller_4, Float64, queue_size = 10)
			self.controller_5_publisher = rospy.Publisher(self.controller_5, Float64, queue_size = 10)
			self.controller_6_publisher = rospy.Publisher(self.controller_6, Float64, queue_size = 10)
			self.controller_7_publisher = rospy.Publisher(self.controller_7, Float64, queue_size = 10)
			self.controller_8_publisher = rospy.Publisher(self.controller_8, Float64, queue_size = 10)
			
			self.joint_command = rospy.Subscriber('~joint_command', JointState, self.jointCommandCb)
			
			self.desired_freq = rospy.get_param('~desired_freq', default = 10.0)
			
		except rospy.ROSException, e:
			rospy.logerr('%s: error getting params %s'%(rospy.get_name(), e))
			exit()

		
	
	def jointCommandCb(self, msg):
		"""
			Receives joint command and send it to the controller
		"""
		if len(msg.name) != len(msg.position):
			rospy.logerr('%s:jointCommandCb: length of joint names and position has to be equal'%(rospy.get_name()))
			return 
		for i in range(len(msg.name)):
			#print 'Joint %s to %lf'%(msg.name[i], msg.position[i])
			self.setJointPosition(msg.name[i], msg.position[i])
		

	def controlLoop(self):
		"""
			Runs the control loop
		"""
		t_sleep = 1.0/self.desired_freq
		
		while not rospy.is_shutdown():
			rospy.sleep(t_sleep)
				
	
	def setJointPosition(self, joint, position):
		"""
			Sends the command to the controller to set the desired joint position
		"""	
		msg = Float64()
		msg.data = position
		
		if joint == self.j1:
			self.controller_1_publisher.publish(msg)
		elif joint == self.j2:
			self.controller_2_publisher.publish(msg)
			msg.data = -position
			self.controller_3_publisher.publish(msg)
		elif joint == self.j3:
			self.controller_4_publisher.publish(msg)
			msg.data = -position
			self.controller_5_publisher.publish(msg)
		elif joint == self.j4:
			self.controller_6_publisher.publish(msg)
		elif joint == self.j5:
			self.controller_7_publisher.publish(msg)
		elif joint == self.j6:
			self.controller_8_publisher.publish(msg)
		
			
	def start(self):
		"""
			Starts the action server and runs spin
		"""	
		try:
			self.controlLoop()
		except rospy.ROSInterruptException:
			rospy.loginfo('%s: Bye!'%rospy.get_name())


def main():

	rospy.init_node('robotnik_phantomx_reactor')
		
	phantomx_reactor_node = PhantomXReactor()
	

	phantomx_reactor_node.start()
	

	
if __name__=='__main__':
	main()
	exit()
	
