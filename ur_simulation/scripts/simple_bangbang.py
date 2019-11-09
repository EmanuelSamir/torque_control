#!/usr/bin/env python
import numpy as np
from sensor_msgs.msg import JointState
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *




class Robot:
	def  __init__(self):
		# Measured data initiation
		self.q = []		# Joint position
		self.qd = []	# Joint velocities 
		self.X = []		# Spatial position: x-y-z
		self.torque = []# Estimated actual torque
		self.joint_names = []


		# Controller data
		self.q0 = [-np.pi/4., -0.4, 0, 0, 0, 0]
		self.torque_desired = []
		self.q_desired = []

		# Reading (subscribe)
		rospy.Subscriber("joint_states", JointState, self.cb_joint_state)

		# Command
		#self.arm_command = rospy.Publisher("arm_controller/command", JointTrajectory, queue_size = 10)
		self.client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
		self.client.wait_for_server()

		self.fjt = FollowJointTrajectoryGoal()


	def cb_joint_state(self, data):
		self.joint_names = data.name
		self.q = data.position
		self.qd = data.velocity

	def position_control_update(self, q_desired):
		self.fjt.trajectory = JointTrajectory()
		self.fjt.trajectory.joint_names = self.joint_names
		self.fjt.trajectory.points = [JointTrajectoryPoint(positions = q_desired, velocities = 6*[0.,],
												time_from_start = rospy.Duration(0.1))]
		self.client.send_goal(self.fjt)
		self.client.wait_for_server()
		# self.q_desired = q_desired
		# jt = JointTrajectory()
		# jt.joint_names = self.joint_names
		# jt.points = []
		# jtp = JointTrajectoryPoint()
		# jtp.positions = self.q_desired
		# jt.points.append(jtp)
		# self.arm_command.publish(jt)

	def bring_initial_position(self):
		#for _ in range(1000):
		self.position_control_update(self.q0)

	def destroy(self):
		self.client.cancel_goal()



def main():
	rospy.init_node("robot_control", anonymous = True)

	robot = Robot()
	r = rospy.Rate(10)

	rospy.loginfo("Bring Initial Position")
	time.sleep(0.5)
	while not rospy.is_shutdown():
		robot.bring_initial_position()
		r.sleep() 
	rospy.loginfo("Robot is in Initial Position")

if __name__ == '__main__':
	print("Starting test")
	try:
		main()
	except rospy.ROSInterruptException:
		robot.destroy()