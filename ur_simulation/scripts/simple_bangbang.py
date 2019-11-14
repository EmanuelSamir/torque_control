#!/usr/bin/env python
import numpy as np
from sensor_msgs.msg import JointState
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from copy import copy
import rbdl




class Robot:
	def  __init__(self):
		# Measured data initiation
		self.deltaT = 0.01

		self.q = []		# Joint position
		self.qd = np.array([])	# Joint velocities
		self.qdd = np.array([])	# Joint acceleration
		self.X = []		# Spatial position: x-y-z
		self.joint_names = []


		# Controller data
		self.q0 = np.array([-np.pi/4., 0.0, 0.4, 0, 0, 0])
		self.torque = np.zeros(self.q0.shape[0]) # Estimated actual torque
		self.torque_desired = []
		self.q_desired = []


		# Command
		self.client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
		self.client.wait_for_server()

		self.fjt = FollowJointTrajectoryGoal()

		# Inverse Dynamics
		self.model = rbdl.loadModel("../urdf/ur5_joint_limited_robot.urdf")

		# Reading (subscribe)
		rospy.Subscriber("joint_states", JointState, self.cb_joint_state)

	def cb_joint_state(self, data):
		self.joint_names = data.name
		self.q = np.array(data.position)
		qd = copy(self.qd)
		self.qd = np.array(data.velocity)

		if qd.shape == self.qd.shape:
			self.qdd = (qd - self.qd)/self.deltaT

		self.calculate_torque()



	def position_control_update(self, q_desired):
		self.fjt.trajectory = JointTrajectory()
		self.fjt.trajectory.joint_names = self.joint_names
		self.fjt.trajectory.points = [JointTrajectoryPoint(positions = q_desired, velocities = 6*[0.,],
												time_from_start = rospy.Duration(0.1))]
		self.client.send_goal(self.fjt)
		self.client.wait_for_server()

	def calculate_torque(self):
		rbdl.InverseDynamics(self.model, self.q, self.qd, self.qdd, self.torque)
		print(self.torque)

	def bring_initial_position(self):
		self.position_control_update(self.q0)

	def destroy(self):
		self.client.cancel_goal()

	def simple_bangbang(self, torque_desired):
		return q_desired

class Controller:
	def __init__(self):
		pass


def main():
	rospy.init_node("robot_control", anonymous = True)

	robot = Robot()
	r = rospy.Rate(100)

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
		pass#robot.destroy()