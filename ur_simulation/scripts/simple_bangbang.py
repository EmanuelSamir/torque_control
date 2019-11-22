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
import matplotlib.pyplot as plt



class Robot:
	def  __init__(self, deltaT):
		# Measured data initiation
		self.deltaT = deltaT

		self.q = np.array([])		# Joint position
		self.qd = np.array([])	# Joint velocities
		self.qdd = np.array([])	# Joint acceleration
		self.X = []		# Spatial position: x-y-z
		self.joint_names = []


		self.velocity_lst = []


		# Controller data
		self.q0 = np.array([0., -0.5, 0., 0., 0., 0.]) # Up Up Up Down Up Up
		self.torque = np.zeros(self.q0.shape[0]) # Estimated actual torque
		self.torque_desired = []
		self.q_desired = np.zeros(self.q0.shape[0])
		self.controller = Controller(self.q0, self.deltaT, self.q0.shape[0])


		# Command
		self.client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
		self.client.wait_for_server()

		self.fjt = FollowJointTrajectoryGoal()

		# Inverse Dynamics
		self.model = rbdl.loadModel("../urdf/ur5_joint_limited_robot.urdf")

		# Reading (subscribe)
		rospy.Subscriber("joint_states", JointState, self.cb_joint_state)

	def cb_joint_state(self, data):
		self.joint_names = sort_data(data.name)

		q = copy(self.q)
		self.q = np.array(sort_data(data.position))

		self.velocity_lst.append(sort_data(data.velocity)[1])

		qd = copy(self.qd)

		if q.shape == self.q.shape:
			self.qd = (self.q - q)/self.deltaT

		#self.qd = np.array(sort_data(data.velocity))

		if qd.shape == self.qd.shape:
			self.qdd = (self.qd - qd)/self.deltaT

		self.calculate_torque()

	def position_control_update(self, q_desired):
		self.fjt.trajectory = JointTrajectory()
		self.fjt.trajectory.joint_names = unsort_data(self.joint_names)
		self.fjt.trajectory.points = [JointTrajectoryPoint(positions = unsort_data(q_desired), velocities = 6*[0.2,],
												time_from_start = rospy.Duration(self.deltaT))]
		self.client.send_goal(self.fjt)
		self.client.wait_for_server()

	def calculate_torque(self):
		rbdl.InverseDynamics(self.model, self.q, self.qd, self.qdd, self.torque)

	def bring_initial_position(self):
		self.position_control_update(self.q0)

	def destroy(self):
		self.client.cancel_goal()

	def simple_bangbang(self, torque_desired):
		e = self.torque - torque_desired
		self.controller.update(e)
		q_desired = self.controller.output()
		# To take off!!!!!
		q_desired = q_desired * [0,1,0,0,0,0] + self.q0 * [1,0,1,1,1,1]
		return q_desired

class Controller:
	def __init__(self, q, deltaT, q_size):
		self.deltaT = deltaT
		self.kb = 0.1
		self.Ki = np.diag(np.array([5.,4.,1.,1.,1.,1.])) #np.ones([q_size, q_size])
		self.u = q #np.zeros(q_size)
		self.ud = np.zeros(q_size)

	def update(self, e):
		self.ud =   self.kb * self.Ki.dot(np.sign(e))

	def output(self):
		self.u = self.u + self.deltaT*self.ud
		return self.u

def sort_data(data):
	x = list(copy(data))
	x[0] = data[2]
	x[2] = data[0]
	return tuple(x)

def unsort_data(data):
	x = list(copy(data))
	x[2] = data[0]
	x[0] = data[2]
	return tuple(x)

#def main():


if __name__ == '__main__':
	print("Starting test")

	rospy.init_node("robot_control", anonymous = True)
	f = 200.
	robot = Robot(1/f)
	q_desired_lst = []
	torque_lst = []
	velocity_lst = []
	q_lst = []

	try:
		r = rospy.Rate(f)
		time.sleep(0.5)

		rospy.loginfo("Bring Initial Position")
		for _ in range(100):
			robot.bring_initial_position()
		rospy.loginfo("Robot is in Initial Position")
		time.sleep(1.)

		t0 = time.time()

		while not rospy.is_shutdown():
			torque_desired =  np.array([0,-20.,0.,0.,0.,0.]) +  np.sin( 1. * (time.time() - t0)) *np.array([0.,0.,0.,0.,0.,0.])
			q_desired = robot.simple_bangbang(torque_desired)
			robot.position_control_update(q_desired)
			q_desired_lst.append(q_desired[1])
			q_lst.append(robot.q[1])
			torque_lst.append(robot.torque[1])
			velocity_lst.append(robot.qd[1])
			r.sleep() 
	except rospy.ROSInterruptException:
		print("holi")
		robot.destroy()
		f, ax = plt.subplots(4)
		ax[0].plot(np.arange(len(q_desired_lst)), q_desired_lst, color = 'red')
		ax[0].plot(np.arange(len(q_lst)), q_lst, color = 'blue')
		ax[1].plot(np.arange(len(torque_lst)), torque_lst, color = 'red')
		ax[2].plot(np.arange(len(velocity_lst)), velocity_lst, color = 'red')
		ax[3].plot(np.arange(len(robot.velocity_lst)), robot.velocity_lst, color = 'red')
		plt.show()
		pass#