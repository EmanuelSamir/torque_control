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

# name: [elbow_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_1_joint, wrist_2_joint,
# wrist_3_joint]

class Robot:
	def  __init__(self, deltaT):
		# Measured data initiation
		self.deltaT = deltaT

		self.q = []		# Joint position
		self.qd = np.array([])	# Joint velocities
		self.qdd = np.array([])	# Joint acceleration
		self.X = []		# Spatial position: x-y-z
		self.joint_names = []


		# Controller data
		self.q0 = np.array([0., 0. , 0., -np.pi/2, 0, 0])
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
												time_from_start = rospy.Duration(self.deltaT))]
		self.client.send_goal(self.fjt)
		self.client.wait_for_server()

	def calculate_torque(self):
		rbdl.InverseDynamics(self.model, self.q, self.qd, self.qdd, self.torque)
		#print(self.torque)

	def bring_initial_position(self):
		self.position_control_update(self.q0)

	def destroy(self):
		self.client.cancel_goal()

	def simple_bangbang(self, torque_desired):
		e = self.torque - torque_desired
		self.controller.update(e)
		q_desired = self.controller.output()

		# To take off!!!!!
		q_desired = q_desired * [0,0,1,0,0,0] + self.q0 * [1,1,0,1,1,1]
		return q_desired

class Controller:
	def __init__(self, q, deltaT, q_size):
		self.deltaT = deltaT
		self.kb = 0.1
		self.Ki = np.diag(np.array([10.,1.,1.,1.,1.,1.])) #np.ones([q_size, q_size])
		self.u = q#np.zeros(q_size)
		self.ud = np.zeros(q_size)

	def update(self, e):
		self.ud =  self.kb * self.Ki.dot(np.sign(e))

	def output(self):
		self.u = self.u + self.deltaT*self.ud
		return self.u


#def main():


if __name__ == '__main__':
	print("Starting test")

	rospy.init_node("robot_control", anonymous = True)
	f = 100.
	robot = Robot(1/f)
	q_desired_lst = []
	torque_lst = []
	q_lst = []

	try:
		r = rospy.Rate(f)

		rospy.loginfo("Bring Initial Position")
		for _ in range(100):
			robot.bring_initial_position()
		rospy.loginfo("Robot is in Initial Position")
		time.sleep(0.5)

		t0 = time.time()

		while not rospy.is_shutdown():
			torque_desired =  np.array([0 ,0, 1.,0.,0.,0.]) +  np.sin( 1. * (time.time() - t0)) *np.array([0.,0.,0.,0.,0.,0.])
			q_desired = robot.simple_bangbang(torque_desired)
			robot.position_control_update(q_desired)
			q_desired_lst.append(q_desired[2])
			q_lst.append(robot.q[2])
			torque_lst.append(robot.torque[2])
			r.sleep() 
	except rospy.ROSInterruptException:
		
		print("holi")
		robot.destroy()
		f, ax = plt.subplots(2)
		ax[0].plot(np.arange(len(q_desired_lst)), q_desired_lst, color = 'red')
		ax[0].plot(np.arange(len(q_lst)), q_lst, color = 'blue')
		ax[1].plot(np.arange(len(torque_lst)), torque_lst, color = 'red')
		plt.show()
		pass#