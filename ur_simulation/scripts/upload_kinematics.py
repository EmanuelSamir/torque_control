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
from numpy.linalg import inv
from numpy import matmul as mx



class Robot:
	def  __init__(self, deltaT):
		# Measured data initiation
		self.deltaT = deltaT
		self.q = np.array([])		# Joint position


		self.joint_spaces = [Joint(self.deltaT),
							 Joint(self.deltaT), 
							 Joint(self.deltaT), 
							 Joint(self.deltaT), 
							 Joint(self.deltaT), 
							 Joint(self.deltaT)]

		self.X = []		# Spatial position: x-y-z
		self.joint_names = []

		self.q0 = np.array([0., 0. , 0., -np.pi/2, np.pi/2, 0])
		self.qd = np.zeros(self.q0.shape[0])	# Joint velocities
		self.qdd = np.zeros(self.q0.shape[0])	# Joint acceleration

		self.J = np.zeros([6,6])
		self.X = np.zeros(3)

		self.velocity_lst = []

		# Controller data
		self.velocity_lst = []
		self.velocity_lst = []

		# Controller data
		self.qdd_tocheck = np.zeros(self.q0.shape[0])
		self.torque = np.zeros(self.q0.shape[0]) # Estimated actual torque
		self.torque_desired = []
		self.q_desired = np.zeros(self.q0.shape[0])
		#self.controller = Controller(self.q0, self.deltaT, self.q0.shape[0])


		# Command
		self.client = actionlib.SimpleActionClient('trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
		self.client.wait_for_server()

		self.fjt = FollowJointTrajectoryGoal()

		# Inverse Dynamics
		self.model = rbdl.loadModel("../urdf/ur5_joint_limited_robot.urdf")


		#self.pub = rospy.Publisher('trajectory_controller/command', JointTrajectory, queue_size = 10)

		# Reading (subscribe)
		rospy.Subscriber("joint_states", JointState, self.cb_joint_state)

	def cb_joint_state(self, data):
		self.joint_names = sort_data(data.name)
		self.q = np.array(sort_data(data.position))

		rbdl.CalcBodyToBaseCoordinates(self.model, self.q, 0, self.X, False)

		rbdl.CalcBodySpatialJacobian(self.model, self.q, 0, self.X, self.J, False)

		rbdl.UpdateKinematics(self.model, self.q, self.qd, self.qdd)

		self.calculate_torque()
		self.calculate_acceleration()

	def position_control_update(self, q_desired):

		self.fjt.trajectory = 	JointTrajectory()
		self.fjt.trajectory.header.stamp =rospy.Time.now()
		#trajectory = JointTrajectory()
		self.fjt.trajectory.joint_names = self.joint_names
		#trajectory.joint_names = self.joint_names
		self.fjt.trajectory.points = [JointTrajectoryPoint(positions = np.concatenate([q_desired, np.array([0.5*np.sin(2*time.time())])], axis = 0), #velocities = 6*[0.2,],
												time_from_start = rospy.Duration(self.deltaT))]
		#trajectory.points = [JointTrajectoryPoint(positions = q_desired, #velocities = 6*[0.2,],
		#									time_from_start = rospy.Duration(self.deltaT))]
		#self.pub.publish(trajectory)
		self.client.send_goal(self.fjt)
		self.client.wait_for_server()

	def calculate_torque(self):
		rbdl.InverseDynamics(self.model, self.q, self.qd, self.qdd, self.torque)

	def calculate_acceleration(self):
		rbdl.ForwardDynamics(self.model, self.q, self.qd, self.torque, self.qdd_tocheck)

	def bring_initial_position(self):
		self.position_control_update(self.q0)

	def destroy(self):
		self.client.cancel_goal()

	def simple_bangbang(self, torque_desired):
		q_desired = 0.4*np.sin(2*time.time())
		q_desired = q_desired * np.array([1,0,0,0,0,0]) + self.q0 * np.array([0,1,1,1,1,1])
		return q_desired

def sort_data(data):
	x = list(copy(data))
	#x[0] = data[2]
	#x[2] = data[0]
	x[0] = data[3]
	x[1] = data[2]
	x[2] = data[0]
	x[3] = data[4]
	x[4] = data[5]
	x[5] = data[6]
	x[6] = data[1]
	x.pop(-1)
	return tuple(x)

def unsort_data(data):
	x = 7*[0]
	#x[2] = data[0]
	#x[0] = data[2]
	x[3] = data[0]
	x[2] = data[1]
	x[0] = data[2]
	x[4] = data[5]
	x[5] = data[4]
	x[6] = data[5]
	x[1] = data[6]	
	return tuple(x)

class SavGol:
	def __init__(self, acc = True):
		if acc:
			#self.coeff = np.array([-2, 3, 6, 7, 6, 3, -2])/21.
			self.coeff = np.array([-78, -13, 42, 87,122, 147,162,167,162,147,122,87,42,-13,-78])/1105
			self.X = np.zeros(15)
		else:
			self.coeff = np.array([-42,-21,-2, 15, 30,43,54,63,70,75,78,79,78,75,70,63,54,43,30,15,-2,-21,-42])/805.
			self.X = np.zeros(23)

	def update(self, x):
		self.X[:-1] = self.X[1:]
		self.X[-1] = x
		y = np.sum(self.coeff * self.X)
		return y


class Joint:
    def __init__(self, deltaT):
        self.q = 0.
        self.qd = 0.
        self.qdd = 0.

        self.deltaT = deltaT
        
        self.sav_gol_qd = SavGol()
        self.sav_gol_qdd = SavGol(False)
        

    def update_data(self, q):
    	qd_ = (q - self.q)/self.deltaT
    	qd = self.sav_gol_qd.update(qd_)

    	qdd_ =(qd - self.qd)/self.deltaT
    	qdd = self.sav_gol_qdd.update(qdd_)

    	self.q = copy(q)
    	self.qd = copy(qd)
    	self.qdd = copy(qdd)


if __name__ == '__main__':
	print("Starting test")

	rospy.init_node("robot_control", anonymous = True)
	f = 100.
	robot = Robot(1/f)
	q_desired_lst = []
	torque_lst = []
	vel_lst = []
	acc_lst = []
	q_lst = []
	acc_check_lst = []
	t = []

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
			torque_desired =  np.array([-2.,0.,0.,0.,0.,0.]) +  np.sin( 1. * (time.time() - t0)) *np.array([0.,0.,0.,0.,0.,0.])
			q_desired = robot.simple_bangbang(torque_desired)
			robot.position_control_update(q_desired)
			q_desired_lst.append(q_desired[0])
			q_lst.append(robot.q[0])
			vel_lst.append(robot.qd[0])
			acc_lst.append(robot.qdd[0])
			acc_check_lst.append(robot.qdd_tocheck[0])
			torque_lst.append(robot.torque[0])
			t.append(time.time())
			r.sleep() 
	except rospy.ROSInterruptException:
		robot.destroy()
		f, ax = plt.subplots(4)
		ax[0].plot(np.arange(len(q_desired_lst)), q_desired_lst, color = 'red')
		ax[0].plot(np.arange(len(q_lst)), q_lst, color = 'blue')
		ax[1].plot(np.arange(len(torque_lst)), torque_lst, color = 'red')
		ax[2].plot(np.arange(len(vel_lst)), vel_lst, color = 'red')
		ax[3].plot(np.arange(len(acc_lst)), acc_lst, color = 'red')
		ax[3].plot(np.arange(len(acc_check_lst)), acc_check_lst, color = 'blue',linestyle='dashed')
		ax[0].set_title("Position")
		ax[1].set_title("Torque")
		ax[2].set_title("Velocidad")
		ax[3].set_title("Aceleracion")
		f_, ax_ = plt.subplots(1)
		tdiff = np.diff(t)
		ax_.plot(np.arange(np.shape(tdiff)[0]), 1/tdiff)

		plt.show()
		pass#
