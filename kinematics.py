import ach
import sys
import time
from ctypes import *
import baxterStructure as bs

import math
import numpy as np

#Open Hubo-Ach feed-forward and feed-back (reference and state) channels
data_out = ach.Channel('ref_channel')
data_in = ach.Channel('state_channel')

state = bs.STATE()
ref = bs.STATE()

data_in.flush()

# def simSleep(sec, s, state):
	# tick = state.time;
	# dt = 0;
	# while(dt <= sec):
		# s.get(state, wait=False, last=True)
		# dt = state.time - tick;
	# return

def assign_thetas(parameter,left_right):
	#print(parameter)
	thetas = [0,0,0,0,0,0,0]
	for i in range(0,7):
		theta = parameter[i,1]
		thetas[i] = theta
	#print(thetas)	
	if(left_right == 0):
		ref.arm[bs.LEFT].joint[bs.SY].ref = -1 * thetas[bs.SY]
		ref.arm[bs.LEFT].joint[bs.SP].ref = -1 * thetas[bs.SP]
		ref.arm[bs.LEFT].joint[bs.WY].ref = -1 * thetas[bs.WY]
		ref.arm[bs.LEFT].joint[bs.WP].ref = -1 * thetas[bs.WP]
		ref.arm[bs.LEFT].joint[bs.WY2].ref = -1 * thetas[bs.WY2]
		ref.arm[bs.LEFT].joint[bs.SR].ref = -1 * thetas[bs.SR]
		ref.arm[bs.LEFT].joint[bs.EP].ref = -1 * thetas[bs.EP]
	else:
		ref.arm[bs.RIGHT].joint[bs.SY].ref = -1 * thetas[bs.SY]
		ref.arm[bs.RIGHT].joint[bs.SP].ref = -1 * thetas[bs.SP]
		ref.arm[bs.RIGHT].joint[bs.WY].ref = -1 * thetas[bs.WY]
		ref.arm[bs.RIGHT].joint[bs.WP].ref = -1 * thetas[bs.WP]
		ref.arm[bs.RIGHT].joint[bs.WY2].ref = -1 * thetas[bs.WY2]
		ref.arm[bs.RIGHT].joint[bs.SR].ref = -1 * thetas[bs.SR]
		ref.arm[bs.RIGHT].joint[bs.EP].ref = -1 * thetas[bs.EP]
	
def transformCalculate(parameter):
	d = parameter[0]
	theta = parameter[1]
	r = parameter[2]
	alpha = parameter[3]
	
	trans_ind = np.array([[np.cos(theta), -1*np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), r*np.cos(theta)],[np.sin(theta), np.cos(theta)*np.cos(alpha), -1*np.cos(theta)*np.sin(alpha), r*np.sin(theta)],[0,np.sin(alpha),np.cos(alpha), d],[0,0,0,1]])
	return trans_ind

def forward_kinematics(parameters):
	joints = 7		#7 dof
	dimension = 3	#x,y,z
	
	Transform = np.eye(dimension+1)
	for i in range(0,joints):
		Transform = Transform.dot(transformCalculate(parameters[i,:]))
	e_homogenous = Transform.dot(np.array([[0.],[0.],[0.],[1.]]))
	e = e_homogenous[0:3,0]
	
	return e
	
def inverse_kinematics(e, parameters):		
	a_lambda = 100
	dimension = 3
	
	parameters[:,1] = np.array([0,0,0,0,0,0,0])	
	#print(parameters)
	
	initial_position = np.empty_like(e)
	initial_position[:] = e
	intended_position = np.empty_like(e)
	intended_position[:] = e
	final_position = forward_kinematics(parameters)
	iterval = 0
	while(np.sqrt((intended_position-final_position).conj().transpose().dot((intended_position-final_position))) > 1):
		Jacobian = np.zeros((3,max(np.shape(parameters[:,0]))))
		joints = 7
		parameters_new = np.empty_like(
		parameters)
		parameters_new[:] = parameters
		
		for i in range (0, joints):
			parameters_new[i,1] = parameters[i,1] - 0.01
			Jacobian[:,i] = (forward_kinematics(parameters) - forward_kinematics(parameters_new))/0.01
			parameters_new[:] = parameters
		
		J = np.empty_like(Jacobian)
		J[:] = Jacobian
		a = J.dot(J.conj().transpose()) + a_lambda * np.eye(dimension)
		b = intended_position - forward_kinematics(parameters)
		a_size = np.shape(a)

		if (a_size[0] == a_size[1]):
			a_b = np.linalg.solve(a,b)
		else:
			a_b = np.linalg.lstsq(a,b)
		parameters[:,1] = parameters[:,1] + J.conj().transpose().dot(a_b)

		initial_position = final_position
		final_position = forward_kinematics(parameters)
		iterval = iterval + 1
		#dis = np.sqrt((intended_position-final_position).conj().transpose().dot((intended_position-final_position)))
		#print(dis)
	return parameters

np.set_printoptions(suppress=True)
initial_params = np.array([[0.069,0,0.2703,-1.571],
			   [0,0,1.82,1.571],
			   [0.069,0,0.6,-1.571],
			   [0,0,0,1.571],
			   [0.01,0,0.3743,-1.571],
			   [0,0,0,1.571],
			   [0,0,0.2295,0]])
#assign_thetas(initial_params)
init_e = forward_kinematics(initial_params)
#print(init_e)

left_goal_one = np.array([1.97,-0.16,-0.12])
params = inverse_kinematics(left_goal_one, initial_params)
e_check = forward_kinematics(params)
print("end_effector_goal")
print(e_check)
#assign_thetas(params,0)
#assign_thetas(params,1)
#time.sleep(10)

while True:
	[statuss,framesizes] = data_in.get(state,wait=False,last=True)
	#print"here"
	#print state.arm[bs.LEFT].joint[bs.WY2].pos
	#print state.arm[bs.RIGHT].joint[bs.WY2].pos
	params_l = np.array([[0.069,state.arm[bs.LEFT].joint[bs.SY].pos,0.2703,-1.571],
			   [0,state.arm[bs.LEFT].joint[bs.SP].pos,1.82,1.571],
			   [0.069,state.arm[bs.LEFT].joint[bs.WY].pos,0.6,-1.571],
			   [0,state.arm[bs.LEFT].joint[bs.WP].pos,0,1.571],
			   [0.01,state.arm[bs.LEFT].joint[bs.WY2].pos,0.3743,-1.571],
			   [0,0,state.arm[bs.LEFT].joint[bs.SR].pos,1.571],
			   [0,state.arm[bs.LEFT].joint[bs.EP].pos,0.2295,0]])
	params_r = np.array([[0.069,state.arm[bs.RIGHT].joint[bs.SY].pos,0.2703,-1.571],
			   [0,state.arm[bs.RIGHT].joint[bs.SP].pos,1.82,1.571],
			   [0.069,state.arm[bs.RIGHT].joint[bs.WY].pos,0.6,-1.571],
			   [0,state.arm[bs.RIGHT].joint[bs.WP].pos,0,1.571],
			   [0.01,state.arm[bs.RIGHT].joint[bs.WY2].pos,0.3743,-1.571],
			   [0,0,state.arm[bs.RIGHT].joint[bs.SR].pos,1.571],
			   [0,state.arm[bs.RIGHT].joint[bs.EP].pos,0.2295,0]])
	e_l = forward_kinematics(params_l)
	e_r = forward_kinematics(params_r)
	print("end-effector left")
	print(e_l)
	print("end-effector right")
	print(e_r)
	assign_thetas(params,0)
	assign_thetas(params,1)
	data_out.put(ref)
	time.sleep(10)
	
# left_goal_two = np.array([0,3.2,-1.2])
# right_goal_two = np.array([0,3.2,-1.2])
# next_params_l = inverse_kinematics(left_goal_two, initial_params)
# next_params_r = inverse_kinematics(right_goal_two, initial_params)
# e_check = forward_kinematics(next_params_l)
# print(e_check)
# assign_thetas(next_params_l,0)
# assign_thetas(next_params_r,1)
# time.sleep(10)

# left_goal_three = np.array([0,3.6,0.8])
# next_params = inverse_kinematics(left_goal_three, initial_params)
# e_check = forward_kinematics(next_params)
# print(e_check)
# assign_thetas(next_params,0)
# assign_thetas(next_params,1)
# time.sleep(10)

# left_goal_four = np.array([0,4.41,0])
# next_params = inverse_kinematics(left_goal_four, initial_params)
# e_check = forward_kinematics(next_params)
# print(e_check)
# assign_thetas(next_params,0)
# assign_thetas(next_params,1)
# time.sleep(10)
