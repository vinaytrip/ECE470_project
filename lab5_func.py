#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from lab5_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
	M = np.eye(4)
	S = np.zeros((6,6))





	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	T = np.eye(4)

	M, S = Get_MS()








	# ==============================================================#

	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):

    # theta1 to theta6
	L1 = 152
	L2 = 120
	L3 = 244
	L4 = 93
	L5 = 213
	L6 = 83
	L7 = 83
	L8 = 82
	L9 = 53.5
	L10 = 59


	yawWgrip = np.radians(yaw_WgripDegree)

	x_cen = xWgrip - L9 * np.cos(yawWgrip) + 150
	y_cen = yWgrip - L9 * np.sin(yawWgrip) - 150
	z_cen = zWgrip - 10

	hyp_cen = np.sqrt(x_cen**2 + y_cen**2)
	gamma = np.arctan2(y_cen, x_cen)
	small_gamma = np.arcsin((L6 + 27) / hyp_cen)
	theta1 = gamma - small_gamma

	x_3end = x_cen - np.cos(theta1) * L7 + np.sin(theta1) * (L6 + 27)
	y_3end = y_cen - np.sin(theta1) * L7 - np.cos(theta1) * (L6 + 27)
	z_3end = z_cen + L8 + L10

	C = np.sqrt((z_3end - L1)**2 + (x_3end**2 + y_3end**2))
	alpha = np.arctan2(z_3end - L1, np.sqrt(x_3end**2 + y_3end**2))
	beta = np.arccos((L3**2 + C**2 - L5**2) / (2 * L3 * C))
	theta2 = -alpha - beta
	# print(C)
	theta3 = np.pi - np.arccos((L3**2 + L5**2 - C**2) / (2 * L3 * L5))

	theta4 = -(np.pi / 2 - (np.pi / 2 - (theta3 - beta)) - alpha)


	# preset
	theta5 = -np.pi / 2

	theta6 = np.pi/2 - (yawWgrip - theta1)

	return_value = [None, None, None, None, None, None]
	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	# ==============================================================#
	return return_value
   
