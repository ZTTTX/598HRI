import numpy as np
from scipy.linalg import expm

PI = 3.1415926535

def get_S_matrix(s):
	return np.array([[0,-s[2],s[1],s[3]],[s[2],0,-s[0],s[4]],[-s[1],s[0],0,s[5]],[0,0,0,0]])


def Get_MS():
	M = np.array([[0,-1,0,390],[0,0,-1,401],[1,0,0,215.5],[0,0,0,1]])
	w = np.empty([6,3])
	q = np.empty([6,3])
	v = np.empty([6,3])
	S = np.empty([6,6])
	w[0] = np.array([0,0,1])
	q[0] = np.array([-150,150,0])
	w[1] = np.array([0,1,0])
	q[1] = np.array([-150,0,152+10])
	w[2] = np.array([0,1,0])
	q[2] = np.array([-150+244,0,152+10])
	w[3] = np.array([0,1,0])
	q[3] = np.array([-150+244+213,0,152+10])
	w[4] = np.array([1,0,0])
	q[4] = np.array([0,150+120-93+83,152+10])
	w[5] = np.array([0,1,0])
	q[5] = np.array([-150+244+213+83,0,152+10])
	v = -np.cross(w,q)
	S = np.concatenate((w,v),axis=1).T
	return M, S


def ur_fk(theta1, theta2, theta3, theta4, theta5, theta6):
	return_value = [None, None, None, None, None, None]
	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6
	return return_value

def forward_k(theta1, theta2, theta3, theta4, theta5, theta6):
	M, S = Get_MS()
	T = expm(get_S_matrix(S[:,0])*(theta1))@expm(get_S_matrix(S[:,1])*(theta2))@\
		expm(get_S_matrix(S[:,2])*(theta3))@expm(get_S_matrix(S[:,3])*(theta4))@\
		expm(get_S_matrix(S[:,4])*(theta5))@expm(get_S_matrix(S[:,5])*(theta6))@M
	return T

def ur_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	xWgrip = xWgrip*1000 + 150
	yWgrip = yWgrip*1000 - 150
	zWgrip = zWgrip*1000 - 10
	L9 = 53.5
	yaw_WgripRadian = np.deg2rad(yaw_WgripDegree)
	coord_cen = np.array([
		xWgrip - L9 * np.cos(yaw_WgripRadian),
		yWgrip - L9 * np.sin(yaw_WgripRadian),
		zWgrip
	])
	L2 = 120
	theta_tmp1 = np.arctan2(coord_cen[1],coord_cen[0])
	edge1 = np.sqrt(coord_cen[0] ** 2 + coord_cen[1] ** 2)
	edge2 = np.sqrt(edge1**2 - (L2-93+83)**2)
	theta_tmp2 = np.arctan2((L2-93+83), edge2)
	theta1 = theta_tmp1 - theta_tmp2
	theta6 = np.pi/2 + theta1 - yaw_WgripRadian
	L7 = 83
	L8 = 82
	L10 = 59
	edge3 = edge2 - L7
	coord_3end = np.array([
		edge3 * np.cos(theta1),
		edge3 * np.sin(theta1),
		coord_cen[2] + L10 + L8
	])
	L1 = 152
	L3 = 244
	L5 = 213
	r = np.sqrt(coord_3end[0]**2 + coord_3end[1]**2 + (coord_3end[2] - L1)**2)
	theta3 = np.pi - np.arccos((L3 ** 2 + L5 ** 2 - r ** 2) / (2*L3*L5))
	theta_tmp3 = np.arccos((L3 ** 2 + r ** 2 - L5 ** 2) / (2*L3*r))
	height_tmp = coord_3end[2] - L1
	theta_tmp4 = np.arcsin(height_tmp/r)
	theta2 = -(theta_tmp3 + theta_tmp4)
	theta_tmp5 = theta3 - theta_tmp3
	theta4 = -(theta_tmp5 - theta_tmp4)
	theta5 = -np.pi/2
	return ur_fk(theta1, theta2, theta3, theta4, theta5, theta6)
