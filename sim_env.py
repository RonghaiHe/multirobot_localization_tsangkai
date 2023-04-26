import numpy as np

from math import cos, sin, atan2, sqrt

np.random.seed(0)



##### Simulation Parameter setup 仿真环境的参数设置


num_of_trial = 100 # 100 boundedness_sim 使用
total_T = 2000  # 2000 topology_sim, boundnedss_sim 使用


##### Topology Parameter

observ_prob = 0.75 # 观测概率
comm_prob = 0.2 # 通讯概率

##### Simulation Environment Setup

# number of robot
N = 5

# number of landmark
M = 1

dt = 0.5

max_v = 0.09 # 最大速度
max_omega = 0.05 # 最大角速度

var_u_v = (0.05**2)*(max_v**2)
var_u_theta = 0.0001

var_v = 2.0 * 4*max_v*max_v / 12 # overestimate 因为不清楚

var_dis = 0.05**2
var_phi = (2.0/180) ** 2

d_max = 25 # 距离阈值，距离原点的


##### Constants

J = np.matrix([[0, -1],[1, 0]])

z_mtx_10 = np.matrix([
	[0,0,0,0,0,0,0,0,0,0], 
	[0,0,0,0,0,0,0,0,0,0], 
	[0,0,0,0,0,0,0,0,0,0], 
	[0,0,0,0,0,0,0,0,0,0], 
	[0,0,0,0,0,0,0,0,0,0], 
	[0,0,0,0,0,0,0,0,0,0],
	[0,0,0,0,0,0,0,0,0,0],
	[0,0,0,0,0,0,0,0,0,0],
	[0,0,0,0,0,0,0,0,0,0],
	[0,0,0,0,0,0,0,0,0,0]], dtype=float)

i_mtx_10 = np.matrix([ # 用于协方差
	[1,0,0,0,0,0,0,0,0,0], 
	[0,1,0,0,0,0,0,0,0,0], 
	[0,0,1,0,0,0,0,0,0,0], 
	[0,0,0,1,0,0,0,0,0,0], 
	[0,0,0,0,1,0,0,0,0,0], 
	[0,0,0,0,0,1,0,0,0,0],
	[0,0,0,0,0,0,1,0,0,0],
	[0,0,0,0,0,0,0,1,0,0],
	[0,0,0,0,0,0,0,0,1,0],
	[0,0,0,0,0,0,0,0,0,1]], dtype=float)

i_mtx_2 = np.matrix([
	[1, 0],
	[0, 1]], dtype=float)

initial_position = np.matrix([1, 1, 1, 2, 2, 1, -1, -1, 1, 3], dtype=float).T
initial_cov = i_mtx_10 * 0.01 # 初始协方差 10*10

landmark_position = [0.01, 0.02] # 1个landmark的位置

origin = [0.0, 0.0] # 原点

def inRange(a, b): # 判断与原点的距离是否超过距离阈值
	if sqrt( (a[0]-b[0])**2 + (a[1]-b[1])**2 ) > d_max:
		return False
	else:
		return True

class Landmark:

	def __init__(self, index, position):
		self.index = index
		self.position = position

def rot_mtx(theta): # 旋转矩阵/C矩阵
	return np.matrix([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])

def relative_measurement(pos_1, theta_1, pos_2): # 1->2 bearing-range

	delta_x = pos_2[0] - pos_1[0]
	delta_y = pos_2[1] - pos_1[1]
	dis = sqrt(delta_y*delta_y + delta_x*delta_x) + np.random.normal(0, sqrt(var_dis))
	phi = atan2(delta_y, delta_x) + np.random.normal(0, sqrt(var_phi)) - theta_1

	return [dis, phi]



