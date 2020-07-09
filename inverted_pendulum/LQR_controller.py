from control.matlab import *
import numpy as np
from scipy import linalg, integrate
import matplotlib.pyplot as plt

def get_system_param():
	m = 1.
	M = 5.
	L = 2.
	g = -10.
	d = 1.
	b = 1. # 1:pendulum up position; -1:pendulum down position
	return m,M,L,g,d,b


def state_space():
	m,M,L,g,d,b = get_system_param()

	A = np.array([[0.,         1.,               0., 0.],\
				  [0.,       -d/M,          b*m*g/M, 0.],\
				  [0.,         0.,               0., 1.],\
				  [0., -b*d/(M*L), -b*(m+M)*g/(M*L), 0.]])

	B = np.array([[0.], [1./M], [0.], [b/(M*L)]])

	C = np.transpose([1, 0, 0, 0])
	
	D = []

	return A,B,C,D

def dynamic_system_analysis(A,B,C,D):
	eig_value, eig_vector = linalg.eig(A)
	print(eig_value.real) # check if the system is sable (any of them is > 0)
	print (np.linalg.matrix_rank(ctrb(A,B))) # check if the system is controllable
	print (np.linalg.matrix_rank(obsv(A,C))) # check if the system is observable

def LQR_controller():
	Q = np.eye(4)
	R = .0001
	print Q

def main():
	A,B,C,D = state_space()
	dynamic_system_analysis(A,B,C,D)
	LQR_controller()

def pendulum_dynamics(x,u):
	m,M,L,g,d,b = get_system_param()
	dx = np.array((4,1))
	'''
	x[0] = distance
	x[1] = velocity
	x[2] = theta (angle)
	x[3] = omega (angular rate)
	'''
	sin_theta = np.sin(x[2])
	cos_theta = np.sin(x[2])

	dx[0][0] = x[1]
	dx[1][0] = (1./D)*(-m**2*L**2*g*cos_theta*sin_theta + \
				m*L**2*(m*L**2(m*L*x[3]**2*sin_theta - d*x[1])) + \
				m*L**2*u)
	dx[2][0] = x[3]
	dx[3][0] = (1./D)*((m+M)*m*g*L*sin_theta -\
				m*L*cos_theta*(m*L*x[3]**2*sin_theta - d*x[1]) - \
				m*L*cos_theta*u)

def simulation():
	t0, t1, dt = 0., 20., 0.001  # start, end, interval
	t = np.linspace(t0,t1, 100)
	x0 = [-1., 0., np.pi()+0.1, 0.] # initial condition
	wr = [0, 0, np.pi(), 0] # reference position
	x  = np.zeros((len(t),len(x0))) # solution array
	x[0,:] = x0 # initialization
	
	ans = integrate.ode(pendulum_dynamics).set_integrator("dopri5") # solve ode
	ans.set_initial_value(x0, t0)
	return dx


if __name__ == '__main__':
	main()
