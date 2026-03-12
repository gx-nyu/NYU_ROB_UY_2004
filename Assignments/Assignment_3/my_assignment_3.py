import math
import numpy as np
import scipy

def forward_kinematics(theta1, theta2, theta3):
    def rotation_x(angle):
        return np.array([
            [1, 0, 0, 0],
            [0, np.cos(angle), -np.sin(angle), 0],
            [0, np.sin(angle), np.cos(angle), 0],
            [0, 0, 0, 1]
        ])

    def rotation_y(angle):
        return np.array([
            [np.cos(angle), 0, np.sin(angle), 0],
            [0, 1, 0, 0],
            [-np.sin(angle), 0, np.cos(angle), 0],
            [0, 0, 0, 1]
        ])

    def rotation_z(angle):
        return np.array([
            [np.cos(angle), -np.sin(angle), 0, 0],
            [np.sin(angle), np.cos(angle), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

    def translation(x, y, z):
        return np.array([
            [1, 0, 0, x],
            [0, 1, 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1]
        ])

    T_0_1 = translation(0.07500, -0.08350, 0) @ rotation_x(1.57080) @ rotation_z(theta1)
    T_1_2 = rotation_y(-1.57080) @ rotation_z(theta2)
    T_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(theta3)
    T_3_ee = translation(0.06231, -0.06216, 0.01800)
    T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee
    return T_0_ee[:3, 3]

def get_error_leg(thetas,ee_des):
    x = forward_kinematics(thetas[0],thetas[1],thetas[2])
    error = np.linalg.norm(ee_des - x)
    return error

def inverse_kinematics_with_optimizer(ee_pos_xyz):
    x_0 = np.array([0.1,0.1,0.1])
    joint_angles = scipy.optimize.minimize(get_error_leg,x_0,args=ee_pos_xyz).x
    return joint_angles

def inverse_kinematics_with_gradient(ee_pos_xyz):
    joint_angles = np.array([0.0, 0.0, 0.0])
    def get_cost(joint_angles, ee_pos_xyz):
        estimation = forward_kinematics(joint_angles[0],joint_angles[1],joint_angles[2])
        error = ee_pos_xyz-estimation
        sum_squared_error = np.sum(error**2)
        mean_absolute_error = np.mean(np.abs(error))
        return sum_squared_error,  mean_absolute_error
   
    def get_gradient(joint_angles, ee_pos_xyz):
        epsilon = 0.0001
        gradient = np.zeros(len(joint_angles))
        for i in range(len(joint_angles)):
            epsilon_vec = np.zeros(len(joint_angles))
            epsilon_vec[i] = epsilon
            gradient[i] = (get_cost(joint_angles + epsilon_vec,ee_pos_xyz)[0] - get_cost(joint_angles - epsilon_vec,ee_pos_xyz)[0])/(2*epsilon)
        # gradient = (get_cost(joint_angles + epsilon,ee_pos_xyz)[0] - get_cost(joint_angles - epsilon,ee_pos_xyz)[0])/(2*epsilon)
        return gradient
    
    alpha = 0.05
    tolerance = 0.001
    for i in range(0,50000):
        joint_angles = joint_angles - alpha*get_gradient(joint_angles,ee_pos_xyz)
        if get_cost(joint_angles,ee_pos_xyz)[1] < tolerance:
            break
    return joint_angles

