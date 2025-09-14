#!usr/bin/env python

import numpy as np
import scipy as sp
import kin_func_skeleton as kfs 

def ur7e_foward_kinematics_from_angles(joint_anges):
    """
    Calculate the orientation of the ur7e's end-effector tool given
    the joint angles of each joint in radians

    Parameters:
    ------------
    joint_angles ((6x) np.ndarray): 6 joint angles (s0, s1, e0, w1, w2, w3)

    Returns: 
    ------------
    (4x4) np.ndarray: homogenous transformation matrix
    """
    q0 = np.ndarray((3, 8)) # Points on each joint axis in the zero config
    w0 = np.ndarray((3, 7)) # Axis vector of each joint axis in the zero config

    q0[0:3,0] = [0.000, 0.000, 0.162]
    q0[0:3,1] = [-0.000, 0.000, 0.162]
    q0[0:3,2] = [-0.010, 0.307, 0.457]
    q0[0:3,3] = [0.111, 0.698, 0.520]
    q0[0:3,4] = [0.112, 0.691, 0.619]
    q0[0:3,5] = [0.119, 0.790, 0.626]
    q0[0:3,6] = [0.119, 0.790, 0.626]
    q0[0:3,7] = [0.119, 0.790, 0.626]

    w0[0:3,0] = [0.000000, 0.000000, 1.000000]
    w0[0:3,1] = [0.999518, 0.031006, 0.000630]
    w0[0:3,2] = [0.999090, 0.031130, -0.000100]
    w0[0:3,3] = [1.000060, 0.033020, -0.000400]
    w0[0:3,4] = [0.003186, -0.072162, 0.997390]
    w0[0:3,5] = [0.077920, 0.994020, 0.073400]
    w0[0:3,6] = [0.004306, -0.072282, 0.997374]

    R = np.array([[-0.9964, 0.0042, 0.0770],
                    [ 0.0387, -0.0734, 0.9940],
                    [ 0.00496, 0.9985, 0.074]])
    # YOUR CODE HERE (Task 1)


def ur7e_forward_kinematics_from_joint_state(joint_state):
    """
    Computes the orientation of the ur7e's end-effector given the joint
    state.

    Parameters
    ----------
    joint_state (sensor_msgs.JointState): JointState of ur7e robot

    Returns
    -------
    (4x4) np.ndarray: homogenous transformation matrix
    """
    
    angles = np.zeros(7)

    # YOUR CODE HERE (Task 2)
    

    # END YOUR CODE HERE
    trans_mat = ur7e_foward_kinematics_from_angles(angles)
    print(trans_mat)