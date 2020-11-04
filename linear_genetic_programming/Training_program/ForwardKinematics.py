# Implementation of Forward Kinematics Functions

import math
import numpy as np


def forwardKinematics(theta1, theta2, theta3):
    """
    Function to Perform Forward Kineamtics

    Parameters
    ----------
    theta1 : Angle 1 input to the Hubert Robot
    theta2 : Angle 2 input to the Hubert Robot
    theta3 : Angle 3 input to the Hubert Robot

    Returns
    -------
    x      : x-position of the Hubert Robot
    y      : y-position of the Hubert Robot
    z      : z-position of the Hubert Robot
    """

    L2 = 0.315
    L3 = 0.045
    L4 = 0.108
    L5 = 0.005
    L6 = 0.034
    L7 = 0.015
    L8 = 0.088
    L9 = 0.204

    cosine1 = math.cos(theta1)
    cosine2 = math.cos(theta2)
    cosine3 = math.cos(theta3)

    sine1 = math.sin(theta1)
    sine2 = math.sin(theta2)
    sine3 = math.sin(theta3)

    mat11 = cosine1 * (cosine2 * cosine3 - sine2 * sine3)
    mat12 = -cosine1 * (cosine3 * sine2 + cosine2 * sine3)
    mat13 = sine1
    mat14 = (L4 - L5) * sine1 + cosine1 * (L6 + cosine2 * L7 + L8 * sine2 + cosine3 * L9 * sine2 + cosine2 * L9 * sine3)

    mat21 = sine1 * (cosine2 * cosine3 - sine2 * sine3)
    mat22 = -sine1 * (cosine3 * sine2 + cosine2 * sine3)
    mat23 = -cosine1
    mat24 = cosine1 * (-L4 + L5) + sine1 * (L6 + cosine2 * L7 + L8 * sine2 + cosine3 * L9 * sine2 + cosine2 * L9 * sine3)

    mat31 = cosine3 * sine2 + cosine2 * sine3
    mat32 = cosine2 * cosine3 - sine2 * sine3
    mat33 = 0
    mat34 = L2 + L3 - cosine2 * (L8 + cosine3 * L9) + L7 * sine2 + L9 * sine2 * sine3

    mat41 = 0
    mat42 = 0
    mat43 = 0
    mat44 = 1

    transformation_matrix= np.array([
        [mat11, mat12, mat13, mat14],
        [mat21, mat22, mat23, mat24],
        [mat31, mat32, mat33, mat34],
        [mat41, mat42, mat43, mat44],
    ])
    vector = np.array([[0], [0], [0], [1]])

    position = transformation_matrix.dot(vector)
    x = position[0, 0]
    y = position[1, 0]
    z = position[2, 0]

    return x, y, z