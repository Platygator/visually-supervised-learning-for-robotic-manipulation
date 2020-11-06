import numpy as np
import math

M_w_p = np.array([[0, 0, -1, 0.035], [0, 1, 0, 0.105], [1, 0, 0, 0.041], [0, 0, 0, 1]])


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
    mat24 = cosine1 * (-L4 + L5) + sine1 * (
            L6 + cosine2 * L7 + L8 * sine2 + cosine3 * L9 * sine2 + cosine2 * L9 * sine3)

    mat31 = cosine3 * sine2 + cosine2 * sine3
    mat32 = cosine2 * cosine3 - sine2 * sine3
    mat33 = 0
    mat34 = L2 + L3 - cosine2 * (L8 + cosine3 * L9) + L7 * sine2 + L9 * sine2 * sine3

    mat41 = 0
    mat42 = 0
    mat43 = 0
    mat44 = 1

    transformation_matrix = np.array([
        [mat11, mat12, mat13, mat14],
        [mat21, mat22, mat23, mat24],
        [mat31, mat32, mat33, mat34],
        [mat41, mat42, mat43, mat44],
    ])
    return transformation_matrix


conversion = math.pi/180
body = - 90 * conversion
shoulder = 90 * conversion
elbow = - 90 * conversion
# ConversionFactor = 180 / 3.1415
# body = int(90 + (body * ConversionFactor))
# shoulder = int(144 - (shoulder * ConversionFactor))
# elbow = int(88 + (elbow * ConversionFactor))

M_r_g = forwardKinematics(body, shoulder, elbow)
M_g_p = np.array([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
M_r_p = M_r_g.dot(M_g_p)

M_p_w = np.linalg.inv(M_w_p)

M_r_w = M_r_p.dot(M_p_w)

print(M_r_w)
np.save('M_r_w', M_r_w)
