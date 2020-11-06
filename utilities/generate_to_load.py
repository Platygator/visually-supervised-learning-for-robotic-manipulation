# Implementation of Data Point Generator Function

import math
import numpy as np

from functions.genetic_functions import forward_kinematics


def data_point_generator():
    """
    Function to Generate Data Points. The Data Points Would Be Stored as a *.npy file.
    
    Parameters
    ----------

    Returns
    -------
    data_points array
    """  
    
    theta1 = np.arange(start=0, stop=180, step=5) * (math.pi/180)
    theta2 = np.arange(start=0, stop=180, step=5) * (math.pi/180)
    theta3 = np.arange(start=0, stop=90, step=5) * (math.pi/180)

    data_points = np.zeros((len(theta1)*len(theta2)*len(theta3), 6))
    
    for i in range(len(theta1)):
        for j in range(len(theta2)):
            for k in range(len(theta3)):

                index = i*(len(theta2) * len(theta3)) + j*(len(theta3)) + k

                x, y, z = forward_kinematics(theta1[i], theta2[j], theta3[k])
                data_points[index, 0] = x
                data_points[index, 1] = y
                data_points[index, 2] = z
                data_points[index, 3] = theta1[i] + np.random.normal(loc=0.0, scale=0.01, size=(1, 1))
                data_points[index, 4] = theta2[j] + np.random.normal(loc=0.0, scale=0.01, size=(1, 1))
                data_points[index, 5] = theta3[k] + np.random.normal(loc=0.0, scale=0.01, size=(1, 1))

    return data_points


if __name__ == '__main__':
    points = data_point_generator()
    np.save("resources/dataPoints.npy", points)
