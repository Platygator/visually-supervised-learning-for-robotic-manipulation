# Implementation of Data Point Generator Function

import math
import numpy as np

from ForwardKinematics import forwardKinematics


def dataPointGenerator(outputFile="dataPoints.npy"):
    """
    Function to Generate Data Points. The Data Points Would Be Stored as a *.npy file.
    
    Parameters
    ----------
    outputFile  : Name of the Output File (Default: dataPoints.npy)

    Returns
    -------
    None
    """  
    
    theta1 = np.arange(start=0, stop=180, step=5) * (math.pi/180)
    theta2 = np.arange(start=0, stop=180, step=5) * (math.pi/180)
    theta3 = np.arange(start=0, stop=90, step=5) * (math.pi/180)

    dataPoints = np.zeros((len(theta1)*len(theta2)*len(theta3), 6))
    
    for i in range(len(theta1)):
        for j in range(len(theta2)):
            for k in range(len(theta3)):

                index = i*(len(theta2) * len(theta3)) + j*(len(theta3)) + k

                x, y, z = forwardKinematics(theta1[i], theta2[j], theta3[k])
                dataPoints[index, 0] = x
                dataPoints[index, 1] = y
                dataPoints[index, 2] = z
                dataPoints[index, 3] = theta1[i] + np.random.normal(loc=0.0, scale=0.01, size=(1,1))
                dataPoints[index, 4] = theta2[j] + np.random.normal(loc=0.0, scale=0.01, size=(1,1))
                dataPoints[index, 5] = theta3[k] + np.random.normal(loc=0.0, scale=0.01, size=(1,1))
                
    np.save(outputFile, dataPoints)


dataPointGenerator()