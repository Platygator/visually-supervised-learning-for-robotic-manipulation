# Implementation to Test Chromosome

import os
import numpy as np

from EvaluateChromosome import evaluateChromosome

def testChromosome(x, y, z, inputFile="bestChromosome.npy"):
    """
    Function to test Chromosome
    
    Parameters
    ----------
    x           : x-position of the Hubert Robot
    y           : y-position of the Hubert Robot
    z           : z-position of the Hubert Robot
    inputFile   : Name of the Input File (Default: bestChromosome.npy)

    Returns
    -------
    theta1 : Angle 1 input to the Hubert Robot
    theta2 : Angle 2 input to the Hubert Robot
    theta3 : Angle 3 input to the Hubert Robot
    """

    chromosome = np.load(inputFile)
    constants = np.array([1, -1, 2])
    nVariables = 7
    theta1, theta2, theta3 = evaluateChromosome(x, y, z, chromosome, nVariables, constants)

    return theta1, theta2, theta3