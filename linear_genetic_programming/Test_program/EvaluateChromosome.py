# Implementation for Evaluating Chromosome

from math import pi
import numpy as np

from ComputeOperation import computerOperation

def evaluateChromosome(x, y, z, chromosome, nVariables, constants):
    """
    Function to Evaluate Chromosome

    Parameters
    ----------
    x           : x-position of the Hubert Robot
    y           : y-position of the Hubert Robot
    z           : z-position of the Hubert Robot
    chromosome  : Chromosomes To Be Evaluated
    nVariables  : Number of Variables 
    constants   : Constants in the Chromosome

    Returns
    -------
    estimate1   : Estimated angle (theta1) of the Hubert Robot
    estimate2   : Estimated angle (theta2) of the Hubert Robot
    estimate3   : Estimated angle (theta3) of the Hubert Robot
    """

    variables = np.zeros(nVariables)
    variables[0:3] = np.array([x, y, z])
    constants = np.array(constants)
    register = np.concatenate((variables, constants))

    for i in range(0, len(chromosome), 4):
        gene = chromosome[i:i+4]
        register = computerOperation(gene, register)

    deg2rad = pi/180
    estimate1 = min(max(register[0], 45 * deg2rad), 135 * deg2rad)
    estimate2 = min(max(register[1], 0 * deg2rad), 90 * deg2rad)
    estimate3 = min(max(register[2], 0 * deg2rad), 90 * deg2rad)

    return estimate1, estimate2, estimate3