# Main Implementation to Perform Inverse Kinematics Using Linear Genetic Programming

import numpy as np

from InitializePopulation import initializePopulation
from EvaluateChromosome import evaluateChromosome

def main(inputFile="dataPoints.npy"):
    """
    Function to Perform Inverse Kinematics Using Linear Genetic Programming
    
    Parameters
    ----------
    inputFile  : Name of the Input File (Default: dataPoints.npy)

    Returns
    -------
    None
    """  
    
    # Initialize GA Parameters
    crossoverProbability = 0.6
    mutationProbability = 7   
    mutationDecrease = 0.9999 
    minMutationProbability = 1

    tournamentSelectionParameter = 0.7
    ntournamentParticipants = 4       

    eliteCopies = 2                   

    minError = 0.01                   

    # Chromosome Desciption and Initialization Parameters
    constants = np.array([1, -1, 2])        
    nVariables = 7                          
    startLength = 4 * 7                     
    populationSize = 80                     
    penaltyLength = 120                     

    # Restrain Chromosome from the following: (1) Having More than 6 Operations (2) Storing More than nVariables (3) Accessing More than registerLength as operands
    registerLength = nVariables + len(constants)
    registerRestraints = np.array([8, nVariables, registerLength, registerLength])

    # Load Datapoints
    dataPoints = np.load(inputFile)
    nPoints = len(dataPoints)

    # Initialization
    population = initializePopulation(startLength, populationSize, registerRestraints)

    fitness = np.zeros([populationSize, 1])


    #run LGP
    error = 1
    maximumFitness = 1
    oldMaximumFitness = 1

    while error > minError:
        # Evaluate population and find fittest
        maximumFitness = 0
        bestIndividualIndex = 0
        for i in range(populationSize):
            chromosome = population[i].get_chromosome()

            #evaluate individual
            sum = 0
            for j in range(nPoints):
                x = dataPoints[j, 0]
                y = dataPoints[j, 1]
                z = dataPoints[j, 2]
                est1, est2, est3 = evaluateChromosome(x, y, z, chromosome, nVariables, constants)

if __name__ == "__main__":
    main()