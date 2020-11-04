import numpy as np

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
dataPoints = np.load('dataPoints.npy')
nPoints = len(dataPoints)