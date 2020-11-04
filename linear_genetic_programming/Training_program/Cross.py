# Implementation to perform crossover

import numpy as np

def cross(chromosome1, chromosome2):
    """
    Function to perform crossover

    Parameters
    ----------
    chromosome1 : First chromosome to be used for crossover
    chromosome1 : Second chromosome to be used for crossover

    Returns
    -------
    newChromosomePair: Pair of crossovered chromosomes
    """
    chromosome1 = chromosome1[0]
    chromosome2 = chromosome2[0]
    tooShortForCrossover = False

    nOperations1 = int(len(chromosome1)/4)
    if nOperations1 <= 3:
        tooShortForCrossover = True
    else:
        crossPoints1 = np.random.randint(low=1, high=(nOperations1), size=(1, 2))
        crossPoints1 = np.sort(crossPoints1, axis=1)[0]

    nOperations2 = int(len(chromosome2)/4)
    if nOperations2 <= 3:
        tooShortForCrossover = True
    else:
        crossPoints2 = np.random.randint(low=1, high=(nOperations2), size=(1, 2))
        crossPoints2 = np.sort(crossPoints2, axis=1)[0]

    if tooShortForCrossover == False:
        snippedA1 = chromosome1[0:4*crossPoints1[0]]
        snippedB1 = chromosome1[4*crossPoints1[0]: 4*crossPoints1[1]]
        snippedC1 = chromosome1[4*crossPoints1[1]:]

        snippedA2 = chromosome1[0:4*crossPoints2[0]]
        snippedB2 = chromosome1[4*crossPoints2[0]: 4*crossPoints2[1]]
        snippedC2 = chromosome1[4*crossPoints2[1]:]

        newChromosome1 = np.concatenate((snippedA1, snippedB2, snippedC1))
        newChromosome2 = np.concatenate((snippedA2, snippedB1, snippedC2))

    else:

        newChromosome1 = chromosome1
        newChromosome2 = chromosome2

    return newChromosome1[np.newaxis, :], newChromosome2[np.newaxis, :]


