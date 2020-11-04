# Implementation of Genetic Mutation Function

import numpy as np


def mutate(chromosome, mutationProbability, registerRestraints):
    """
    Function to Perform Genetic Mutation Function
    
    Parameters
    ----------
    chromosome          : Chromosome to be Mutated
    mutationProbability : Probability of Mutation
    registerRestraints  : Restraints to the Chromosome

    Returns
    -------
    mutatedChromosome   : Mutated Chromosome
    """
    chromosome = chromosome[0]
    nGenes = len(chromosome)
    mutationProbability /= nGenes
    mutatedChromosome = chromosome.copy()

    for i in range(3):
        for j in range(i, nGenes-4+i, 4):
            randomness = np.random.rand()
            if mutationProbability > randomness:
                mutatedChromosome[j] = np.random.randint(low=0, high=registerRestraints[i], size=(1, 1))

    return mutatedChromosome[np.newaxis, :]
