# Function to Initialize Population

import numpy as np


def initializePopulation(startLength, populationSize, registerRestraints):
    
    population = []
    for individual in range(populationSize):
        chromosome = np.zeros([1, startLength])
        for i in range(len(registerRestraints)):
            r = np.random.randint(low=1, high=int(registerRestraints[i]), size=(1, int(startLength/4)))
            for n, j in enumerate(range(i, startLength+i-1, 4)):
                chromosome[0, j] = r[0, n].copy()

        individual = chromosome
        population.append(individual)

    return population


if __name__ == '__main__':
    p = initializePopulation(4*3, 4, np.array([8, 7, 10, 10]))

    print(p)
