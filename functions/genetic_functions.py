"""
Created by Jan Schiffeler on 05.11.20
jan.schiffeler[at]gmail.com

Changed by


"""

from math import asin, acos, sin, cos, pi
import numpy as np


def computer_operation(gene, register):
    """
    Function to Computer Operation

    Parameters
    ----------
    gene        : Gene for which the operation must be computed
    register    : Register to Store The Updated Values

    Returns
    -------
    updated_register   : Register with Updated Values
    """

    updated_register = register.copy()
    operand1 = updated_register[int(gene[2])]
    operand2 = updated_register[int(gene[3])]
    save_to = int(gene[1])

    if gene[0] == 0:
        updated_register[save_to] = operand1 + operand2

    elif gene[0] == 1:
        updated_register[save_to] = operand1 - operand2

    elif gene[0] == 2:
        updated_register[save_to] = operand1 * operand2

    elif gene[0] == 3:
        if operand2 != 0:
            updated_register[save_to] = operand1 / operand2
        else:
            updated_register[save_to] = 1e20

    elif gene[0] == 4:
        if np.absolute(operand1) > 1:
            updated_register[save_to] = 1e20
        else:
            updated_register[save_to] = asin(operand1)

    elif gene[0] == 5:
        if np.absolute(operand1) > 1:
            updated_register[save_to] = 1e20
        else:
            updated_register[save_to] = acos(operand1)

    elif gene[0] == 6:
        updated_register[save_to] = sin(operand1)

    elif gene[0] == 7:
        updated_register[save_to] = cos(operand1)

    else:
        raise ValueError("Operation for Requested Parameter Not Defined!")

    return updated_register


def cross(chromosome1, chromosome2):
    """
    Function to perform crossover

    Parameters
    ----------
    chromosome1 : First chromosome to be used for crossover
    chromosome2 : Second chromosome to be used for crossover

    Returns
    -------
    new_chromosome_pair: Pair of crossovered chromosomes
    """
    chromosome1 = chromosome1[0]
    chromosome2 = chromosome2[0]
    too_short_for_crossover = False

    n_operations1 = int(len(chromosome1) / 4)
    if n_operations1 <= 3:
        too_short_for_crossover = True
    else:
        cross_points1 = np.random.randint(low=1, high=(n_operations1), size=(1, 2))
        cross_points1 = np.sort(cross_points1, axis=1)[0]

    n_operations2 = int(len(chromosome2) / 4)
    if n_operations2 <= 3:
        too_short_for_crossover = True
    else:
        cross_points2 = np.random.randint(low=1, high=(n_operations2), size=(1, 2))
        cross_points2 = np.sort(cross_points2, axis=1)[0]

    if not too_short_for_crossover:
        snipped_a1 = chromosome1[0:4 * cross_points1[0]]
        snipped_b1 = chromosome1[4 * cross_points1[0]: 4 * cross_points1[1]]
        snipped_c1 = chromosome1[4 * cross_points1[1]:]

        snipped_a2 = chromosome1[0:4 * cross_points2[0]]
        snipped_b2 = chromosome1[4 * cross_points2[0]: 4 * cross_points2[1]]
        snipped_c2 = chromosome1[4 * cross_points2[1]:]

        new_chromosome1 = np.concatenate((snipped_a1, snipped_b2, snipped_c1))
        new_chromosome2 = np.concatenate((snipped_a2, snipped_b1, snipped_c2))

    else:

        new_chromosome1 = chromosome1
        new_chromosome2 = chromosome2

    return new_chromosome1[np.newaxis, :], new_chromosome2[np.newaxis, :]


def evaluate_chromosome(x, y, z, chromosome, n_variables, constants):
    """
    Function to Evaluate Chromosome

    Parameters
    ----------
    x           : x-position of the Hubert Robot
    y           : y-position of the Hubert Robot
    z           : z-position of the Hubert Robot
    chromosome  : Chromosomes To Be Evaluated
    n_variables  : Number of Variables
    constants   : Constants in the Chromosome

    Returns
    -------
    estimate1   : Estimated angle (theta1) of the Hubert Robot
    estimate2   : Estimated angle (theta2) of the Hubert Robot
    estimate3   : Estimated angle (theta3) of the Hubert Robot
    """

    variables = np.zeros(n_variables)
    variables[0:3] = np.array([x, y, z])
    constants = np.array(constants)
    register = np.concatenate((variables, constants))

    for i in range(0, len(chromosome), 4):
        gene = chromosome[i:i + 4]
        register = computer_operation(gene, register)

    deg2rad = pi / 180
    estimate1 = min(max(register[0], 45 * deg2rad), 135 * deg2rad)
    estimate2 = min(max(register[1], 0 * deg2rad), 90 * deg2rad)
    estimate3 = min(max(register[2], 0 * deg2rad), 90 * deg2rad)

    return estimate1, estimate2, estimate3


def initialize_population(start_length, population_size, register_restraints):
    population = []
    for individual in range(population_size):
        chromosome = np.zeros([1, start_length])
        for i in range(len(register_restraints)):
            r = np.random.randint(low=1, high=int(register_restraints[i]), size=(1, int(start_length / 4)))
            for n, j in enumerate(range(i, start_length + i - 1, 4)):
                chromosome[0, j] = r[0, n].copy()

        individual = chromosome
        population.append(individual)

    return population


def insert_best_individual(population, best_individual, n_copies_best):
    """
    Function to Insert Best Individual

    Parameters
    ----------
    population      : Population
    best_individual  : Best Individual
    n_copies_best     : Number of Copies of Best Individual

    Returns
    -------
    modifiedPopulation  : Modified Population
    """

    for i in range(n_copies_best):
        population[i] = best_individual

    return population


def mutate(chromosome, mutation_probability, register_restraints):
    """
    Function to Perform Genetic Mutation Function

    Parameters
    ----------
    chromosome          : Chromosome to be Mutated
    mutation_probability : Probability of Mutation
    register_restraints  : Restraints to the Chromosome

    Returns
    -------
    mutated_chromosome   : Mutated Chromosome
    """
    chromosome = chromosome[0]
    n_genes = len(chromosome)
    mutation_probability /= n_genes
    mutated_chromosome = chromosome.copy()

    for i in range(3):
        for j in range(i, n_genes - 4 + i, 4):
            randomness = np.random.rand()
            if mutation_probability > randomness:
                mutated_chromosome[j] = np.random.randint(low=0, high=register_restraints[i], size=(1, 1))

    return mutated_chromosome[np.newaxis, :]


def tournament_select(fitness, p_tournament, n_participants):
    """
    Function to Perform Tournament Selection Function

    Parameters
    ----------
    fitness         : Fitness Function
    p_tournament     : Probability of Tournament
    n_participants   : Number of Participants

    Returns
    -------
    selected    : Selected Individual
    """

    population_size = fitness.shape[0]
    participant_fitness = np.zeros([2, n_participants])

    for i in range(n_participants):
        participant_fitness[0, i] = int(np.random.rand() * population_size)
        participant_fitness[1, i] = fitness[int(participant_fitness[1, i])]

    fitness_ind = np.argsort(participant_fitness[1])[::-1]

    participant_order = participant_fitness[0, fitness_ind]

    randomness = np.random.rand()
    winner = 0

    while randomness > p_tournament:
        winner = winner + 1
        if winner == n_participants - 1:
            break
        randomness = np.random.rand()

    selected = int(participant_order[winner])

    return selected


def forward_kinematics(theta1, theta2, theta3):
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

    cosine1 = cos(theta1)
    cosine2 = cos(theta2)
    cosine3 = cos(theta3)

    sine1 = sin(theta1)
    sine2 = sin(theta2)
    sine3 = sin(theta3)

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

    vector = np.array([[0], [0], [0], [1]])

    position = transformation_matrix.dot(vector)
    x = position[0, 0]
    y = position[1, 0]
    z = position[2, 0]

    return x, y, z
