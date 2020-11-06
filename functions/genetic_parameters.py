import numpy as np
from resources import data_points

# Initialize GA Parameters
crossover_probability = 0.6
mutation_probability = 7
mutation_decrease = 0.9999
min_mutation_probability = 1

tournament_selection_parameter = 0.7
ntournament_participants = 4

elite_copies = 2

min_error = 0.01

# Chromosome Desciption and Initialization Parameters
constants = np.array([1, -1, 2])
n_variables = 7
start_length = 4 * 7
population_size = 80
penalty_length = 120

# Restrain Chromosome from the following: (1) Having More than 6 Operations (2) Storing More than n_variables
# (3) Accessing More than register_length as operands
register_length = n_variables + len(constants)
register_restraints = np.array([8, n_variables, register_length, register_length])

# Load Datapoints
n_points = len(data_points)
