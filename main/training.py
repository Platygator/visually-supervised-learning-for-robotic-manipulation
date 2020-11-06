"""
Created by Jan Schiffeler on 18.10.20
jan.schiffeler[at]gmail.com

Changed by

MICROSERVICE for:
Training an LGP chromosome which should find the reverse kinematics for a Hubert kind robot.

Instructions:
- First set hardware_learning to False and let it train purely in simulation until sufficient convergence
- Then set hardware_learning to True to start the hardware in the loop training, which uses the simulation based
  training as a starting point

In:
Name | ID    sender number: description, ...
frame | 1001    1: Cube frame
frame | 1001    2: Hand frame

Out:
Name | ID    sender number: description, ...
frame | 1001    3: Angles

"""

# hand eye calibration matrix
from resources import M_r_w

# genetic
from functions.genetic_parameters import *
from functions.genetic_functions import evaluate_chromosome, mutate, cross, tournament_select, \
    insert_best_individual, initialize_population, forward_kinematics
from functions.position_class import Position

# utilities
import numpy as np
from time import sleep
from time import time as time_now
import argparse
import logging
from math import sqrt, pow

logging.basicConfig(level=logging.INFO)


def create_argparser():
    """
    setup all required arguments as in:
        parser.add_argument('-s', '--side', required=False, default=0, help='some help text')
    :return: argparse dictionary
    """
    parser = argparse.ArgumentParser(description='template opendlv for python')
    parser.add_argument('-f', '--frequency', required=False, default=2, help='updating frequency')
    parser.add_argument('-c', '--cid', required=False, default=111, help='conference ID')
    parser.add_argument('-n', '--name', required=False, default="training", help='logging output name')
    parser.add_argument('-l', '--hardware', required=False, default=False, help='use hardware training')
    args = vars(parser.parse_args())
    return args


# handling setup arguments
arg = create_argparser()
logger = logging.getLogger(arg["name"])

own_cid = int(arg['cid'])
print(f"CID: {own_cid}")
freq = int(arg['frequency'])
sleeping_time = 1 / freq
print(f"Frequency: {freq}")
hardware_learning = arg['hardware'] == "True"
print("Hardware learning: ", hardware_learning)

# connectivity
if hardware_learning:
    # UPD multicast
    from libcluon import OD4Session
    from libcluon import opendlv_sim_Frame

    # serial
    from functions.serial_connection import SerialConnection

# initialize storage for callback function
cube_position = Position()
hand_position = Position()


def process_message(msg, senderStamp, timeStamps):
    """
    Handle the incoming messages.
    Change the parameter name according to the received message type!
    :param msg: message
    :param senderStamp: cid of the sender
    :param timeStamps: array of timeStamps [sent, received, sample]
    :return: nothing, writes in message_info dictionary
    """
    logger.debug(f"Received distance; senderStamp= {senderStamp}")
    logger.debug(f"sent: {timeStamps[0]}, received: {timeStamps[1]}, sample time stamps: {timeStamps[2]}")
    logger.debug(msg)
    if senderStamp == 0:
        cube_position.x = msg.x
        cube_position.y = msg.y
        cube_position.z = msg.z
        # distance is the name of a entry in the msg
    if senderStamp == 1:
        hand_position.x = msg.x
        hand_position.y = msg.y
        hand_position.z = msg.z


def evaluate_fitness(ser, body, shoulder, elbow):
    ser.run_serial(body, shoulder, elbow, False)
    ser.wait_for_execution()
    if cube_position.x == 500 or hand_position.x == 500 or (cube_position.x == 0.0 and hand_position.x == 0.0):
        # give bad fitness
        return 0.1

    return 1/(cube_position - hand_position)


def transform_world_robot(x, y, z):
    vector_world = np.array([[x, y, z, 1]]).T
    vector_robot = M_r_w.dot(vector_world)
    return vector_robot[0, 0], vector_robot[1, 0], vector_robot[2, 0]


# connection setup
if hardware_learning:
    # list of all incoming feeds
    incoming_id = 1001
    incoming_type = opendlv_sim_Frame

    session = OD4Session(cid=own_cid)
    session.registerMessageCallback(incoming_id, process_message, incoming_type)
    session.connect()

    serial = SerialConnection()
    sleep(2)


print("n_points: ", n_points)

population_size = 10
population = initialize_population(start_length, population_size, register_restraints)

if hardware_learning:
    from resources import best_chromosome
    population = insert_best_individual(population, best_chromosome, elite_copies)

fitness = np.zeros(population_size)
old_fitness = 0
epoch = 0
while True:
    start_time = time_now()

    # ----- Fitness evaluation
    for n, individual in enumerate(population):
        individual = individual[0]
        sum = 0

        if hardware_learning:
            print("Evaluate Fitness for individual: ", n)
            x, y, z = cube_position.position()

            x_r, y_r, z_r = transform_world_robot(x, y, z)
            theta1, theta2, theta3 = evaluate_chromosome(x=x_r, y=y_r, z=z_r, chromosome=individual,
                                                         n_variables=n_variables, constants=constants)

            x_est, y_est, z_est = forward_kinematics(theta1, theta2, theta3)

            print(x_est, y_est, z_est)

            error = 1 / evaluate_fitness(serial, theta1, theta2, theta3)
            error_theoretical = sqrt(pow(x - x_est, 2) + pow(y - y_est, 2) + pow(z - z_est, 2))

            print("Error measured: ", error)
            print("Error simulated: ", error_theoretical)
            print("Reality gap: ", abs(error - error_theoretical))
            print("---------------------------------------------")
        else:
            for point in data_points:
                x, y, z = point[0:3]
                theta1, theta2, theta3 = evaluate_chromosome(x, y, z, individual, n_variables, constants)
                x_est, y_est, z_est = forward_kinematics(theta1, theta2, theta3)
                sum += sqrt(pow(x-x_est, 2) + pow(y-y_est, 2) + pow(z-z_est, 2))

            error = sum / n_points

        fitness[n] = 1/error

    # get best chromosome
    best_indiv_index = np.argmax(fitness)
    max_fitness = fitness[best_indiv_index]
    best_indiv_chromosome = population[best_indiv_index]

    # ---- Genetic process
    if hardware_learning:
        print("Calculating Genetics")
        serial.move_to_home()
        serial.wait_for_execution()
    temp_population = []

    for i in range(0, population_size, 2):
        i1 = tournament_select(fitness, tournament_selection_parameter, ntournament_participants)
        i2 = tournament_select(fitness, tournament_selection_parameter, ntournament_participants)
        chromosome1 = population[i1]
        chromosome2 = population[i2]

        r = np.random.rand()
        if r < crossover_probability:
            new_chromosome_pair = cross(chromosome1, chromosome2)
            individualPair = new_chromosome_pair
        else:
            individual1 = chromosome1
            individual2 = chromosome2
            individualPair = [individual1, individual2]
        temp_population.extend(individualPair)

    for n, originalChromosome in enumerate(temp_population):
        mutatedChromosome = mutate(originalChromosome, mutation_probability, register_restraints)
        temp_population[n] = mutatedChromosome

    temp_population = insert_best_individual(temp_population, best_indiv_chromosome, elite_copies)

    population = temp_population.copy()

    mutationPobability = max(min_mutation_probability, mutation_decrease * mutation_probability)

    if max_fitness > old_fitness:
        print("Latest learning error: ", 1/max_fitness)
        old_fitness = max_fitness

    print("Epoch: ", epoch)
    epoch += 1

    if cube_position.x == 1000:
        if not hardware_learning:
            serial.move_to_home()
            sleep(10)
            serial.close_connection()
        break

    sleep(max(0, sleeping_time - (time_now() - start_time)))
