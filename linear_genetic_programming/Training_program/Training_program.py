"""
Created by Jan Schiffeler on 18.10.20
jan.schiffeler[at]gmail.com

Changed by

MICROSERVICE for:
Finding gripping angles for a desired position

In:
Name | ID    sender number: description, ...
frame | 1001    1: Cube frame
frame | 1001    2: Hand frame

Out:
Name | ID    sender number: description, ...
frame | 1001    3: Angles

"""

# UPD multicast
hardware_learning = True

if hardware_learning:
    import OD4Session
    import opendlv_standard_message_set_v0_9_10_pb2
    from SerialConnection import SerialConnection

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
    parser.add_argument('-n', '--name', required=False, default="python-template", help='logging output name')
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


class Position:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def position(self):
        return self.x, self.y, self.z

    def __sub__(self, other):
        return sqrt(pow(self.x - other.x, 2) + pow(self.y - other.y, 2) + pow(self.z - other.z, 2))


# callback function for receive handling
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

    error = cube_position - hand_position

    return 1/error


def transform_world_robot(x, y, z):
    M_r_w = np.load('/home/jan/Documents/humanoid-robotics/code/utility/M_r_w.npy')
    vector_world = np.array([[x, y, z, 1]]).T
    vector_robot = M_r_w.dot(vector_world)
    return vector_robot[0, 0], vector_robot[1, 0], vector_robot[2, 0]


# setup UPD multicast
# list of all incoming feeds
if hardware_learning:
    incoming_id = 1001
    incoming_type = opendlv_standard_message_set_v0_9_10_pb2.opendlv_sim_Frame

    # list of all outgoing feeds
    outgoing_id = 1001
    outgoing_type = opendlv_standard_message_set_v0_9_10_pb2.opendlv_sim_Frame()

    session = OD4Session.OD4Session(cid=own_cid)
    session.registerMessageCallback(incoming_id, process_message, incoming_type)
    session.connect()

# initializations
if hardware_learning:
    serial = SerialConnection()
    sleep(2)



# initialize population
from genetic_parameters import *
from EvaluateChromosome import evaluateChromosome
from Mutate import mutate
from Cross import cross
from InsertBestIndividual import InsertBestIndividual
from InitializePopulation import initializePopulation
from ForwardKinematics import forwardKinematics
from TournamentSelect import TournamentSelect

print("nPoints: ", nPoints)

populationSize = 10
population = initializePopulation(startLength, populationSize, registerRestraints)

if hardware_learning:
    best_indiv_chromosome = np.load('/home/jan/Documents/humanoid-robotics/code/utility/bestChromosomMinimal.npy')[np.newaxis, :]
    # population = InsertBestIndividual(population, best_indiv_chromosome, eliteCopies)

fitness = np.zeros(populationSize)
old_fitness = 0
epoch = 0
while True:
    start_time = time_now()

    #### Fitness evaluation
    for n, individual in enumerate(population):
        individual = individual[0]
        sum = 0

        if hardware_learning:
            print("Evaluate Fitness for indiv: ", n)
            x, y, z = cube_position.position()

            x_r, y_r, z_r = transform_world_robot(x, y, z)
            theta1, theta2, theta3 = evaluateChromosome(x=x_r, y=y_r, z=z_r, chromosome=individual,
                                                        nVariables=nVariables, constants=constants)

            x_est, y_est, z_est = forwardKinematics(theta1, theta2, theta3)

            print(x_est, y_est, z_est)

            error = 1 / evaluate_fitness(serial, theta1, theta2, theta3)
            error_theoretical = sqrt(pow(x - x_est, 2) + pow(y - y_est, 2) + pow(z - z_est, 2))

            print("Error measured: ", error)
            print("Error simulated: ", error_theoretical)
            print("Reality gap: ", abs(error - error_theoretical))
            print("---------------------------------------------")
        else:
            for point in dataPoints:
                x, y, z = point[0:3]
                theta1, theta2, theta3 = evaluateChromosome(x, y, z, individual, nVariables, constants)
                x_est, y_est, z_est = forwardKinematics(theta1, theta2, theta3)
                sum += sqrt(pow(x-x_est, 2) + pow(y-y_est, 2) + pow(z-z_est, 2))

            error = sum/nPoints

        fitness[n] = 1/error

    # get best chromosome
    best_indiv_index = np.argmax(fitness)
    max_fitness = fitness[best_indiv_index]
    best_indiv_chromosome = population[best_indiv_index]

    ##### Genetic process
    if hardware_learning:
        print("Calculating Genetics")
        serial.move_to_home()
        serial.wait_for_execution()
    temp_population = []

    for i in range(0, populationSize, 2):
        i1 = TournamentSelect(fitness, tournamentSelectionParameter, ntournamentParticipants)
        i2 = TournamentSelect(fitness, tournamentSelectionParameter, ntournamentParticipants)
        chromosome1 = population[i1]
        chromosome2 = population[i2]

        r = np.random.rand()
        if r < crossoverProbability:
            newChromosomePair = cross(chromosome1, chromosome2)
            individualPair = newChromosomePair
        else:
            individual1 = chromosome1
            individual2 = chromosome2
            individualPair = [individual1, individual2]
        temp_population.extend(individualPair)

    for n, originalChromosome in enumerate(temp_population):
        mutatedChromosome = mutate(originalChromosome, mutationProbability, registerRestraints)
        temp_population[n] = mutatedChromosome

    temp_population = InsertBestIndividual(temp_population, best_indiv_chromosome, eliteCopies)

    population = temp_population.copy()

    mutationPobability = max(minMutationProbability, mutationDecrease*mutationProbability)

    if max_fitness > old_fitness:
        print("Latest learning error: ", 1/max_fitness)
        old_fitness = max_fitness

    print("Epoch: ", epoch)
    epoch += 1


    # read data
    # print("Cube Position: {:.4f} | {:.4f} | {:.4f}".format(*cube_position.position()))
    # print("Hand Position: {:.4f} | {:.4f} | {:.4f}".format(*hand_position.position()))

    if cube_position.x == 1000:
        if not hardware_learning:
            serial.move_to_home()
            sleep(10)
            serial.close_connection()
        break

    if hardware_learning:
        # send data
        outgoing_type.roll = 123.45
        outgoing_type.pitch = 123.45
        outgoing_type.yaw = 123.45

        session.send(outgoing_id, outgoing_type.SerializeToString(), senderStamp=3)

    sleep(max(0, sleeping_time - (time_now() - start_time)))
