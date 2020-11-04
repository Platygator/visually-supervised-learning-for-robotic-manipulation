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
import OD4Session
import opendlv_standard_message_set_v0_9_10_pb2

# utilities
import numpy as np
from time import sleep
from time import time as time_now
from SerialConnection import SerialConnection
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
    ser.run_serial(body, shoulder, elbow)
    sleep(5)

    if cube_position.x == 500 or hand_position.x == 500:
        # give bad fitness
        return 0.0

    error = cube_position - hand_position

    return 1/error


def transform_world_robot(x, y, z):
    M_r_w = np.load('/home/jan/Documents/humanoid-robotics/code/utility/M_r_w.npy')
    vector_world = np.array([[x, y, z, 1]]).T
    vector_robot = M_r_w.dot(vector_world)
    return vector_robot[0, 0], vector_robot[1, 0], vector_robot[2, 0]


# setup UPD multicast
# list of all incoming feeds
incoming_id = 1001
incoming_type = opendlv_standard_message_set_v0_9_10_pb2.opendlv_sim_Frame

# list of all outgoing feeds
outgoing_id = 1001
outgoing_type = opendlv_standard_message_set_v0_9_10_pb2.opendlv_sim_Frame()

session = OD4Session.OD4Session(cid=own_cid)
session.registerMessageCallback(incoming_id, process_message, incoming_type)
session.connect()

# initializations

serial = SerialConnection()

sleep(5)


# initialize genetic code
from genetic_parameters import *
from EvaluateChromosome import evaluateChromosome
from ForwardKinematics import forwardKinematics

# Load Chromosome
test_chromosome = np.load("/home/jan/Documents/humanoid-robotics/code/utility/bestChromosomMinimal.npy")

print("Cube Position: {:.4f} | {:.4f} | {:.4f}".format(*cube_position.position()))
print("Hand Position: {:.4f} | {:.4f} | {:.4f}".format(*hand_position.position()))

x, y, z = cube_position.position()
print(f'WORLD: X: {x} | Y: {y} | Z: {z}')
x_r, y_r, z_r = transform_world_robot(x, y, z)
# x_r = 0.25
# y_r = 0.06
# z_r = 0.16
print(f'ROBOT: X: {x_r} | Y: {y_r} | Z: {z_r}')
theta1, theta2, theta3 = evaluateChromosome(x=x_r, y=y_r, z=z_r, chromosome=test_chromosome, nVariables=nVariables,
                                            constants=constants)
print("Angles:")
print(theta1 * 180/3.1415, theta2 * 180/3.1415, theta3 * 180/3.1415)
print("Forward:")
x_new, y_new, z_new = forwardKinematics(theta1, theta2, theta3)
print('{1}, {0}, {2}'.format(-y_new, -x_new, z_new))
# move and grab
serial.run_serial(theta1, theta2, theta3, False)
serial.wait_for_execution()
serial.run_serial(theta1, theta2, theta3, True)
serial.wait_for_execution()
serial.move_to_home()
serial.wait_for_execution()
# read data
print("Cube Position: {:.4f} | {:.4f} | {:.4f}".format(*cube_position.position()))
print("Hand Position: {:.4f} | {:.4f} | {:.4f}".format(*hand_position.position()))



# send data
outgoing_type.roll = 123.45
outgoing_type.pitch = 123.45
outgoing_type.yaw = 123.45

session.send(outgoing_id, outgoing_type.SerializeToString(), senderStamp=3)

serial.move_to_home()
serial.wait_for_execution()
serial.close_connection()
