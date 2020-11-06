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
from libcluon import OD4Session
from libcluon import opendlv_sim_Frame

# hand eye calibration matrix
from resources import M_r_w
from resources import best_chromosome

# genetic
from functions.genetic_parameters import *
from functions.genetic_functions import evaluate_chromosome, forward_kinematics
from functions.position_class import Position

# utilities
import numpy as np
from time import sleep
from functions.serial_connection import SerialConnection
import argparse
import logging

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
    parser.add_argument('-n', '--name', required=False, default="execution", help='logging output name')
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

    return 1/(cube_position - hand_position)


def transform_world_robot(x, y, z):
    vector_world = np.array([[x, y, z, 1]]).T
    vector_robot = M_r_w.dot(vector_world)
    return vector_robot[0, 0], vector_robot[1, 0], vector_robot[2, 0]


# setup UPD multicast
# list of all incoming feeds
incoming_id = 1001
incoming_type = opendlv_sim_Frame


session = OD4Session(cid=own_cid)
session.registerMessageCallback(incoming_id, process_message, incoming_type)
session.connect()

# initializations
serial = SerialConnection()

sleep(5)

print("Cube Position: {:.4f} | {:.4f} | {:.4f}".format(*cube_position.position()))
print("Hand Position: {:.4f} | {:.4f} | {:.4f}".format(*hand_position.position()))

x, y, z = cube_position.position()
print(f'WORLD: X: {x} | Y: {y} | Z: {z}')
x_r, y_r, z_r = transform_world_robot(x, y, z)

print(f'ROBOT: X: {x_r} | Y: {y_r} | Z: {z_r}')
theta1, theta2, theta3 = evaluate_chromosome(x=x_r, y=y_r, z=z_r, chromosome=best_chromosome, n_variables=n_variables,
                                             constants=constants)
print("Angles:")
print(theta1 * 180/3.1415, theta2 * 180/3.1415, theta3 * 180/3.1415)
print("Forward:")
x_new, y_new, z_new = forward_kinematics(theta1, theta2, theta3)
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


serial.move_to_home()
serial.wait_for_execution()
serial.close_connection()
