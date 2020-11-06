"""
Created by Jan Schiffeler on 16.04.20
jan.schiffeler[at]gmail.com

Changed by

MICROSERVICE for:
Description

In:
Name | ID    sender number: description, ...

Out:
Name | ID    sender number: description, ...

Information about message types
https://github.com/chalmers-revere/opendlv.standard-message-set/blob/master/opendlv.odvd#L113-L115
1. Delete areas in ######## if no shared memory is used
(2. If using shared memory change name)
3. define in- and outcoming message types and add description to docstring
4. Adapt process_message and message_info
5. Adapt create_argparse and get its values in the initialization OR if not needed delete completely
6. Write initialization and loop part

"""

# UPD multicast
import OD4Session
import opendlv_standard_message_set_v0_9_10_pb2

# list of all incoming feeds
incoming_id_1 = 1039
incoming_type_1 = opendlv_standard_message_set_v0_9_10_pb2.opendlv_proxy_DistanceReading

# list of all outgoing feeds. !!!! Call message function ()!!!!!
outgoing_id_1 = 1038
outgoing_type_1 = opendlv_standard_message_set_v0_9_10_pb2.opendlv_proxy_AngleReading()


# shared memory ################################################################
import sysv_ipc
import cv2
file_name = "/tmp/img.argb"  # This name must match with the name used in the h264-decoder-viewer.yml file.
################################################################################

# utilities
import numpy as np
from time import sleep
from time import time as time_now
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
    parser.add_argument('-f', '--frequency', required=False, default=10, help='updating frequency')
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
sleeping_time = 1/freq
print(f"Frequency: {freq}")

# callback function for receive handling
message_info = {"some": 0.0, "date": 0.0}


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
        message_info["some"] = msg.distance
        # distance is the name of a entry in the msg
    if senderStamp == 1:
        message_info["date"] = msg.distance


# setup UPD multicast
session = OD4Session.OD4Session(cid=own_cid)
session.registerMessageCallback(incoming_id_1, process_message, incoming_type_1)
session.connect()


# setup shared memory ###########################################################
keySharedMemory = sysv_ipc.ftok(file_name, 1, True)
keySemMutex = sysv_ipc.ftok(file_name, 2, True)
keySemCondition = sysv_ipc.ftok(file_name, 3, True)
shm = sysv_ipc.SharedMemory(keySharedMemory)
mutex = sysv_ipc.Semaphore(keySemCondition)
cond = sysv_ipc.Semaphore(keySemCondition)
################################################################################


# initializations


while True:
    start_time = time_now()

    # Wait for next notification. ###################################################
    cond.Z()
    logger.info("Received new frame.")

    # getting the latest image in buffer
    mutex.acquire()
    shm.attach()
    buf = shm.read()
    shm.detach()
    mutex.release()

    # turn buffer image into OpenCV readable image
    img = np.frombuffer(buf, np.uint8).reshape(720, 1280, 4)

    ########################################################
                    ##### CV Part here #####
    cv2.imshow("image", img)
    cv2.waitKey(2)
    ########################################################

    ################################################################################

    ########################################################
                    ##### UPD Part here #####

    # read data
    print(message_info["some"])
    print(message_info["date"])

    # send data
    outgoing_type_1.angle = 123.45

    session.send(outgoing_id_1, outgoing_type_1.SerializeToString(), senderStamp=0)
    ########################################################

    sleep(sleeping_time - (time_now() - start_time))
