"""
Created by Jan Schiffeler on 18.10.20
jan.schiffeler[at]gmail.com

Changed by

MICROSERVICE for:
Finding the cube and Hubert's hand in a video feed in respect to a world coordinate system

Out:
Name | ID    sender number: description, ...
frame | 1001    0: Cube frame
frame | 1001    1: Hand frame

"""

# UPD multicast
import OD4Session
import opendlv_standard_message_set_v0_9_10_pb2

# utilities
from time import sleep
from time import time as time_now
import argparse
import logging

# core
from cv2 import aruco as arc
import cv2
import numpy as np

logging.basicConfig(level=logging.INFO)


# parameter

# general
visual_debug = True

# pattern
pattern_size = 0.0125

p_world = 13
p_cube = [0, 12, 15]
p_hand = [1, 2, 5, 7, 14]
a = 0.08  # distance axial the arm
b = 0.016  # distance radial the arm
c = 0.018/2  # half the cube length
base_cube = {0: np.array([[0, 0, -c, 1]]).T, 12: np.array([[0, 0, -c, 1]]).T, 15: np.array([[0, 0, -c, 1]]).T}
base_hand = {1: np.array([[-b, a, -c, 1]]).T, 2: np.array([[0, b, -c - a, 1]]).T,
             5: np.array([[b, 0, a - c, 1]]).T, 7: np.array([[a, 0, -b - c, 1]]).T,
             14: np.array([[a, -b, -c, 1]]).T}

# camera
camera_matrix = np.load("/home/jan/Documents/humanoid-robotics/code/computer_vision/camMat.npy")
dist_coefficients = np.load("/home/jan/Documents/humanoid-robotics/code/computer_vision/distCoeff.npy")


def create_argparser():
    """
    setup all required arguments as in:
        parser.add_argument('-s', '--side', required=False, default=0, help='some help text')
    :return: argparse dictionary
    """
    parser = argparse.ArgumentParser(description='template opendlv for python')
    parser.add_argument('-f', '--frequency', required=False, default=10, help='updating frequency')
    parser.add_argument('-c', '--cid', required=False, default=111, help='conference ID')
    parser.add_argument('-n', '--name', required=False, default="computer_vision", help='logging output name')
    args = vars(parser.parse_args())
    return args


def detect_cube_and_hand(img: np.ndarray, pat_cube: [int], pat_hand: [int], vis: bool,
                          aruco, param, cam_mat: np.ndarray, dist_coef: np.ndarray, pat_size: float) -> [np.ndarray]:
    """
    :param img: current frame
    :param pat_cube: id of the pattern involved in the cube
    :param pat_hand: id of the pattern involved in the hand
    :param vis: show visual output
    :param aruco: dictionary for arucos
    :param param: detection parameters
    :param cam_mat: camera calibration matrix
    :param dist_coef: camera distortion coefficients
    :param pat_size: size of pattern in metres
    :return: homogenous transformation matrix camera to world coodrinate system, annotated image
    """
    cube_mats = {}
    hand_mats = {}

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = arc.detectMarkers(image=gray, dictionary=aruco, parameters=param)

    if ids is not None:
        rvecs, tvecs, _ = arc.estimatePoseSingleMarkers(corners=corners, markerLength=pat_size,
                                                        cameraMatrix=cam_mat, distCoeffs=dist_coef)

        tvecs_4 = tvecs.transpose((1, 2, 0))[0]
        tvecs_4 = np.concatenate((tvecs_4, np.ones((1, tvecs.shape[0]))), axis=0)

        for n, id in enumerate(ids):
            if id in pat_cube:
                rotation = cv2.Rodrigues(rvecs[n])[0]
                M = np.zeros([4, 4], dtype=np.float32)
                M[0:3, 0:3] = rotation
                M[0:4, 3:4] = tvecs_4[:, n][:, np.newaxis]
                cube_mats[id[0]] = M
            elif id in pat_hand:
                rotation = cv2.Rodrigues(rvecs[n])[0]
                M = np.zeros([4, 4], dtype=np.float32)
                M[0:3, 0:3] = rotation
                M[0:4, 3:4] = tvecs_4[:, n][:, np.newaxis]
                hand_mats[id[0]] = M

        if vis:
            arc.drawDetectedMarkers(image=img, corners=corners, ids=ids, borderColor=(255, 255, 0))
            for j in range(np.shape(rvecs)[0]):
                rvec = rvecs[j, :]
                tvec = tvecs[j, :]
                arc.drawAxis(image=img, cameraMatrix=cam_mat, distCoeffs=dist_coef, rvec=rvec, tvec=tvec, length=0.05)

    return cube_mats, hand_mats, img


def get_world_system(img: np.ndarray, pattern_id: int, vis: bool,
                     aruco, param, cam_mat: np.ndarray, dist_coef: np.ndarray, pat_size: float) -> [np.ndarray]:
    """
    :param img: current frame
    :param pattern_id: id of the pattern used for the world coordinate system
    :param vis: show visual output
    :param aruco: dictionary for arucos
    :param param: detection parameters
    :param cam_mat: camera calibration matrix
    :param dist_coef: camera distortion coefficients
    :param pat_size: size of pattern in metres
    :return: homogenous transformation matrix camera to world coodrinate system, annotated image
    """
    # initialize empty mat
    M_cw = np.zeros((4, 4), dtype=np.float32)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = arc.detectMarkers(image=gray, dictionary=aruco, parameters=param)

    if ids is not None and pattern_id in ids:
        corner = list(corners[np.where(ids == pattern_id)[0][0]][np.newaxis, :, :])
        id = np.array([[pattern_id]])

        rvec, tvec, _ = arc.estimatePoseSingleMarkers(corners=corner, markerLength=pat_size,
                                                      cameraMatrix=cam_mat, distCoeffs=dist_coef)

        tvec_4 = np.append(tvec, 1)[:, np.newaxis]

        rotation = cv2.Rodrigues(rvec)[0]
        M_wc = np.zeros((4, 4), dtype=np.float32)
        M_wc[0:3, 0:3] = rotation
        M_wc[0:4, 3:4] = tvec_4
        M_cw = np.linalg.inv(M_wc)

        if vis:
            arc.drawDetectedMarkers(image=img, corners=corner, ids=id, borderColor=(255, 255, 0))
            arc.drawAxis(image=img, cameraMatrix=cam_mat, distCoeffs=dist_coef, rvec=rvec, tvec=tvec, length=0.05)

    return M_cw, img


def transform_and_average(M_cw: np.ndarray, base: {np.ndarray}, matrices: {np.ndarray}) -> np.ndarray:
    """
        transforms and averages coordinates to get an estimate for the position of objects
        :param M_cw: transformation matrix from camera to world coordinate system
        :param base: displacement vector from initial coordinate system
        :param matrices: homogenous transformation matrices for all involved points
        :return: averaged point coordinates in the world system
    """
    sum_vec = np.zeros([4, 1])
    for i, M in matrices.items():
        base_vec = base[i]
        t_ic = M.dot(base_vec)
        t_iw = M_cw.dot(t_ic)
        sum_vec += t_iw

    if len(matrices) != 0:
        return sum_vec / len(matrices)
    else:
        return np.array([[500, 500, 500, 1]]).T


# handling setup arguments
arg = create_argparser()
logger = logging.getLogger(arg["name"])

own_cid = int(arg['cid'])
print(f"CID: {own_cid}")
freq = int(arg['frequency'])
sleeping_time = 1/freq
print(f"Frequency: {freq}")

# setup UPD multicast
output_id = 1001
output_type = opendlv_standard_message_set_v0_9_10_pb2.opendlv_sim_Frame()

session = OD4Session.OD4Session(cid=own_cid)
session.connect()


# openCV detection setup

# Initialize pattern
marker_dict_type = arc.DICT_6X6_250
aruco_dict = arc.Dictionary_get(marker_dict_type)
aruco_parameters = arc.DetectorParameters_create()

# Initialize Camera
cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)

# greeting
ret, frame = cam.read()
assert ret, "Cannot connect with camera"
height, width, _ = frame.shape
print(f'Camera found. Height: {height} | Width: {width}')

# find camera position concerned to world coordinates
max_tries = 5
for i in range(max_tries):
    ret, frame = cam.read()
    T_wc, frame = get_world_system(img=frame, pattern_id=p_world, vis=visual_debug, aruco=aruco_dict,
                                   param=aruco_parameters, cam_mat=camera_matrix, dist_coef=dist_coefficients,
                                   pat_size=0.023)
    if T_wc.any():
        print(f"Pattern position | X: {T_wc[0, 0]}  Y: {T_wc[0, 1]}  Z: {T_wc[0, 2]} ")
        break
    else:
        if i == max_tries-1:
            print(f"Not pattern found in {max_tries} attempts")
            cam.release()
            cv2.destroyAllWindows()
            quit()
        print(f"Failed to detect pattern {p_world}")
        if visual_debug:
            cv2.imshow("Detected World frame",
                       cv2.resize(frame, (int(0.6 * width), int(0.6 * height)), interpolation=cv2.INTER_LINEAR))
            cv2.waitKey(0)
            cv2.destroyWindow("Detected World frame")

while True:
    start_time = time_now()

    # marker detection
    ret, frame = cam.read()
    assert ret, "Cannot connect with camera"
    cube_matrices, hand_matrices, frame = detect_cube_and_hand(frame, p_cube, p_hand, visual_debug,
                                                               aruco=aruco_dict, param=aruco_parameters,
                                                               cam_mat=camera_matrix, dist_coef=dist_coefficients,
                                                               pat_size=pattern_size)

    cube_position = transform_and_average(M_cw=T_wc, base=base_cube, matrices=cube_matrices)
    hand_position = transform_and_average(M_cw=T_wc, base=base_hand, matrices=hand_matrices)

    # show image output
    if visual_debug:
        t = T_wc[0:4, 3:4]
        r = cv2.Rodrigues(T_wc[0:3, 0:3])
        f_x = camera_matrix[0, 0]
        f_y = camera_matrix[1, 1]

        if cube_position.any():
            cube_position_2 = np.linalg.inv(T_wc).dot(cube_position)
            x = int(round(f_x * cube_position_2[0, 0] / cube_position_2[2, 0])) + 800
            y = int(round(f_y * cube_position_2[1, 0] / cube_position_2[2, 0])) + 680
            frame = cv2.circle(frame, (x, y), 5, (200, 0, 200), 10)
            message = f"Cube Pose    | X: {cube_position[0]}  Y: {cube_position[1]}  Z: {cube_position[2]}"
            cv2.putText(frame, message, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        if hand_position.any():
            hand_position_2 = np.linalg.inv(T_wc).dot(hand_position)
            x = int(round(f_x * hand_position_2[0, 0] / hand_position_2[2, 0])) + 800
            y = int(round(f_y * hand_position_2[1, 0] / hand_position_2[2, 0])) + 680
            frame = cv2.circle(frame, (x, y), 5, (200, 200, 0), 10)
            message = f"Hand Pose | X: {hand_position[0]}  Y: {hand_position[1]}  Z: {hand_position[2]}"
            cv2.putText(frame, message, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

        cv2.imshow("Frame", cv2.resize(frame, (int(0.6*width), int(0.6*height)), interpolation=cv2.INTER_LINEAR))
    key = cv2.waitKey(10)
    if key == ord('q'):
        print('Closing down')
        output_type.x = 1000
        session.send(output_id, output_type.SerializeToString(), senderStamp=0)
        break

    # send data
    output_type.x = float(cube_position[0, 0])
    output_type.y = float(cube_position[1, 0])
    output_type.z = float(cube_position[2, 0])
    session.send(output_id, output_type.SerializeToString(), senderStamp=0)

    output_type.x = float(hand_position[0, 0])
    output_type.y = float(hand_position[1, 0])
    output_type.z = float(hand_position[2, 0])
    session.send(output_id, output_type.SerializeToString(), senderStamp=1)

    sleep(max(0, sleeping_time - (time_now() - start_time)))

cam.release()
cv2.destroyAllWindows()
print('End')
