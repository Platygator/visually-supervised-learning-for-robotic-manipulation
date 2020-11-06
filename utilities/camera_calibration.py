"""
Created by Jan Schiffeler on 15.09.20
jansch[at]student.chalmers.se

Changed by
Jan Paul Theune
theune[at]student.chalmers.se

Calibration procedure for stereo cameras using a ChArUco pattern.

Python 3.8.2
Library version:
cv2 - 4.2.0

"""

import cv2
import cv2.aruco as arc
import numpy as np

import platform
cool_people_use_unix = platform.system() == "Darwin" or platform.system() == "Linux"

# board specifications
num_blocks_height = 7
num_blocks_width = 5
length_checker = 0.041  # [m]
length_aruco = 0.027  # [m]

parameters = arc.DetectorParameters_create()
parameters.minDistanceToBorder = 40  # 3
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
dictionary = arc.getPredefinedDictionary(arc.DICT_6X6_250)
board = arc.CharucoBoard_create(num_blocks_width, num_blocks_height, length_checker, length_aruco, dictionary)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*8, 3), np.float32)
objp[:, :2] = np.mgrid[0:8, 0:6].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images.
cornerpoints = []
aruco_ids = []

if cool_people_use_unix:

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
else:
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)

image_count = 0
ret, img = cap.read()
assert ret, "Cannot connect with camera"
width, height, _ = img.shape

font = cv2.FONT_HERSHEY_SIMPLEX

import glob
image_list = glob.glob('./calib_set_18_10/*.jpg')
img_list = []
while True:
    ret, img = cap.read()
    assert ret, "Cannot connect with camera"
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Find aruco markers in the query image
    corners, ids, _ = arc.detectMarkers(image=gray, dictionary=dictionary)

    # Outline the aruco markers found in our query image
    img = arc.drawDetectedMarkers(image=img, corners=corners)

    if len(corners) > 10:
        # Get charuco corners and ids from detected aruco markers
        response, charuco_corners, charuco_ids = arc.interpolateCornersCharuco(
            markerCorners=corners,
            markerIds=ids,
            image=gray,
            board=board)

        # Draw the Charuco board we've detected to show our calibrator the board was properly detected
        img = arc.drawDetectedCornersCharuco(
            image=img,
            charucoCorners=charuco_corners,
            charucoIds=charuco_ids)

        if cv2.waitKey(1) & 0xFF == ord('s'):
            # Add these corners and ids to our calibration arrays
            cornerpoints.append(charuco_corners)
            aruco_ids.append(charuco_ids)
            img_list.append(img)
            print("Saved image")
            image_count += 1

    cv2.putText(img, f"Images taken: {image_count}", (10, 50), font, 0.8, (0, 0, 255), 2, cv2.LINE_AA)
    cv2.imshow("Frame", cv2.resize(img, (int(0.6 * height), int(0.6 * width)), interpolation=cv2.INTER_LINEAR))
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        cap.release()
        break

image_size = img.shape[:2]


for n, img in enumerate(img_list):
    cv2.imshow("Selection", cv2.resize(img, (int(0.6 * height), int(0.6 * width)), interpolation=cv2.INTER_LINEAR))
    key = cv2.waitKey(0)
    if key == ord('q'):
        break
    elif key == ord('d'):
        del cornerpoints[n]
        del aruco_ids[n]
    else:
        continue

cv2.destroyAllWindows()
if aruco_ids:
    print("Starting Calibration")
    calibration, cameraMatrix, distCoeffs, rvecs, tvecs = arc.calibrateCameraCharuco(
        charucoCorners=cornerpoints,
        charucoIds=aruco_ids,
        board=board,
        imageSize=image_size,
        cameraMatrix=None,
        distCoeffs=None)

    # img = cv2.circle(img, tuple(cornerpoints[0][0][0]), 5, (100, 200, 50), 5)
    # cv2.imshow("qwekn", img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # Print matrix and distortion coefficient to the console
    print(cameraMatrix)
    np.save("camMat", cameraMatrix)
    print(distCoeffs)
    np.save("distCoeff", distCoeffs)
else:
    print("There where no images found")

