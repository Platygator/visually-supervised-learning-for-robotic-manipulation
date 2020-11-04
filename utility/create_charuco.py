"""
Created by Jan Schiffeler on 15.09.20
jan.schiffeler[at]gmail.com

Changed by



Python 3.8.
Library version:


"""

import cv2
import cv2.aruco as arc

A4_width = 0.297
A4_height = 0.210
num_blocks_height = 7
num_blocks_width = 5
length_checker = round(min(A4_height/ num_blocks_height, A4_width / num_blocks_width), 3)  # [m]
print(length_checker)
length_aruco = round(length_checker * 0.6, 3)  # [m]
print(length_aruco)
length_checker = 0.041
length_aruco = 0.027

parameters = arc.DetectorParameters_create()
parameters.minDistanceToBorder = 40  # 3
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
dictionary = arc.getPredefinedDictionary(arc.DICT_6X6_250)
board = arc.CharucoBoard_create(num_blocks_width, num_blocks_height, length_checker, length_aruco, dictionary)

# mat = arc.drawPlanarBoard(board, (2480, 3508))
mat = board.draw((2480, 3508))
cv2.imwrite("ChAruCo_board.png", mat)
# cv2.imshow("ChAruCo", mat)
# cv2.waitKey()
# cv2.destroyAllWindows()
