# Implementation of a Test Case for Linear Genetic Programming

from TestChromosome import testChromosome
from ForwardKinematics import forwardKinematics
from math import sqrt

x = 0.1
y = 0.1
z = 0.06

theta1, theta2, theta3 = testChromosome(x, y, z)
print(theta1, theta2, theta3)

x_e, y_e, z_e = forwardKinematics(theta1, theta2, theta3)

print("x_e", x_e)
print("y_e", y_e)
print("z_e", z_e)

print(sqrt((x_e - x)**2 + (y_e - y)**2 + (z_e - z)**2))
# with 0.1, 0.1, 0.06 the error is expected to be something like 0.03
