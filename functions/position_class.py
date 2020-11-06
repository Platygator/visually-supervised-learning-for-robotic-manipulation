"""
Created by Jan Schiffeler on 05.11.20
jan.schiffeler[at]gmail.com

Changed by


"""

from math import sqrt


class Position:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def position(self):
        return self.x, self.y, self.z

    def __sub__(self, other):
        return sqrt(pow(self.x - other.x, 2) + pow(self.y - other.y, 2) + pow(self.z - other.z, 2))