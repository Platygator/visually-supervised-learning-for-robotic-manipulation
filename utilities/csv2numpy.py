# Implementation to convert a CSV file to a NPY file

import numpy as np


def convert_csv_to_npy(input_file):
    output_file = input_file.split(".")[0] + ".npy"
    data = np.genfromtxt(input_file, delimiter=',') - 1
    np.save(output_file, data)


convert_csv_to_npy("resources/bestChromosome.csv")
