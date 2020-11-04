# Implementation to convert a CSV file to a NPY file

import numpy as np

def convertCsvToNpy(inputFile):
    outputFile = inputFile.split(".")[0] + ".npy"
    data = np.genfromtxt(inputFile, delimiter=',') - 1
    np.save(outputFile, data)

convertCsvToNpy("bestChromosomMinimal.csv")