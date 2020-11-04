# Implementation for Computing Operation

import math
import numpy as np

def computerOperation(gene, register):
    """
    Function to Computer Operation

    Parameters
    ----------
    gene        : Gene for which the operation must be computed
    register    : Register to Store The Updated Values

    Returns
    -------
    updatedRegister   : Register with Updated Values
    """
    
    updatedRegister = register.copy()
    operand1 = updatedRegister[int(gene[2])]
    operand2 = updatedRegister[int(gene[3])]
    saveTo = int(gene[1])

    if (gene[0] == 0):
        updatedRegister[saveTo] = operand1 + operand2
        
    elif (gene[0] == 1):
        updatedRegister[saveTo] = operand1 - operand2

    elif (gene[0] == 2):
        updatedRegister[saveTo] = operand1 * operand2

    elif (gene[0] == 3):
        if operand2 != 0:
            updatedRegister[saveTo] = operand1 / operand2
        else:
            updatedRegister[saveTo] = 1e20

    elif (gene[0] == 4):
        if np.absolute(operand1) > 1:
            updatedRegister[saveTo] = 1e20
        else:
            updatedRegister[saveTo] = math.asin(operand1)

    elif (gene[0] == 5):
        if np.absolute(operand1) > 1:
            updatedRegister[saveTo] = 1e20
        else:
            updatedRegister[saveTo] = math.acos(operand1)

    elif (gene[0] == 6):
        updatedRegister[saveTo] = math.sin(operand1)

    elif (gene[0] == 7):
        updatedRegister[saveTo] = math.cos(operand1)

    else:
        raise ValueError("Operation for Requested Parameter Not Defined!")

    return updatedRegister