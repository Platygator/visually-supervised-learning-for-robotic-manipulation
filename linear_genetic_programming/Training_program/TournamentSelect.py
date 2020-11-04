# Implementation of Tournament Selection Function

import numpy as np

def TournamentSelect(fitness, pTournament, nParticipants):
    """
    Function to Perform Tournament Selection Function
    
    Parameters
    ----------
    fitness         : Fitness Function
    pTournament     : Probability of Tournament
    nParticipants   : Number of Participants

    Returns
    -------
    selected    : Selected Individual
    """

    populationSize = fitness.shape[0]
    participantFitness = np.zeros([2, nParticipants])

    for i in range(nParticipants):
        participantFitness[0, i] = int(np.random.rand() * populationSize)
        participantFitness[1, i] = fitness[int(participantFitness[1, i])]

    fitness_ind = np.argsort(participantFitness[1])[::-1]

    participantOrder = participantFitness[0, fitness_ind]

    randomness = np.random.rand()
    winner = 0

    while randomness > pTournament:
        winner = winner + 1
        if winner == nParticipants - 1:
            break
        randomness = np.random.rand()
    
    selected = int(participantOrder[winner])

    return selected
