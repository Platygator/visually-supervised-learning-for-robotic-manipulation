# Implementation to Insert Best Individual

def InsertBestIndividual(population, bestIndividual, nCopiesBest):
    """
    Function to Insert Best Individual
    
    Parameters
    ----------
    population      : Population
    bestIndividual  : Best Individual
    nCopiesBest     : Number of Copies of Best Individual

    Returns
    -------
    modifiedPopulation  : Modified Population
    """

    for i in range(nCopiesBest):
        population[i] = bestIndividual

    return population