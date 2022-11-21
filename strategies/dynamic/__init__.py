from .baselines import greedy, lazy, random
from .simulate import simulate

# This dict stores strategies that can be used to decide which requests to
# dispatch in each epoch. A strategy is a function that takes the following
# arguments:
#
# * static_info: static info, including base instance and number of epochs.
# * observation: the realisations for the current epoch.
# * rng: a seeded random number generator.
# * kwargs: any additional keyword arguments taken from the configuration
#           object's strategy parameters.
#
# Using these arguments, the strategy should return which requests to dispatch
# in the current epoch.
STRATEGIES = {
    "greedy": greedy,
    "lazy": lazy,
    "random": random,
    "simulate": simulate,
}
