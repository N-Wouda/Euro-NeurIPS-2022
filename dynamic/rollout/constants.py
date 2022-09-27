EPOCH_N_REQUESTS = 100
EPOCH_DURATION = 3600

# Simulated requests start at 1000 idx because dynamic instance have less
# than 1000 requests total
SIM_IDX = 1000

N_LOOKAHEAD = 3  # number of lookahead simulations
SIM_TLIM_FACTOR = 0.5  # percent of epoch tlim
SIM_SOLVE_ITERS = 50  # iterations to solve a simulation instance

POSTPONE_THRESHOLD = 0.85

# Configuration to solve simulation instances
SIM_CONFIG = {
    "seed": 1,
    "generationSize": 20,
    "minPopSize": 5,
    "nbGranular": 80,
    "initialTimeWarpPenalty": 250,
    "penaltyDecrease": 0.5,
}
