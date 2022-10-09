EPOCH_N_REQUESTS = 100
EPOCH_DURATION = 3600

N_LOOKAHEAD = 3  # number of lookahead simulations
SIM_TLIM_FACTOR = 0.6  # percent of epoch tlim
SIM_SOLVE_ITERS = 45  # iterations to solve a simulation instance

DISPATCH_THRESHOLD = 0.20

# Configuration to solve simulation instances
SIM_SOLVE_CONFIG = {
    "generationSize": 2,
    "minPopSize": 1,
    "nbGranular": 70,
    "initialTimeWarpPenalty": 25,
    "penaltyDecrease": 0.25,
    "nbPenaltyManagement": 1,
    "repairProbability": 0,
}
