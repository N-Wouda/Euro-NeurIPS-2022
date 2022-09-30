import time

import numpy as np

from .constants import (
    DISPATCH_THRESHOLD,
    N_LOOKAHEAD,
    SIM_CONFIG,
    SIM_SOLVE_ITERS,
    SIM_TLIM_FACTOR,
)
from .simulate_instance import simulate_instance
from .solve_simulation import solve_simulation
from .. import utils


def rollout(info, obs, rng):
    """
    Determine the dispatch instance by simulating the next epochs and analyzing
    those simulations.
    """
    # Return the full epoch instance for first and last epoch
    if obs["current_epoch"] in [info["start_epoch"], info["end_epoch"]]:
        return obs["epoch_instance"]

    start = time.perf_counter()

    # Parameters
    ep_inst = obs["epoch_instance"]
    n_requests = ep_inst["is_depot"].size
    sim_tlim = info["epoch_tlim"] * SIM_TLIM_FACTOR

    # Statistics
    n_sims = 0
    avg_duration = 0.
    dispatch_count = np.zeros(n_requests, dtype=int)

    # Only do another simulation if there's (on average) enough time for it to
    # complete before the time limit.
    while (sim_start := time.perf_counter()) + avg_duration < start + sim_tlim:
        sim_inst = simulate_instance(info, obs, rng, N_LOOKAHEAD)

        # sim_sol has indices 1, ..., N
        sim_sol, _, is_feas = solve_simulation(
            sim_inst, SIM_SOLVE_ITERS, **SIM_CONFIG
        )

        if not is_feas:
            continue

        # req_sol has requests indices
        req_sol = utils.sol2ep(sim_sol, sim_inst, postpone_routes=False)

        for route_idx, sim_route in enumerate(sim_sol):
            # The requests in the route are postponed oOnly if the route
            # contains simulated requests (identified by negative index).
            if (req_sol[route_idx] >= 0).all():
                dispatch_count[sim_route] += 1

        sim_duration = time.perf_counter() - sim_start
        avg_duration = (n_sims * avg_duration + sim_duration) / (n_sims + 1)
        n_sims += 1

    dispatch = (
        ep_inst["is_depot"]
        | ep_inst["must_dispatch"]
        # Only dispatch requests that are dispatched in enough simulations
        | (dispatch_count >= max(1, n_sims) * DISPATCH_THRESHOLD)
    )

    return utils.filter_instance(ep_inst, dispatch)
