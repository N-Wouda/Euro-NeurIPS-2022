import time

import numpy as np

from .constants import (
    SIM_SOLVE_ITERS,
    SIM_TLIM_FACTOR,
    SIM_SOLVE_CONFIG,
    N_LOOKAHEAD,
    POSTPONE_THRESHOLD,
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
    n_simulations = 0
    dispatch_count = np.zeros(n_requests, dtype=int)

    while time.perf_counter() - start < sim_tlim:
        sim_inst = simulate_instance(info, obs, rng, N_LOOKAHEAD)

        # sim_sol is has indices 1, ..., N
        sim_sol, _, is_feas = solve_simulation(
            sim_inst, SIM_SOLVE_ITERS, **SIM_SOLVE_CONFIG
        )

        if not is_feas:
            continue

        # req_sol has requests indices
        req_sol = utils.sol2ep(sim_sol, sim_inst, postpone_routes=False)

        for route_idx, sim_route in enumerate(sim_sol):
            # Routes that contain simulated requests (negative index) are postponed
            if any(req_sol[route_idx] < 0):
                continue

            dispatch_count[sim_route] += 1

        n_simulations += 1

    # Postpone requests that are often postponed in simulations
    dispatch_fraction = dispatch_count / max(1, n_simulations)
    postpone = dispatch_fraction <= 1 - POSTPONE_THRESHOLD
    dispatch = ep_inst["is_depot"] | ep_inst["must_dispatch"] | ~postpone

    return utils.filter_instance(ep_inst, dispatch)
