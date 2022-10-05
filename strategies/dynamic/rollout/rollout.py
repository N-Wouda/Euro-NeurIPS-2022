import time

import numpy as np

from strategies.utils import filter_instance
from .constants import (
    DISPATCH_THRESHOLD,
    N_LOOKAHEAD,
    SIM_SOLVE_CONFIG,
    SIM_SOLVE_ITERS,
    SIM_TLIM_FACTOR,
)
from .simulate_instance import simulate_instance
from .solve_simulation import solve_simulation


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
    must_dispatch = set(np.flatnonzero(ep_inst["must_dispatch"]))

    # Statistics
    n_sims = 0
    avg_duration = 0.0
    dispatch_count = np.zeros(n_requests, dtype=int)

    # Only do another simulation if there's (on average) enough time for it to
    # complete before the time limit.
    while (sim_start := time.perf_counter()) + avg_duration < start + sim_tlim:
        sim_inst = simulate_instance(info, obs, rng, N_LOOKAHEAD)

        # Epoch requests have index between 1 and n_requests in sim_sol,
        # whereas simulated requests have index largen than n_requests.
        sim_sol, _ = solve_simulation(
            sim_inst, SIM_SOLVE_ITERS, **SIM_SOLVE_CONFIG
        )

        for sim_route in sim_sol:
            # Only dispatch routes that contain must dispatch requests
            if any(idx in must_dispatch for idx in sim_route):
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

    return filter_instance(ep_inst, dispatch)
