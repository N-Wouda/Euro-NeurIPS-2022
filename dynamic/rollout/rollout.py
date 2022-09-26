import time

import numpy as np

from ..solve_static import solve_static
from .sim_config import sim_config
from .constants import (
    SIM_SOLVE_ITERS,
    SIM_TLIM_FACTOR,
    START_IDX,
    N_LOOKAHEAD,
    POSTPONE_THRESHOLD,
)

from .. import utils
from .simulate_instance import simulate_instance


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
    static_inst = info["dynamic_context"]
    ep_inst = obs["epoch_instance"]
    planning_start_time = obs["planning_starttime"]
    n_requests = len(ep_inst["coords"])
    sim_tlim = info["epoch_tlim"] * SIM_TLIM_FACTOR

    # Statistics
    n_simulations = 0
    dispatch_actions = np.zeros(n_requests, dtype=int)

    while time.perf_counter() - start < sim_tlim:
        sim_inst = simulate_instance(
            static_inst, ep_inst, N_LOOKAHEAD, planning_start_time, rng
        )

        try:  # TODO create new solver that doesnt raise infeasibility error
            # sim_sol is has indices 1, ..., N
            sim_sol, _ = solve_static(
                sim_inst, max_iterations=SIM_SOLVE_ITERS, **sim_config
            )

            # req_sol has requests indices
            req_sol = utils.sol2ep(sim_sol, sim_inst, postpone_routes=False)

            for sim_route, req_route in zip(sim_sol, req_sol):
                # Routes with req. idcs < START_IDX are dispatched now
                # TODO could be made faster if we check any & use postpone_actions
                if all(req_route < START_IDX):
                    dispatch_actions[sim_route] += 1

            n_simulations += 1
        except AssertionError as e:
            pass

    print(n_simulations)

    # Postpone clients that are often postponed and non-urgent
    dispatch_fraction = dispatch_actions / np.maximum(1, n_simulations)
    postpone = dispatch_fraction <= 1 - POSTPONE_THRESHOLD
    non_urgent = ~ep_inst["must_dispatch"]

    # Mask contains True for dispatch, False for postpone
    mask = np.full(n_requests, True)
    mask = np.where(postpone & non_urgent, False, mask)
    mask[0] = True

    return utils.filter_instance(ep_inst, mask)
