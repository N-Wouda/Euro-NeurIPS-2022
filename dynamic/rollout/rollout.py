import time

import numpy as np

from ..solve_static import solve_static
from .sim_config import sim_config
from .constants import (
    SIM_STATIC_TLIM,
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

    static_inst = info["dynamic_context"]
    ep_inst = obs["epoch_instance"]
    start_time = obs["planning_starttime"]

    sim_tlim = info["epoch_tlim"] * SIM_TLIM_FACTOR * 1000
    start = time.perf_counter()
    until = (sim_tlim - SIM_STATIC_TLIM) / 1000

    # REVIEW maybe we need to include something about the route cost
    # alongside the delta cost. the delta cost is probably not a good
    # enough indicator of how expensive a client is. We also need
    # track something about how effective a route is used.
    n_ep_requests = len(ep_inst["coords"])  # include depot for idx
    dispatch_actions = np.zeros(n_ep_requests, dtype=int)
    dispatch_costs = np.zeros(n_ep_requests, dtype=int)
    postpone_costs = np.zeros(n_ep_requests, dtype=int)

    n_simulations = 0
    while time.perf_counter() - start < until:
        sim_inst = simulate_instance(
            static_inst, ep_inst, N_LOOKAHEAD, start_time, rng
        )

        try:  # Simulations may be infeasible within TLIM
            # Sim_sol is has indices 1, ..., N
            sim_sol, _ = solve_static(sim_inst, SIM_STATIC_TLIM, **sim_config)

            # req_sol has requests indices
            req_sol = utils.sol2ep(sim_sol, sim_inst, postpone_routes=False)

            for sim_route, req_route in zip(sim_sol, req_sol):
                # Routes with req. idcs >= START_IDX are postponed
                action = 0 if (req_route >= START_IDX).any() else 1

                delta_costs = utils.delta_cost(
                    sim_route, sim_inst["duration_matrix"]
                )

                for idx, request, delta in zip(
                    sim_route, req_route, delta_costs
                ):
                    if request >= START_IDX:
                        continue

                    dispatch_actions[idx] += action

                    if action:
                        dispatch_costs[idx] += delta
                    else:
                        postpone_costs[idx] += delta

            n_simulations += 1
        except AssertionError as e:
            # print(e)
            pass

    dispatch_fraction = dispatch_actions / np.maximum(1, n_simulations)
    mean_dispatch_costs = dispatch_costs * dispatch_fraction
    mean_postpone_costs = postpone_costs * (1 - dispatch_fraction)

    mask = np.full(n_ep_requests, True)

    # Postpone clients that are often postponed and not must-dispatch
    postpone = (dispatch_fraction <= 1 - POSTPONE_THRESHOLD) & ~ep_inst[
        "must_dispatch"
    ]
    mask = np.where(postpone, False, mask)
    mask[0] = True

    return utils.filter_instance(ep_inst, mask)
