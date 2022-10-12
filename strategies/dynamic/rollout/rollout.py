import time

import numpy as np

import hgspy
from strategies.static import hgs
from strategies.utils import filter_instance
from .simulate_instance import simulate_instance


def rollout(
    info,
    obs,
    rng,
    n_lookahead: int,
    n_requests: int,
    n_update_threshold: int,
    sim_tlim_factor: float,
    sim_solve_iters: int,
    dispatch_threshold: float,
    sim_config: dict,
    node_ops: list,
    route_ops: list,
    crossover_ops: list,
    **kwargs,
):
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
    sim_tlim = info["epoch_tlim"] * sim_tlim_factor
    must_dispatch = set(np.flatnonzero(ep_inst["must_dispatch"]))
    n_ep_reqs = ep_inst["is_depot"].size

    # Statistics
    n_sims = 0
    avg_duration = 0.0
    dispatch_count = np.zeros(n_ep_reqs, dtype=int)
    postpone_idcs = np.zeros(n_ep_reqs, dtype=int)

    # Initial solution based on epoch instance
    res_init = hgs(
        ep_inst,
        hgspy.Config(**sim_config),
        [getattr(hgspy.operators, op) for op in node_ops],
        [getattr(hgspy.operators, op) for op in route_ops],
        [getattr(hgspy.crossover, op) for op in crossover_ops],
        hgspy.stop.MaxIterations(sim_solve_iters),
    )
    base_init = res_init.get_best_found().get_routes()

    # Only do another simulation if there's (on average) enough time for it to
    # complete before the time limit.
    while (sim_start := time.perf_counter()) + avg_duration < start + sim_tlim:
        sim_inst = simulate_instance(
            info,
            obs,
            rng,
            n_lookahead,
            n_requests,
            ep_release=postpone_idcs * 3600,
        )

        singletons = [[r] for r in range(n_ep_reqs, sim_inst["is_depot"].size)]
        sim_init = base_init + singletons

        res = hgs(
            sim_inst,
            hgspy.Config(**sim_config),
            [getattr(hgspy.operators, op) for op in node_ops],
            [getattr(hgspy.operators, op) for op in route_ops],
            [getattr(hgspy.crossover, op) for op in crossover_ops],
            hgspy.stop.MaxIterations(sim_solve_iters),
            initial_solutions=[sim_init],
        )

        best = res.get_best_found()

        for sim_route in best.get_routes():
            # Only dispatch routes that contain must dispatch requests
            if any(idx in must_dispatch for idx in sim_route):
                dispatch_count[sim_route] += 1

        sim_duration = time.perf_counter() - sim_start
        avg_duration = (n_sims * avg_duration + sim_duration) / (n_sims + 1)
        n_sims += 1

        # Postpone requests after ``n_update_threshold`` simulations,
        # and update (lower) the corresponding postponement threshold.
        # if n_sims % n_update_threshold == 0:

    postpone_count = n_sims - dispatch_count
    postpone_threshold = n_sims - dispatch_threshold
    # factor = max(0.90 * n_sims // n_update_threshold, 0.8)
    postpone_idcs = postpone_count >= postpone_threshold  # * factor

    print(n_sims)

    dispatch = (
        ep_inst["is_depot"]
        | ep_inst["must_dispatch"]
        # Only dispatch requests that are dispatched in enough simulations
        | (dispatch_count >= max(1, n_sims) * dispatch_threshold)
    )
    # dispatch = ep_inst["is_depot"] | ep_inst["must_dispatch"] | ~postpone_idcs

    return filter_instance(ep_inst, dispatch)
