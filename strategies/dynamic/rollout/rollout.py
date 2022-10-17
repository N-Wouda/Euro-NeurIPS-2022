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
    rollout_tlim: float,
    sim_cycle_time: int,
    sim_cycle_iters: int,
    n_lookahead: int,
    n_requests: int,
    dispatch_threshold: float,
    postpone_threshold: float,
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
    must_dispatch = set(np.flatnonzero(ep_inst["must_dispatch"]))
    ep_size = ep_inst["is_depot"].size

    # Statistics
    n_sims = 0
    avg_duration = 0.0
    dispatch_count = np.zeros(ep_size, dtype=int)
    to_postpone = np.zeros(ep_size, dtype=bool)
    to_dispatch = np.zeros(ep_size, dtype=bool)  # unused

    n_cycles = rollout_tlim // sim_cycle_time
    sim_tlim = sim_cycle_time / sim_cycle_iters

    for _ in range(n_cycles):
        # Simulate ``sim_cycle_iters`` instances and count dispatch actions
        for _ in range(sim_cycle_iters):
            sim_inst = simulate_instance(
                info, obs, rng, n_lookahead, n_requests, to_postpone * 3600
            )

            res = hgs(
                sim_inst,
                hgspy.Config(**sim_config),
                [getattr(hgspy.operators, op) for op in node_ops],
                [getattr(hgspy.operators, op) for op in route_ops],
                [getattr(hgspy.crossover, op) for op in crossover_ops],
                hgspy.stop.MaxRuntime(sim_tlim),
            )

            best = res.get_best_found()

            for sim_route in best.get_routes():
                # Only dispatch routes that contain must dispatch requests
                if any(idx in must_dispatch for idx in sim_route):
                    dispatch_count[sim_route] += 1

            dispatch_count[0] += 1  # Depot is also "dispatched"

        # Select requests to postpone based on thresholds
        postpone_count = sim_cycle_iters - dispatch_count
        to_postpone = postpone_count >= postpone_threshold * sim_cycle_iters
        dispatch_count *= 0  # reset dispatch count

    dispatch = ep_inst["is_depot"] | ep_inst["must_dispatch"] | ~to_postpone

    return filter_instance(ep_inst, dispatch)
