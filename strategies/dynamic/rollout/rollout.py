import numpy as np

import hgspy
from strategies.static import hgs
from strategies.utils import filter_instance
from .simulate_instance import simulate_instance


def rollout(
    info,
    obs,
    rng,
    rollout_tlim_factor: float,
    n_simulations: int,
    n_lookahead: int,
    n_requests: int,
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
    # Return the full epoch instance for the last epoch
    if obs["current_epoch"] == info["end_epoch"]:
        return obs["epoch_instance"]

    # Parameters
    ep_inst = obs["epoch_instance"]
    n_ep_reqs = ep_inst["is_depot"].size
    must_dispatch = set(np.flatnonzero(ep_inst["must_dispatch"]))
    rollout_tlim = rollout_tlim_factor * info["epoch_tlim"]
    sim_tlim = rollout_tlim / n_simulations

    dispatch_count = np.zeros(ep_inst["is_depot"].size, dtype=int)

    # Initial solution based on epoch instance
    res_init = hgs(
        ep_inst,
        hgspy.Config(**sim_config),
        [getattr(hgspy.operators, op) for op in node_ops],
        [getattr(hgspy.operators, op) for op in route_ops],
        [getattr(hgspy.crossover, op) for op in crossover_ops],
        hgspy.stop.MaxRuntime(10),
    )
    base_init = res_init.get_best_found().get_routes()

    for _ in range(n_simulations):
        sim_inst = simulate_instance(info, obs, rng, n_lookahead, n_requests)

        # Add each simulated request as single route to obtain full solution
        sim_reqs = [[r] for r in range(n_ep_reqs, sim_inst["is_depot"].size)]
        sim_init = base_init + sim_reqs

        res = hgs(
            sim_inst,
            hgspy.Config(**sim_config),
            [getattr(hgspy.operators, op) for op in node_ops],
            [getattr(hgspy.operators, op) for op in route_ops],
            [getattr(hgspy.crossover, op) for op in crossover_ops],
            hgspy.stop.MaxRuntime(sim_tlim),
            initial_solutions=[sim_init],
        )

        best = res.get_best_found()

        for sim_route in best.get_routes():
            # Only dispatch routes that contain must dispatch requests
            if any(idx in must_dispatch for idx in sim_route):
                dispatch_count[sim_route] += 1

    dispatch = (
        ep_inst["is_depot"]
        | ep_inst["must_dispatch"]
        # Only dispatch requests that are dispatched in enough simulations
        | (dispatch_count >= max(1, n_simulations) * dispatch_threshold)
    )

    return filter_instance(ep_inst, dispatch)
