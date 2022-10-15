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
    solver_config = {
        "sim_config": sim_config,
        "node_ops": node_ops,
        "route_ops": route_ops,
        "crossover_ops": crossover_ops,
        "sim_solve_iters": sim_solve_iters,
    }

    # Statistics
    n_sims = 0
    avg_duration = 0.0
    dispatch_count = np.zeros(ep_inst["is_depot"].size, dtype=int)

    # Create initial epoch solution
    ep_init = _epoch_initial_solution(ep_inst, **solver_config)

    # Only do another simulation if there's (on average) enough time for it to
    # complete before the time limit.
    while (sim_start := time.perf_counter()) + avg_duration < start + sim_tlim:
        sim_inst = simulate_instance(info, obs, rng, n_lookahead, n_requests)
        sim_only_init = _sim_only_initial_solution(sim_inst, **solver_config)

        res = hgs(
            sim_inst,
            hgspy.Config(**sim_config),
            [getattr(hgspy.operators, op) for op in node_ops],
            [getattr(hgspy.operators, op) for op in route_ops],
            [getattr(hgspy.crossover, op) for op in crossover_ops],
            hgspy.stop.MaxIterations(sim_solve_iters),
            initial_solutions=(ep_init + sim_only_init,),
        )

        best = res.get_best_found()

        for sim_route in best.get_routes():
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
        | (dispatch_count >= max(1, n_sims) * dispatch_threshold)
    )

    return filter_instance(ep_inst, dispatch)


def _epoch_initial_solution(
    ep_inst, sim_config, node_ops, route_ops, crossover_ops, sim_solve_iters
):
    """
    Make the initial epoch solution using the full epoch instance. This
    is the same for all simulation instances.
    """
    res = hgs(
        ep_inst,
        hgspy.Config(**sim_config),
        [getattr(hgspy.operators, op) for op in node_ops],
        [getattr(hgspy.operators, op) for op in route_ops],
        [getattr(hgspy.crossover, op) for op in crossover_ops],
        hgspy.stop.MaxIterations(sim_solve_iters // 2),
    )
    return res.get_best_found().get_routes()


def _sim_only_initial_solution(
    sim_inst,
    sim_config,
    node_ops,
    route_ops,
    crossover_ops,
    sim_solve_iters,
):
    """
    Solve the simulation instance without the epoch instance. Then create a
    complete initial solution for the simulation instance by appending the
    found initial solution and the epoch initial solution.
    """
    sim_idcs = sim_inst["request_idx"] <= 0
    sim_only_inst = filter_instance(sim_inst, sim_idcs)

    res = hgs(
        sim_only_inst,
        hgspy.Config(**sim_config),
        [getattr(hgspy.operators, op) for op in node_ops],
        [getattr(hgspy.operators, op) for op in route_ops],
        [getattr(hgspy.crossover, op) for op in crossover_ops],
        hgspy.stop.MaxIterations(sim_solve_iters // 2),
    )

    # Re-index the sim-only solution to the full simulation instance indices
    n_ep_reqs = sum(sim_inst["request_idx"] >= 0)
    sim_only_init = res.get_best_found().get_routes()
    sim_only_init = [[idx + n_ep_reqs - 1 for idx in r] for r in sim_only_init]

    return sim_only_init
