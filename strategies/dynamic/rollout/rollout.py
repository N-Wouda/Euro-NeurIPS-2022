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
        return obs["epoch_instance"], {}

    # Parameters
    ep_inst = obs["epoch_instance"]
    n_ep_reqs = ep_inst["is_depot"].size
    must_dispatch = set(np.flatnonzero(ep_inst["must_dispatch"]))
    rollout_tlim = rollout_tlim_factor * info["epoch_tlim"]
    sim_tlim = rollout_tlim / n_simulations

    dispatch_count = np.zeros(n_ep_reqs, dtype=int)
    to_postpone = np.zeros(n_ep_reqs, dtype=bool)

    stats = {"sim_sols": []}
    dispatch_count = np.zeros(ep_inst["is_depot"].size, dtype=int)

    for _ in range(2):
        ep_inst["release_times"] = to_postpone * 3600

        # Initial solution based on epoch instance
        res_init = hgs(
            ep_inst,
            hgspy.Config(**sim_config),
            [getattr(hgspy.operators, op) for op in node_ops],
            [getattr(hgspy.operators, op) for op in route_ops],
            [getattr(hgspy.crossover, op) for op in crossover_ops],
            hgspy.stop.MaxRuntime(10),
        )
        base_init = [x for x in res_init.get_best_found().get_routes() if x]

        for _ in range(n_simulations):
            sim_inst = simulate_instance(
                info,
                obs,
                rng,
                n_lookahead,
                n_requests,
                ep_release=to_postpone * 3600,
            )

            # Add each simulated request as single route to obtain full solution
            sim_reqs = [
                [r] for r in range(n_ep_reqs, sim_inst["is_depot"].size)
            ]
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
            # stats = res.get_statistics()
            # breakpoint()

            best = res.get_best_found()

            ep_sol = []  # Store the epoch solutions
            for sim_route in best.get_routes():
                # Only dispatch routes that contain must dispatch requests
                if any(idx in must_dispatch for idx in sim_route):
                    dispatch_count[sim_route] += 1
                    ep_sol.append(sim_route)

            stats["sim_sols"].append(ep_sol)

        # Select requests to postpone based on thresholds
        postpone_count = n_simulations - dispatch_count
        postpone_threshold = 1 - dispatch_threshold
        to_postpone = postpone_count >= postpone_threshold * n_simulations
        to_postpone[0] = False  # fix depot
        # breakpoint()
        dispatch_count *= 0  # reset dispatch count

        # stats["dispatch_count"] = dispatch_count

        # breakpoint()

    dispatch = (
        ep_inst["is_depot"]
        | ep_inst["must_dispatch"]
        | ~to_postpone
        # Only dispatch requests that are dispatched in enough simulations
        # | (dispatch_count >= max(1, n_simulations) * dispatch_threshold)
    )

    return filter_instance(ep_inst, dispatch), stats
