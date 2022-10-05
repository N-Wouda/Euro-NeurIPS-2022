import time

import numpy as np

import tools
from .solve_static import solve_static
from .utils import sol2ep

hgspy = tools.get_hgspy_module()


def solve_dynamic(env, dispatch_strategy, solver_seed):
    """
    Solve the dynamic VRPTW problem using the passed-in dispatching strategy.
    The given seed is used to initialise both the random number stream on the
    Python side, and for the static solver on the C++ side.

    ``dispatch_strategy`` is a function that should take as inputs
    - ``static_info``: static information, including base instance and number
                       of epochs.
    - ``observation``: the observation of the current epoch.
    - ``rng``: a random number generator (same seed as static solver).
    and returns
    - ``dispatch_instance``: the dispatched requests for the current epoch
    """
    rng = np.random.default_rng(solver_seed)

    observation, static_info = env.reset()
    ep_tlim = static_info["epoch_tlim"]

    solutions = {}
    costs = {}
    done = False

    while not done:
        start = time.perf_counter()

        dispatch_inst = dispatch_strategy(static_info, observation, rng)
        solve_tlim = round(ep_tlim - (time.perf_counter() - start))

        # TODO use a seed different from the dynamic rng for the static solver
        config = hgspy.Config(seed=solver_seed)
        stop = hgspy.stop.MaxRuntime(solve_tlim)

        node_ops = [
            hgspy.operators.Exchange10,
            hgspy.operators.Exchange11,
            hgspy.operators.Exchange20,
            hgspy.operators.MoveTwoClientsReversed,
            hgspy.operators.Exchange21,
            hgspy.operators.Exchange22,
            hgspy.operators.TwoOpt,
        ]

        route_ops = [
            hgspy.operators.RelocateStar,
            hgspy.operators.SwapStar,
        ]

        crossover_ops = [
            hgspy.crossover.selective_route_exchange,
        ]

        res = solve_static(
            dispatch_inst, config, node_ops, route_ops, crossover_ops, stop
        )

        best = res.get_best_found()
        routes = [route for route in best.get_routes() if route]

        ep_sol = sol2ep(routes, dispatch_inst)

        current_epoch = observation["current_epoch"]
        solutions[current_epoch] = ep_sol

        observation, reward, done, info = env.step(ep_sol)
        costs[current_epoch] = abs(reward)

        assert info["error"] is None, info["error"]

    return costs, solutions
