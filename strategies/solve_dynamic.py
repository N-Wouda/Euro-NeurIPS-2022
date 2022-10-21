import time

import numpy as np

import hgspy
from strategies.dynamic import STRATEGIES
from strategies.static import hgs
from .utils import sol2ep


def solve_dynamic(env, config, solver_seed):
    """
    Solve the dynamic VRPTW problem using the passed-in dispatching strategy.
    The given seed is used to initialise both the random number stream on the
    Python side, and for the static solver on the C++ side.

    Parameters
    ----------
    env : Environment
    config : Config
        The configuration object, storing the strategy and solver parameters.
    solver_seed : int
        RNG seed. Seed is shared between static and dynamic solver.
    """
    rng = np.random.default_rng(solver_seed)

    observation, static_info = env.reset()
    ep_tlim = static_info["epoch_tlim"]

    solutions = {}
    costs = {}
    done = False

    if static_info["is_static"]:
        config = config.static()
    else:
        config = config.dynamic()

    while not done:
        start = time.perf_counter()

        if static_info["is_static"]:
            dispatch_inst = observation["epoch_instance"]
        else:
            strategy = STRATEGIES[config.strategy()]
            dispatch_inst = strategy(
                static_info, observation, rng, **config.strategy_params()
            )

        solve_tlim = ep_tlim - (time.perf_counter() - start) + 1

        # TODO use a seed different from the dynamic rng for the static solver
        res = hgs(
            dispatch_inst,
            hgspy.Config(seed=solver_seed, **config.solver_params()),
            config.node_ops(),
            config.route_ops(),
            config.crossover_ops(),
            hgspy.stop.MaxRuntime(solve_tlim),
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
