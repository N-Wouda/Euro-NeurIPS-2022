import time

import numpy as np

import tools
from strategies.static import hgs
from .utils import sol2ep

hgspy = tools.get_hgspy_module()


def solve_dynamic(env, config, solver_seed, dispatch_strategy):
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
    dispatch_strategy : callable
        The strategy to use for deciding which requests to dispatch in each
        epoch. This function should take the following arguments:

        * static_info: static information, including base instance and number
                       of epochs.
        * observation: the realisations of the current epoch.
        * rng: a seeded random number generator.
        * kwargs: any additional keyword arguments taken from the configuration
          object's strategy parameters.

        Using these arguments, the function should return which requests to
        dispatch in the current epoch.
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
            dispatch_inst = dispatch_strategy(
                static_info, observation, rng, **config.strategy_params()
            )

        solve_tlim = round(ep_tlim - (time.perf_counter() - start))

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
