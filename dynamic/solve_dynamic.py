import time

import numpy as np

import static
import tools
from dynamic import utils


def solve_epoch(instance, time_limit, initial_solutions=(), **kwargs):
    # Return empty solution if the instance contains no clients
    if instance["is_depot"].size <= 1:
        return [], 0

    # Return singleton solution if the instance contains a single client
    if instance["is_depot"].size <= 2:
        solution = [[1]]
        cost = tools.validate_static_solution(instance, solution)
        return solution, cost

    res = static.solve_static(instance, initial_solutions=initial_solutions, max_runtime=time_limit, **kwargs)

    best = res.get_best_found()
    routes = [route for route in best.get_routes() if route]
    cost = best.cost()

    assert best.is_feasible()

    return routes, cost


def solve_dynamic(env, dispatch_strategy, **kwargs):
    """
    Solve the dynamic VRPTW problem using the passed-in dispatching strategy.

    ``dispatch_strategy`` is a function that should take as inputs
    - ``static_info``: static information, including base instance and number
                       of epochs.
    - ``observation``: the observation of the current epoch.
    - ``rng``: a random number generator (same seed as static solver).
    and returns
    - ``dispatch_instance``: the dispatched requests for the current epoch
    """
    rng = np.random.default_rng(kwargs["solver_seed"])

    observation, static_info = env.reset()
    ep_tlim = static_info["epoch_tlim"]

    while not env.is_done:
        start = time.perf_counter()

        dispatch_inst = dispatch_strategy(static_info, observation, rng)
        solve_tlim = round(ep_tlim - (time.perf_counter() - start))

        sol, _ = solve_epoch(dispatch_inst, time_limit=solve_tlim)
        ep_sol = utils.sol2ep(sol, dispatch_inst)

        observation, reward, done, info = env.step(ep_sol)
        assert info["error"] is None, info["error"]

    return env.final_costs, env.final_solutions
