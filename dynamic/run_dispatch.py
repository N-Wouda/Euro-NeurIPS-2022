import time

import numpy as np

from .configs import dispatch_config
from .solve_static import solve_static
import dynamic.utils as utils


def run_dispatch(env, dispatch_strategy, **kwargs):
    """
    Solve the dynamic VRPTW problem using the passed-in dispatching strategy.

    ``strategy`` is a function that should take as inputs
    - ``static_info``: static information, including base instance and number of epochs.
    - ``observation``: the observation of the current epoch.
    - ``rng``: a random number generator (same seed as static solver).
    and returns
    - ``dispatch_instance``: the dispatched requests for the current epoch
    """
    rng = np.random.default_rng(kwargs["solver_seed"])

    observation, static_info = env.reset()
    ep_tlim = static_info["epoch_tlim"]
    total_reward = 0
    done = False

    while not done:
        start = time.perf_counter()

        dispatch_inst = dispatch_strategy(static_info, observation, rng)
        solve_tlim = ep_tlim - (time.perf_counter() - start)

        sol, _ = solve_static(
            dispatch_inst, time_limit=int(solve_tlim) * 1000, **dispatch_config
        )
        ep_sol = utils.sol2ep(sol, dispatch_inst)

        ep_inst = observation["epoch_instance"]
        n_dispatch = len(dispatch_inst["coords"]) - 1
        n_requests = len(ep_inst["coords"]) - 1
        n_must_dispatch = sum(ep_inst["must_dispatch"])
        n_sol = len([x for route in ep_sol for x in route])
        print(
            f"Epoch: {observation['current_epoch']} / {static_info['end_epoch']}",
            end=" - ",
        )
        print(
            f"Dispatch: {n_dispatch} / {n_requests}, {n_must_dispatch=}, {n_sol=}"
        )

        observation, reward, done, info = env.step(ep_sol)
        assert info["error"] is None, f"{info['error']}"

        total_reward += reward

    return total_reward
