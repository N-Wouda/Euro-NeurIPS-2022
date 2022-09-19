from .strategies import STRATEGIES
import numpy as np

import dynamic_tools

from .solve_static import solve_static


def run_baseline(env, **kwargs):
    """
    Solve the dynamic VRPTW problem using baseline strategies, filtering
    requests using a greedy, lazy or random strategy.
    """
    seed = kwargs["solver_seed"]
    rng = np.random.default_rng(seed)

    observation, static_info = env.reset()
    ep_tlim = static_info["epoch_tlim"]
    static_inst = static_info["dynamic_context"]

    total_reward = 0
    done = False

    while not done:
        ep_inst = observation["epoch_instance"]
        dispatch_strategy = STRATEGIES[kwargs["strategy"]]
        dispatch_ep_inst = dispatch_strategy(ep_inst, rng)

        # Return an empty solution if the instance contains no requests
        if dispatch_ep_inst["coords"].shape[0] <= 1:
            ep_sol, cost = [], 0
        else:
            sol, cost = solve_static(
                dispatch_ep_inst,
                time_limit=ep_tlim - 1,  # Margin for grace period
                seed=seed,
            )
            ep_sol = dynamic_tools.idx2request(sol, dispatch_ep_inst, kwargs["postpone_routes"])

        # Submit solution to environment
        observation, reward, done, info = env.step(ep_sol)
        assert cost is None or reward == -cost, f"{info['error']}"

        total_reward += reward

    return total_reward
