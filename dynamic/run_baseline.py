from .strategies import STRATEGIES
import numpy as np

from .solve_static import solve_static
from .utils import sol2ep


def run_baseline(args, env):
    """
    Solve the dynamic VRPTW problem using baseline strategies, filtering
    requests using a greedy, lazy or random strategy.
    """
    rng = np.random.default_rng(args.solver_seed)

    observation, static_info = env.reset()
    ep_tlim = static_info["epoch_tlim"]
    static_inst = static_info["dynamic_context"]

    total_reward = 0
    done = False

    while not done:
        ep_inst = observation["epoch_instance"]
        dispatch_strategy = STRATEGIES[args.strategy]
        dispatch_ep_inst = dispatch_strategy(ep_inst, rng)

        sol, cost = solve_static(
            dispatch_ep_inst,
            time_limit=ep_tlim - 1,  # Margin for grace period
            seed=args.solver_seed,
        )

        ep_sol = sol2ep(sol, dispatch_ep_inst)

        # Submit solution to environment
        observation, reward, done, info = env.step(ep_sol)
        assert cost is None or reward == -cost, f"{info['error']}"

        total_reward += reward

    return total_reward
