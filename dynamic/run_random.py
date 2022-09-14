import numpy as np

from .solve_static import solve_static
import dynamic.utils as utils


def run_random(env, **kwargs):
    """
    Randomly dispatch customers with a given probability.
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

        dispatch_prob = kwargs["dispatch_prob"]
        dispatch_idcs = utils.dispatch_decision(ep_inst, dispatch_prob, rng)
        dispatch_ep_inst = utils.filter_instance(ep_inst, dispatch_idcs)

        # Return an empty solution if the instance contains no requests
        if dispatch_ep_inst["coords"].shape[0] <= 1:
            ep_sol, cost = [], 0
        else:
            sol, cost = solve_static(
                dispatch_ep_inst,
                time_limit=ep_tlim - 1,  # Margin for grace period
                seed=seed,
            )
            ep_sol = utils.sol2ep(sol, dispatch_ep_inst)

        # Submit solution to environment
        observation, reward, done, info = env.step(ep_sol)
        assert cost is None or reward == -cost, f"{info['error']}"

        total_reward += reward

    return total_reward
