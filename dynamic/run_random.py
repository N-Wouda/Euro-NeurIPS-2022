import numpy as np

from .configs import dispatch_config
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

        sol, _ = solve_static(
            dispatch_ep_inst, time_limit=ep_tlim - 1, **dispatch_config
        )
        ep_sol = utils.sol2ep(sol, dispatch_ep_inst)

        n_dispatch = len(dispatch_ep_inst["coords"]) - 1
        n_requests = len(ep_inst["coords"]) - 1
        n_must_dispatch = sum(ep_inst["must_dispatch"])
        n_sol = len([x for route in ep_sol for x in route])

        print(
            f"Dispatch: {n_dispatch} / {n_requests}, {n_must_dispatch=}, {n_sol=}"
        )
        observation, reward, done, info = env.step(ep_sol)
        assert info["error"] is None, f"{info['error']}"

        total_reward += reward

    return total_reward
