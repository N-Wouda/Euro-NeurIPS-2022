import numpy as np
from environment import State


def run_test(args, env):
    rng = np.random.default_rng(args.solver_seed)

    observation, static_info = env.reset()
    ep_tlim = static_info["epoch_tlim"]
    static_inst = static_info["dynamic_context"]

    total_reward = 0
    done = False

    while not done:
        # Solve for the entire instance first
        ep_inst = observation["epoch_instance"]
        sol, cost = solve(ep_inst, time_limit=1, seed=args.solver_seed)

        # Filter the instance
        inst = ...
        sol, cost = solve(inst, time_limit=1, seed=args.solver_seed)
        ep_sol = sol2ep(sol, inst)

        # Submit solution to environment
        observation, reward, done, info = env.step(ep_sol)
        assert cost is None or reward == -cost, f"{info['error']}"

        total_reward += reward


def _filter_instance(observation: State, mask: np.ndarray) -> dict:
    res = {}

    for key, value in observation.items():
        if key == "capacity":
            res[key] = value
            continue

        if key == "duration_matrix":
            res[key] = value[mask]
            res[key] = res[key][:, mask]
            continue

        res[key] = value[mask]

    return res


def remove_outliers(observation: State, rng: np.random.Generator) -> dict:
    """
    Given an observation, remove all outliers
    """
    return {
        "must_dispatch": np.ones_like(observation["must_dispatch"]).astype(
            np.bool8
        ),
    }


def _greedy(observation: State, rng: np.random.Generator) -> dict:
    return {
        **observation,
        "must_dispatch": np.ones_like(observation["must_dispatch"]).astype(
            np.bool8
        ),
    }


def _lazy(observation: State, rng: np.random.Generator) -> dict:
    mask = np.copy(observation["must_dispatch"])
    mask[0] = True
    return _filter_instance(observation, mask)


def _random(observation: State, rng: np.random.Generator) -> dict:
    mask = np.copy(observation["must_dispatch"])
    mask = mask | rng.binomial(1, p=0.5, size=len(mask)).astype(np.bool8)
    mask[0] = True
    return _filter_instance(observation, mask)


STRATEGIES = dict(greedy=_greedy, lazy=_lazy, random=_random)
