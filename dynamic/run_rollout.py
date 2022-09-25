import time

import numpy as np
from numpy import concatenate as concat

from .solve_static import solve_static
import dynamic.utils as utils

EPOCH_DURATION = 3600
EPOCH_N_REQUESTS = 100
START_IDX = 1000  # Dynamic instances have less than 1000 requests total

# STATIC SOLVE CONFIG
SIM_CONFIG = {
    "generationSize": 20,
    "minPopSize": 5,
    "repairProbability": 100,
    "repairBooster": 1000,
    "nbGranular": 80,
    "intensificationProbability": 0,
}


def rollout(ep_inst, static_inst, start_time, sim_tlim, rng):
    """
    Return the dispatch instance.
    """
    n_ep_requests = len(ep_inst["coords"])  # include depot for idx
    dispatch_actions = np.zeros(n_ep_requests, dtype=int)
    dispatch_costs = np.zeros(n_ep_requests, dtype=int)
    postpone_costs = np.zeros(n_ep_requests, dtype=int)

    # REVIEW maybe we need to include something about the route cost
    # alongside the delta cost. the delta cost is probably not a good
    # enough indicator of how expensive a client is. We also need
    # track something about how effective a route is used.

    sim_static_tlim = 250  # TODO make param, milliseconds
    start = time.perf_counter()
    until = (sim_tlim - sim_static_tlim) / 1000

    n_simulations = 0
    while time.perf_counter() - start < until:
        sim_inst = simulate_instance(static_inst, ep_inst, 1, start_time, rng)

        try:
            # Sim_sol is has indices 1, ..., N
            sim_sol, cost = solve_static(
                sim_inst, sim_static_tlim, **SIM_CONFIG
            )

            # req_sol has requests indices
            req_sol = utils.sol2ep(sim_sol, sim_inst, postpone_routes=False)

            for sim_route, req_route in zip(sim_sol, req_sol):
                # Routes with req. idcs >= START_IDX are postponed
                action = 0 if (req_route >= START_IDX).any() else 1

                delta_costs = utils.delta_cost(
                    sim_route, sim_inst["duration_matrix"]
                )

                for idx, request, delta in zip(
                    sim_route, req_route, delta_costs
                ):
                    if request >= START_IDX:
                        continue

                    dispatch_actions[idx] += action

                    if action:
                        dispatch_costs[idx] += delta
                    else:
                        postpone_costs[idx] += delta

            n_simulations += 1
        except AssertionError as e:
            # print(e)
            pass

    mean_dispatch_costs = dispatch_costs / np.maximum(1, dispatch_actions)
    mean_postpone_costs = postpone_costs / np.maximum(
        1, n_simulations - dispatch_actions
    )
    dispatch_fraction = dispatch_actions / np.maximum(1, n_simulations)

    mask = np.full(n_ep_requests, True)

    # Postpone clients that are often postponed and not must-dispatch
    threshold = 0.15  # TODO make parameter
    postpone = (dispatch_fraction <= threshold) & ~ep_inst["must_dispatch"]
    mask = np.where(postpone, False, mask)
    mask[0] = True

    return utils.filter_instance(ep_inst, mask)


def run_rollout(env, **kwargs):
    seed = kwargs["solver_seed"]
    rng = np.random.default_rng(seed)

    observation, static_info = env.reset()
    ep_tlim = static_info["epoch_tlim"]
    static_inst = static_info["dynamic_context"]

    total_reward = 0
    done = False

    while not done:
        ep_inst = observation["epoch_instance"]

        # First and last epoch should be solved completely
        if observation["current_epoch"] in [
            static_info["start_epoch"],
            static_info["end_epoch"],
        ]:
            dispatch_inst = ep_inst
            sol, _ = solve_static(ep_inst, ep_tlim)
            ep_sol = utils.sol2ep(sol, ep_inst)
        else:
            dispatch_inst = rollout(
                ep_inst,
                static_inst,
                observation["planning_starttime"],
                sim_tlim=ep_tlim // 2 - 1,
                rng=rng,
            )

            sol, _ = solve_static(dispatch_inst, ep_tlim // 2)
            ep_sol = utils.sol2ep(sol, dispatch_inst)

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

        # Submit solution to environment
        observation, reward, done, info = env.step(ep_sol)
        assert info["error"] is None, f"{info['error']}"

        total_reward += reward

    return total_reward


def simulate_instance(
    static_instance, ep_instance, n_lookahead, start_time, rng
):
    """
    Simulate a VRPTW instance with n_lookahead epochs.

    - Sample n_lookahead * ... customers
    - Filter the customers that cannot be served within the epoch
    - Concatenate the static instance and the simulated epoch instances
    """
    duration_matrix = static_instance["duration_matrix"]

    # Put the epoch instance at the beginning, with epoch 0.
    req_idx = ep_instance["request_idx"]
    req_customer_idx = ep_instance["customer_idx"]
    req_timewi = ep_instance["time_windows"]
    req_service = ep_instance["service_times"]
    req_demand = ep_instance["demands"]
    req_epoch = np.full(len(req_idx), 0)

    # REVIEW do I need to check that we consider epochs beyond max epoch?
    for epoch in range(1, n_lookahead + 1):
        epoch_start = start_time + epoch * EPOCH_DURATION

        # Sample uniformly
        num_customers = len(static_instance["coords"]) - 1  # Exclude depot

        # Sample data uniformly from customers (1 to num_customers)
        def sample_from_customers(k=EPOCH_N_REQUESTS):
            return rng.integers(num_customers, size=k) + 1

        cust_idx = sample_from_customers()
        timewi_idx = sample_from_customers()
        demand_idx = sample_from_customers()
        service_idx = sample_from_customers()

        new_req_timewi = static_instance["time_windows"][timewi_idx]

        # Filter requests that cannot be served as singleton route in time
        # Time + margin for dispatch + drive time from depot should not exceed latest arrival
        earliest_arrival = np.maximum(
            epoch_start + duration_matrix[0, cust_idx],
            new_req_timewi[:, 0],
        )

        # Also, return at depot in time must be feasible
        earliest_return_at_depot = (
            earliest_arrival
            + static_instance["service_times"][service_idx]
            + duration_matrix[cust_idx, 0]
        )

        is_feasible = (earliest_arrival <= new_req_timewi[:, 1]) & (
            earliest_return_at_depot <= static_instance["time_windows"][0, 1]
        )

        if not is_feasible.any():
            continue

        num_new_requests = is_feasible.sum()
        req_idx = concat(
            (req_idx, np.arange(num_new_requests) + len(req_idx) + START_IDX)
        )
        req_customer_idx = concat((req_customer_idx, cust_idx[is_feasible]))

        # Renormalize time to start_time, and clip time windows in the past
        new_req_timewi = np.clip(
            new_req_timewi - start_time, a_min=0, a_max=None
        )
        req_timewi = concat((req_timewi, new_req_timewi[is_feasible]))
        req_service = concat(
            (
                req_service,
                static_instance["service_times"][service_idx[is_feasible]],
            )
        )
        req_demand = concat(
            (req_demand, static_instance["demands"][demand_idx[is_feasible]])
        )
        req_epoch = concat((req_epoch, np.full(num_new_requests, epoch)))

    sim_instance = {
        "is_depot": static_instance["is_depot"][req_customer_idx],
        "customer_idx": req_customer_idx,
        "request_idx": req_idx,
        "coords": static_instance["coords"][req_customer_idx],
        "demands": req_demand,
        "capacity": static_instance["capacity"],
        "time_windows": req_timewi,
        "service_times": req_service,
        "duration_matrix": static_instance["duration_matrix"][
            np.ix_(req_customer_idx, req_customer_idx)
        ],
        "release_times": req_epoch * EPOCH_DURATION,
    }

    return sim_instance
