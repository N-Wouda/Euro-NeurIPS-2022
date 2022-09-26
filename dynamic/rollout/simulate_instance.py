import numpy as np
from numpy import concatenate as concat

from .constants import EPOCH_DURATION, EPOCH_N_REQUESTS, START_IDX


def simulate_instance(info, obs, rng, n_lookahead=1):
    """
    Simulate a VRPTW instance with n_lookahead epochs.
    - Sample ``EPOCH_N_REQUESTS`` for ``n_lookahead`` future epochs
    - Filter the customers that cannot be served within the corresponding epoch
    - Concatenate the static instance and the simulated epoch instances
    """
    # Parameters
    static_inst = info["dynamic_context"]
    ep_instance = obs["epoch_instance"]
    start_time = obs["planning_starttime"]
    duration_matrix = static_inst["duration_matrix"]
    num_customers = len(static_inst["coords"]) - 1  # Exclude depot

    # Put the epoch instance at the beginning, with epoch 0.
    req_idx = ep_instance["request_idx"]
    req_customer_idx = ep_instance["customer_idx"]
    req_tw = ep_instance["time_windows"]
    req_service = ep_instance["service_times"]
    req_demand = ep_instance["demands"]
    req_epoch = np.full(len(req_idx), 0)

    for epoch in range(1, n_lookahead + 1):
        # Release time w.r.t. static instance
        release_time = start_time + epoch * EPOCH_DURATION

        # Sample data uniformly from customers (1 to num_customers)
        def sample_from_customers(k=EPOCH_N_REQUESTS):
            return rng.integers(num_customers, size=k) + 1

        cust_idx = sample_from_customers()
        tw_idx = sample_from_customers()
        demand_idx = sample_from_customers()
        service_idx = sample_from_customers()

        new_req_tw = static_inst["time_windows"][tw_idx]

        # Filter requests that cannot be served as roundtrip in time
        # Earliest arrival is release time + drive time or earliest time window.
        earliest_arrival = np.maximum(
            release_time + duration_matrix[0, cust_idx],
            new_req_tw[:, 0],
        )

        earliest_return_at_depot = (
            earliest_arrival
            + static_inst["service_times"][service_idx]
            + duration_matrix[cust_idx, 0]
        )

        is_feasible = (earliest_arrival <= new_req_tw[:, 1]) & (
            earliest_return_at_depot <= static_inst["time_windows"][0, 1]
        )

        if not is_feasible.any():
            continue

        num_new_requests = is_feasible.sum()

        # Concatenate the new feasible requests to the current instance
        # NOTE START_IDX is used to distinguish between known and simulated requests
        new_req_idx = np.arange(num_new_requests) + len(req_idx) + START_IDX
        req_idx = concat((req_idx, new_req_idx))

        new_cust_idx = cust_idx[is_feasible]
        req_customer_idx = concat((req_customer_idx, new_cust_idx))

        # Renormalize TW to start_time, and clip TW in the past
        new_req_tw = np.clip(new_req_tw - start_time, a_min=0, a_max=None)
        req_tw = concat((req_tw, new_req_tw[is_feasible]))

        new_service = static_inst["service_times"][service_idx[is_feasible]]
        req_service = concat((req_service, new_service))

        new_demand = static_inst["demands"][demand_idx[is_feasible]]
        req_demand = concat((req_demand, new_demand))

        req_epoch = concat((req_epoch, np.full(num_new_requests, epoch)))

    sim_instance = {
        "is_depot": static_inst["is_depot"][req_customer_idx],
        "customer_idx": req_customer_idx,
        "request_idx": req_idx,
        "coords": static_inst["coords"][req_customer_idx],
        "demands": req_demand,
        "capacity": static_inst["capacity"],
        "time_windows": req_tw,
        "service_times": req_service,
        "duration_matrix": static_inst["duration_matrix"][
            np.ix_(req_customer_idx, req_customer_idx)
        ],
        "release_times": req_epoch * EPOCH_DURATION,
    }

    return sim_instance
