import numpy as np
from numpy import concatenate as concat

from .. import utils
from .constants import EPOCH_DURATION, EPOCH_N_REQUESTS, START_IDX


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
