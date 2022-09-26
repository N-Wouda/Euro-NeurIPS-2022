import numpy as np
from numpy import concatenate as concat

from .constants import EPOCH_DURATION, EPOCH_N_REQUESTS, SIM_IDX


def simulate_instance(info, obs, rng, n_lookahead=1):
    """
    Simulate a VRPTW instance with n_lookahead epochs.
    - Sample ``EPOCH_N_REQUESTS`` requests per future epoch
    - Filter the customers that cannot be served in a round trip
    - Concatenate the epoch instance and the simulated requests
    """
    # Parameters
    static_inst = info["dynamic_context"]
    ep_inst = obs["epoch_instance"]
    start_time = obs["planning_starttime"]
    duration_matrix = static_inst["duration_matrix"]
    num_customers = len(static_inst["coords"]) - 1  # Exclude depot

    # Simulate not more epochs than that are left
    max_lookahead = min(n_lookahead, info["end_epoch"] - obs["current_epoch"])

    def sample_from_customers(k=max_lookahead * EPOCH_N_REQUESTS):
        return rng.integers(num_customers, size=k) + 1

    cust_idx = sample_from_customers()
    tw_idx = sample_from_customers()
    demand_idx = sample_from_customers()
    service_idx = sample_from_customers()

    # These are unnormalized time windows and release times, which are used to
    # determine request feasibility. Will be clipped later.
    new_tw = static_inst["time_windows"][tw_idx]
    new_epochs = np.repeat(np.arange(1, max_lookahead + 1), EPOCH_N_REQUESTS)
    new_release = start_time + new_epochs * EPOCH_DURATION
    new_demand = static_inst["demands"][demand_idx]
    new_service = static_inst["service_times"][service_idx]

    # Earliest arrival is release time + drive time or earliest time window.
    earliest_arrival = np.maximum(
        new_release + duration_matrix[0, cust_idx],
        new_tw[:, 0],
    )

    earliest_return_at_depot = (
        earliest_arrival + new_service + duration_matrix[cust_idx, 0]
    )

    is_feasible = (earliest_arrival <= new_tw[:, 1]) & (
        earliest_return_at_depot <= static_inst["time_windows"][0, 1]
    )

    # Concatenate the new feasible requests to the epoch instance
    # NOTE SIM_IDX is used to distinguish between known and simulated requests
    new_req_idx = np.arange(is_feasible.sum()) + SIM_IDX
    req_idx = concat((ep_inst["request_idx"], new_req_idx))

    new_cust_idx = cust_idx[is_feasible]
    req_customer_idx = concat((ep_inst["customer_idx"], new_cust_idx))

    # Renormalize TW and release to start_time, and clip the past
    new_tw = np.clip(new_tw - start_time, a_min=0, a_max=None)
    req_tw = concat((ep_inst["time_windows"], new_tw[is_feasible]))

    ep_release = np.zeros(len(ep_inst["coords"]), dtype=int)
    new_release = np.clip(new_release - start_time, a_min=0, a_max=None)
    req_release = concat((ep_release, new_release[is_feasible]))

    req_demand = concat((ep_inst["demands"], new_demand[is_feasible]))
    req_service = concat((ep_inst["service_times"], new_service[is_feasible]))

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
        "release_times": req_release,
    }

    return sim_instance
