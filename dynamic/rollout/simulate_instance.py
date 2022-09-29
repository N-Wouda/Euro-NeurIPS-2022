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
    num_customers = static_inst["is_depot"].size - 1  # Exclude depot

    epochs_left = info["end_epoch"] - obs["current_epoch"]
    max_lookahead = min(n_lookahead, epochs_left)
    n_samples = max_lookahead * EPOCH_N_REQUESTS

    cust_idx = rng.integers(num_customers, size=n_samples) + 1
    tw_idx = rng.integers(num_customers, size=n_samples) + 1
    demand_idx = rng.integers(num_customers, size=n_samples) + 1
    service_idx = rng.integers(num_customers, size=n_samples) + 1

    # These are unnormalized time windows and release times, which are used to
    # determine request feasibility. Will be clipped later.
    sim_tw = static_inst["time_windows"][tw_idx]
    sim_epochs = np.repeat(np.arange(1, max_lookahead + 1), EPOCH_N_REQUESTS)
    sim_release = start_time + sim_epochs * EPOCH_DURATION
    sim_demand = static_inst["demands"][demand_idx]
    sim_service = static_inst["service_times"][service_idx]

    # Earliest arrival is release time + drive time or earliest time window.
    earliest_arrival = np.maximum(
        sim_release + duration_matrix[0, cust_idx],
        sim_tw[:, 0],
    )

    earliest_return_at_depot = (
        earliest_arrival + sim_service + duration_matrix[cust_idx, 0]
    )

    is_feasible = (earliest_arrival <= sim_tw[:, 1]) & (
        earliest_return_at_depot <= static_inst["time_windows"][0, 1]
    )

    # Concatenate the new feasible requests to the epoch instance
    req_customer_idx = concat((ep_inst["customer_idx"], cust_idx[is_feasible]))

    # NOTE Simulated request indices start from SIM_IDX
    sim_req_idx = np.arange(is_feasible.sum()) + SIM_IDX
    req_idx = concat((ep_inst["request_idx"], sim_req_idx))

    # Renormalize TW and release to start_time, and clip the past
    sim_tw = np.clip(sim_tw - start_time, a_min=0, a_max=None)
    req_tw = concat((ep_inst["time_windows"], sim_tw[is_feasible]))

    ep_release = np.zeros(ep_inst["is_depot"].size, dtype=int)
    sim_release = np.clip(sim_release - start_time, a_min=0, a_max=None)
    req_release = concat((ep_release, sim_release[is_feasible]))

    req_demand = concat((ep_inst["demands"], sim_demand[is_feasible]))
    req_service = concat((ep_inst["service_times"], sim_service[is_feasible]))

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
