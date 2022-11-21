import numpy as np

# Fixed value given in the competition rules.
_EPOCH_DURATION = 3600


def simulate_instance(
    info, obs, rng, n_lookahead: int, n_requests: int, ep_release=None
):
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
    dist = static_inst["duration_matrix"]
    tws = static_inst["time_windows"]

    epochs_left = info["end_epoch"] - obs["current_epoch"]
    max_lookahead = min(n_lookahead, epochs_left)
    n_samples = max_lookahead * n_requests

    n_customers = static_inst["is_depot"].size - 1  # Exclude depot

    cust_idx = rng.integers(n_customers, size=n_samples) + 1
    tw_idx = rng.integers(n_customers, size=n_samples) + 1
    service_idx = rng.integers(n_customers, size=n_samples) + 1

    # These are static time windows and release times, which are used to
    # determine request feasibility. Will be clipped later to fit the epoch.
    sim_tw = tws[tw_idx]
    sim_epochs = np.repeat(np.arange(1, max_lookahead + 1), n_requests)
    sim_release = start_time + sim_epochs * _EPOCH_DURATION
    sim_service = static_inst["service_times"][service_idx]

    # Earliest arrival is release time + drive time or earliest time window.
    earliest_arrival = np.maximum(
        sim_release + dist[0, cust_idx], sim_tw[:, 0]
    )
    earliest_return = earliest_arrival + sim_service + dist[cust_idx, 0]
    feas = (earliest_arrival <= sim_tw[:, 1]) & (earliest_return <= tws[0, 1])

    new_custs = cust_idx[feas]
    n_new_customers = len(new_custs)

    if n_new_customers == 0:  # this should not happen a lot
        return simulate_instance(
            info, obs, rng, n_lookahead, n_requests, ep_release
        )

    sim_tw = sim_tw[feas]
    sim_release = sim_release[feas]
    sim_service = sim_service[feas]

    # Concatenate the new feasible requests to the epoch instance
    req_customer_idx = np.concatenate((ep_inst["customer_idx"], new_custs))

    # Simulated request indices are always negative (so we can identify them)
    sim_req_idx = -(np.arange(n_new_customers) + 1)
    req_idx = np.concatenate((ep_inst["request_idx"], sim_req_idx))

    # Normalize TW and release to start_time, and clip the past
    sim_tw = np.maximum(sim_tw - start_time, 0)
    req_tw = np.concatenate((ep_inst["time_windows"], sim_tw))

    ep_release = (
        ep_release
        if ep_release is not None
        else np.zeros_like(ep_inst["is_depot"])
    )
    sim_release = np.maximum(sim_release - start_time, 0)
    req_release = np.concatenate((ep_release, sim_release))

    demand_idx = rng.integers(n_customers, size=n_new_customers) + 1
    sim_demand = static_inst["demands"][demand_idx]

    req_demand = np.concatenate((ep_inst["demands"], sim_demand))
    req_service = np.concatenate((ep_inst["service_times"], sim_service))

    return {
        "is_depot": static_inst["is_depot"][req_customer_idx],
        "customer_idx": req_customer_idx,
        "request_idx": req_idx,
        "coords": static_inst["coords"][req_customer_idx],
        "demands": req_demand,
        "capacity": static_inst["capacity"],
        "time_windows": req_tw,
        "service_times": req_service,
        "duration_matrix": dist[req_customer_idx][:, req_customer_idx],
        "release_times": req_release,
    }
