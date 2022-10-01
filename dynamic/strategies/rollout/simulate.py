import numpy as np

import tools
from . import config

hgspy = tools.get_hgspy_module()


def make_instance(info, obs, rng, n_lookahead=1):
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
    n_samples = max_lookahead * config.EPOCH_N_REQUESTS

    n_customers = static_inst["is_depot"].size - 1  # Exclude depot

    cust_idx = rng.integers(n_customers, size=n_samples) + 1
    tw_idx = rng.integers(n_customers, size=n_samples) + 1
    service_idx = rng.integers(n_customers, size=n_samples) + 1

    # These are static time windows and release times, which are used to
    # determine request feasibility. Will be clipped later to fit the epoch.
    sim_tw = tws[tw_idx]
    sim_epochs = np.repeat(np.arange(1, max_lookahead + 1), config.EPOCH_N_REQUESTS)
    sim_release = start_time + sim_epochs * config.EPOCH_DURATION
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
        return make_instance(info, obs, rng, n_lookahead)

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

    ep_release = np.zeros(ep_inst["is_depot"].size, dtype=int)
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


def solve_instance(instance, max_iterations=None, **kwargs):
    config = hgspy.Config(**kwargs)
    params = hgspy.Params(config, **tools.inst_to_vars(instance))

    rng = hgspy.XorShift128(seed=kwargs["seed"])
    pop = hgspy.Population(params, rng)
    ls = hgspy.LocalSearch(params, rng)

    node_ops = [
        hgspy.operators.Exchange10(params),
        hgspy.operators.Exchange11(params),
        hgspy.operators.Exchange20(params),
        hgspy.operators.MoveTwoClientsReversed(params),
        hgspy.operators.Exchange21(params),
        hgspy.operators.Exchange22(params),
        hgspy.operators.TwoOpt(params),
    ]

    for op in node_ops:
        ls.add_node_operator(op)

    route_ops = [
        hgspy.operators.RelocateStar(params),
        hgspy.operators.SwapStar(params),
    ]

    for op in route_ops:
        ls.add_route_operator(op)

    algo = hgspy.GeneticAlgorithm(params, rng, pop, ls)

    algo.add_crossover_operator(hgspy.crossover.selective_route_exchange)

    stop = hgspy.stop.MaxIterations(max_iterations)
    res = algo.run(stop)

    best = res.get_best_found()
    routes = [route for route in best.get_routes() if route]
    cost = best.cost()
    feasible = best.is_feasible()

    return routes, cost, feasible
