import numpy as np
import tools


def solve_static(instance, time_limit=60, **kwargs):
    hgspy = tools.get_hgspy_module()

    if "seed" not in kwargs:
        kwargs["seed"] = 1

    if "nbVeh" not in kwargs:
        kwargs["nbVeh"] = tools.n_vehicles_bin_pack(instance) + 30

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

    crossover_ops = [
        hgspy.crossover.broken_pairs_exchange,
        hgspy.crossover.selective_route_exchange,
    ]

    for op in crossover_ops:
        algo.add_crossover_operator(op)

    stop = hgspy.stop.MaxRuntime(time_limit)

    res = algo.run(stop)

    best = res.get_best_found()
    routes = [route for route in best.get_routes() if route]
    cost = best.cost()

    assert np.isclose(tools.validate_static_solution(instance, routes), cost)

    return routes, cost
