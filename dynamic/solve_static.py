import numpy as np
import tools


def solve_static(instance, time_limit=3600, seed=1):
    hgspy = tools.get_hgspy_module()

    # TODO Determine a better strategy for selecting nbVeh
    config = hgspy.Config(
        seed=seed, nbVeh=tools.n_vehicles_bin_pack(instance, margin=2) + 30
    )
    params = hgspy.Params(config, **tools.inst_to_vars(instance))

    rng = hgspy.XorShift128(seed=seed)
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
        hgspy.crossover.alternating_exchange,
        hgspy.crossover.broken_pairs_exchange,
        hgspy.crossover.ordered_exchange,
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
