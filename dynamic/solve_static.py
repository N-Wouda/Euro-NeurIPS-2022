import numpy as np
import tools


def solve_static(
    instance,
    time_limit=None,
    max_iterations=None,
    initial_solutions=None,
    **kwargs
):
    # Return empty solution if the instance contains no clients
    if instance["coords"].shape[0] <= 1:
        return [], 0

    # Return singleton solution if the instance contains a single client
    if instance["coords"].shape[0] <= 2:
        solution = [[1]]
        cost = tools.validate_static_solution(instance, solution)
        return solution, cost

    hgspy = tools.get_hgspy_module()

    # TODO configs should be functions that take instance
    if "nbVeh" not in kwargs:
        kwargs["nbVeh"] = len(instance["coords"])

    config = hgspy.Config(**kwargs)
    params = hgspy.Params(config, **tools.inst_to_vars(instance))

    rng = hgspy.XorShift128(seed=kwargs["seed"])
    pop = hgspy.Population(params, rng)

    if initial_solutions is not None:
        for sol in initial_solutions:
            pop.add_individual(hgspy.Individual(params, sol))

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
        hgspy.crossover.selective_route_exchange,
    ]

    for op in crossover_ops:
        algo.add_crossover_operator(op)

    if time_limit is not None:
        stop = hgspy.stop.MaxRuntime(time_limit)
    else:
        stop = hgspy.stop.MaxIterations(max_iterations)

    res = algo.run(stop)

    best = res.get_best_found()
    routes = [route for route in best.get_routes() if route]
    cost = best.cost()

    try:
        assert np.isclose(
            tools.validate_static_solution(instance, routes), cost
        )
    except:
        breakpoint()

    return routes, cost
