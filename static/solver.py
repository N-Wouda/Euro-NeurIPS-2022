import tools

hgspy = tools.get_hgspy_module()


def solve(
    instance,
    node_ops,
    route_ops,
    crossover_ops,
    *,
    seed=1,
    max_runtime=None,
    max_iterations=None,
    initial_solutions=(),
    **kwargs,
):
    config = hgspy.Config(seed=seed, **kwargs)
    params = hgspy.Params(config, **tools.inst_to_vars(instance))

    rng = hgspy.XorShift128(seed=seed)
    pop = hgspy.Population(params, rng)

    for sol in initial_solutions:
        hgspy.Individual(params, sol)

    ls = hgspy.LocalSearch(params, rng)

    node_ops = [
        getattr(hgspy.operators, op)(params)
        for op in node_ops
    ]

    for op in node_ops:
        ls.add_node_operator(op)

    route_ops = [
        getattr(hgspy.operators, op)(params)
        for op in route_ops
    ]

    for op in route_ops:
        ls.add_route_operator(op)

    algo = hgspy.GeneticAlgorithm(params, rng, pop, ls)

    crossover_ops = [
        getattr(hgspy.crossover, op)
        for op in crossover_ops
    ]

    for op in crossover_ops:
        algo.add_crossover_operator(op)

    if max_runtime is not None:
        stop = hgspy.stop.MaxRuntime(max_runtime)
    else:
        stop = hgspy.stop.MaxIterations(max_iterations)

    return algo.run(stop)


def get_solutions(res):
    best = res.get_best_found()

    routes = [route for route in best.get_routes() if route]
    cost = best.cost()

    return routes, cost
