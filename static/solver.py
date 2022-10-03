import toml

import tools

hgspy = tools.get_hgspy_module()


def solve(
    instance,
    *,
    seed=1,
    max_runtime=None,
    max_iterations=None,
    initial_solutions=(),
    mode="static",
):
    configuration = toml.load(f"configurations/{mode}.toml")

    config = hgspy.Config(seed=seed, **configuration["params"])
    params = hgspy.Params(config, **tools.inst_to_vars(instance))

    rng = hgspy.XorShift128(seed=seed)
    pop = hgspy.Population(params, rng)

    initial_individuals = [
        hgspy.Individual(params, sol) for sol in initial_solutions
    ]

    for indiv in initial_individuals:
        pop.add_individual(indiv)

    ls = hgspy.LocalSearch(params, rng)

    node_ops = [
        getattr(hgspy.operators, op)(params)
        for op in configuration["operators"]["node"]
    ]

    for op in node_ops:
        ls.add_node_operator(op)

    route_ops = [
        getattr(hgspy.operators, op)(params)
        for op in configuration["operators"]["route"]
    ]

    for op in route_ops:
        ls.add_route_operator(op)

    algo = hgspy.GeneticAlgorithm(params, rng, pop, ls)

    crossover_ops = [
        getattr(hgspy.crossover, op)
        for op in configuration["operators"]["crossover"]
    ]

    for op in crossover_ops:
        algo.add_crossover_operator(op)

    if max_runtime is not None:
        stop = hgspy.stop.MaxRuntime(max_runtime)
    else:
        stop = hgspy.stop.MaxIterations(max_iterations)

    return algo.run(stop)
