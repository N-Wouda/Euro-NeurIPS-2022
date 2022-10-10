import hgspy
import tools


def hgs(
    instance,
    config,
    node_ops,
    route_ops,
    crossover_ops,
    stop,
    initial_solutions=(),
):
    params = hgspy.Params(config, **tools.inst_to_vars(instance))

    rng = hgspy.XorShift128(seed=config.seed)
    pop = hgspy.Population(params, rng)

    for sol in initial_solutions:
        pop.add_individual(hgspy.Individual(params, sol))

    # These operators need to be stored somewhere for the lifetime of the algo,
    # since the cpp side only takes a (non-owning) reference.
    node_ops = [op(params) for op in node_ops]
    route_ops = [op(params) for op in route_ops]

    ls = hgspy.LocalSearch(params, rng)

    for op in node_ops:
        ls.add_node_operator(op)

    for op in route_ops:
        ls.add_route_operator(op)

    algo = hgspy.GeneticAlgorithm(params, rng, pop, ls)

    for op in crossover_ops:
        algo.add_crossover_operator(op)

    return algo.run(stop)
