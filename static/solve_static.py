import tools
from . import hgspy


def solve_static(instance, *, seed=1, max_runtime=None, max_iterations=None, initial_solutions=(), excl_ops=(), solver_config=None, **kwargs):
    config = hgspy.Config(seed=seed, **solver_config if solver_config is not None else {})
    params = hgspy.Params(config, **tools.inst_to_vars(instance))

    rng = hgspy.XorShift128(seed=seed)
    pop = hgspy.Population(params, rng)

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
        if not any(isinstance(op, excl) for excl in excl_ops):
            ls.add_node_operator(op)

    route_ops = [
        hgspy.operators.RelocateStar(params),
        hgspy.operators.SwapStar(params),
    ]

    for op in route_ops:
        if not any(isinstance(op, excl) for excl in excl_ops):
            ls.add_route_operator(op)

    algo = hgspy.GeneticAlgorithm(params, rng, pop, ls)

    crossover_ops = [
        hgspy.crossover.broken_pairs_exchange,
        hgspy.crossover.selective_route_exchange,
    ]

    for op in crossover_ops:
        if op not in excl_ops:
            algo.add_crossover_operator(op)

    if max_runtime is not None:
        stop = hgspy.stop.MaxRuntime(max_runtime)
    else:
        stop = hgspy.stop.MaxIterations(max_iterations)

    return algo.run(stop)
