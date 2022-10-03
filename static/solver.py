import os
from pathlib import Path

import toml

import tools

hgspy = tools.get_hgspy_module()

modes_dir = Path("static/modes")
modes = {
    mode.stem: toml.load(modes_dir / mode)
    for mode in map(Path, os.listdir(modes_dir))
}


def solve(
    instance,
    *,
    seed=1,
    max_runtime=None,
    max_iterations=None,
    initial_solutions=(),
    mode="static",
):
    config = hgspy.Config(seed=seed, **modes[mode]["params"])
    params = hgspy.Params(config, **tools.inst_to_vars(instance))

    rng = hgspy.XorShift128(seed=seed)
    pop = hgspy.Population(params, rng)

    for sol in initial_solutions:
        hgspy.Individual(params, sol)

    ls = hgspy.LocalSearch(params, rng)

    node_ops = [
        getattr(hgspy.operators, op)(params)
        for op in modes[mode]["operators"]["node"]
    ]

    for op in node_ops:
        ls.add_node_operator(op)

    route_ops = [
        getattr(hgspy.operators, op)(params)
        for op in modes[mode]["operators"]["route"]
    ]

    for op in route_ops:
        ls.add_route_operator(op)

    algo = hgspy.GeneticAlgorithm(params, rng, pop, ls)

    crossover_ops = [
        getattr(hgspy.crossover, op)
        for op in modes[mode]["operators"]["crossover"]
    ]

    for op in crossover_ops:
        algo.add_crossover_operator(op)

    if max_runtime is not None:
        stop = hgspy.stop.MaxRuntime(max_runtime)
    else:
        stop = hgspy.stop.MaxIterations(max_iterations)

    return algo.run(stop)
