from datetime import datetime, timedelta

import tools


def solve(instance, seed: int, time_limit: int, **kwargs):
    """
    Solves a given problem instance using the passed-in seed and time limit.
    Any additional keyword arguments are passed directly to the solver.
    """
    hgspy = tools.get_hgspy_module()
    start = datetime.now()

    config = hgspy.Config(seed=seed, **kwargs)
    params = hgspy.Params(config, **tools.inst_to_vars(instance))

    rng = hgspy.XorShift128(seed=seed)
    pop = hgspy.Population(params, rng)
    ls = hgspy.LocalSearch(params, rng)

    algo = hgspy.GeneticAlgorithm(params, rng, pop, ls)
    return algo.run_until(start + timedelta(seconds=time_limit))
