import argparse
from functools import partial

import numpy as np
from skopt import gp_minimize
from skopt.space import Integer
from skopt.utils import use_named_args

import hgspy
import tools
from strategies.config import Config
from strategies.static import hgs


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--verbose", action="store_true")

    return parser.parse_args()


def evaluate(defaults: Config, **params):
    params = defaults.solver_params() | params
    params["seed"] = np.random.randint(100)  # TODO find way to generate noise

    config = hgspy.Config(**params)

    # TODO generalise to training subset of all static instances
    loc = "instances/ORTEC-VRPTW-ASYM-9016f313-d1-n200-k20.txt"
    run_time = tools.static_time_limit(tools.name2size(loc), "quali")
    stop = hgspy.stop.MaxRuntime(run_time)

    res = hgs(
        tools.read_vrplib(loc),
        config,
        defaults.node_ops(),
        defaults.route_ops(),
        defaults.crossover_ops(),
        stop
    )

    return res.get_best_found().cost()


def main():
    args = parse_args()

    # Population management search space
    space = [
        Integer(5, 100, name="minPopSize"),
        Integer(1, 100, name="generationSize"),
        Integer(0, 25, name="nbElite"),
        Integer(1, 25, name="nbClose"),
    ]

    defaults = Config.from_file("configs/solver.toml").static()
    func = use_named_args(space)(partial(evaluate, defaults=defaults))
    res = gp_minimize(func,
                      space,
                      n_calls=50,
                      random_state=args.seed,
                      verbose=args.verbose)

    print(res.fun, res.x)


if __name__ == "__main__":
    main()
