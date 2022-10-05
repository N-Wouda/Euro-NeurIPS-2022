import argparse
from collections import defaultdict
from functools import partial
from glob import glob
from itertools import product
from pathlib import Path
from time import perf_counter

import numpy as np
from tqdm.contrib.concurrent import process_map

import tools
from environment import VRPEnvironment
from strategies import solve_dynamic, solve_hindsight
from strategies.dynamic import STRATEGIES


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--num_seeds", type=int, default=1)
    parser.add_argument("--solver_seed", type=int, default=1)
    parser.add_argument("--num_procs", type=int, default=4)
    parser.add_argument(
        "--instance_pattern", default="instances/ORTEC-VRPTW-ASYM-*.txt"
    )

    problem_type = parser.add_mutually_exclusive_group()
    problem_type.add_argument("--hindsight", action="store_true")
    problem_type.add_argument(
        "--strategy", choices=STRATEGIES.keys(), default="rollout"
    )

    stop = parser.add_mutually_exclusive_group(required=True)
    stop.add_argument("--epoch_tlim", type=int)
    stop.add_argument("--phase", choices=["quali", "final"])

    parser.add_argument("--aggregate", action="store_true")

    return parser.parse_args()


def solve(
    loc: str, instance_seed: int, hindsight: bool, strategy: str, **kwargs
):
    path = Path(loc)

    if kwargs["phase"] is not None:
        tlim = tools.dynamic_time_limit(kwargs["phase"])
    else:
        tlim = kwargs["epoch_tlim"]

    env = VRPEnvironment(
        seed=instance_seed, instance=tools.read_vrplib(path), epoch_tlim=tlim
    )

    start = perf_counter()

    seed = kwargs["solver_seed"]

    if hindsight:
        costs, routes = solve_hindsight(env, seed)
    else:
        costs, routes = solve_dynamic(env, STRATEGIES[strategy], seed)

    run_time = round(perf_counter() - start, 3)

    return path.stem, instance_seed, sum(costs.values()), run_time


def groupby_mean(data):
    n_items = defaultdict(int)
    rewards = defaultdict(int)
    runtimes = defaultdict(int)

    for inst, _, reward, runtime in data:
        n_items[inst] += 1
        rewards[inst] += reward
        runtimes[inst] += runtime

    averaged = [
        (inst, round(rewards[inst] / n), round(runtimes[inst] / n, 3))
        for inst, n in n_items.items()
    ]

    dtypes = [
        ("inst", "U37"),
        ("cost", int),
        ("time", float),
    ]

    return np.asarray(averaged, dtype=dtypes)


def main():
    args = parse_args()

    func = partial(solve, **vars(args))
    func_args = product(glob(args.instance_pattern), range(args.num_seeds))

    tqdm_kwargs = dict(max_workers=args.num_procs, unit="instance")
    data = process_map(func, *zip(*func_args), **tqdm_kwargs)
    if args.aggregate:
        headers = [
            "Instance",
            "Avg. cost",
            "Time (s)",
        ]

        data = groupby_mean(data)
    else:
        headers = [
            "Instance",
            "Seed",
            "Cost",
            "Time (s)",
        ]

        dtypes = [
            ("inst", "U37"),
            ("seed", int),
            ("cost", int),
            ("time", float),
        ]
        data = np.asarray(data, dtype=dtypes)

    table = tools.tabulate(headers, data)
    print(
        Path(__file__).name,
        " ".join(f"--{key} {value}" for key, value in vars(args).items()),
    )
    if args.strategy == "rollout":
        from strategies.dynamic.rollout import constants

        print(
            " ".join(
                f"--{key} {value}"
                for key, value in vars(constants).items()
                if not key.startswith("_")
            )
        )
    print("\n", table, "\n", sep="")
    print(f"      Avg. objective: {data['cost'].mean():.0f}")
    print(f"   Avg. run-time (s): {data['time'].mean():.2f}")


if __name__ == "__main__":
    main()
