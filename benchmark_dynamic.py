import argparse
import numpy as np

import tools

from collections import defaultdict
from functools import partial
from glob import glob
from itertools import product
from pathlib import Path
from time import perf_counter
from tqdm.contrib.concurrent import process_map

from dynamic.run_dispatch import run_dispatch
from environment import VRPEnvironment


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--num_seeds", type=int, default=1)
    parser.add_argument("--solver_seed", type=int, default=1)
    parser.add_argument("--num_procs", type=int, default=4)
    parser.add_argument("--strategy", type=str, default="greedy")
    parser.add_argument("--instance_pattern", default="instances/ORTEC-VRPTW-ASYM-*.txt")

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--epoch_tlim", type=int)
    group.add_argument("--phase", choices=["quali", "final"])

    return parser.parse_args()


def solve(loc: str, instance_seed: int, **kwargs):
    path = Path(loc)

    tlim = tools.dynamic_time_limit(kwargs["phase"]) if kwargs.get("phase") is not None else kwargs["epoch_tlim"]

    env = VRPEnvironment(seed=instance_seed, instance=tools.read_vrplib(path), epoch_tlim=tlim)

    start = perf_counter()

    if kwargs["strategy"] == "oracle":
        from dynamic.run_oracle import run_oracle
        costs, routes = run_oracle(env, **kwargs)

    elif kwargs["strategy"] == "dqn":
        from dynamic.dqn.run_dqn import run_dqn
        costs, routes = run_dqn(env, **kwargs)

    else:
        if kwargs["strategy"] in ["greedy", "random", "lazy"]:
            from dynamic.random import random_dispatch
            probs = {"greedy": 1, "random": .5, "lazy": 0}
            strategy = random_dispatch(probs[kwargs["strategy"]])

        elif kwargs["strategy"] == "rollout":
            from dynamic.rollout import rollout as strategy

        else:
            raise NotImplementedError(f"Invalid strategy: {kwargs['strategy']}")

        costs, routes = run_dispatch(env, dispatch_strategy=strategy, **kwargs)

    dtime = round(perf_counter() - start, 3)

    return path.stem, instance_seed, sum(costs.values()), dtime


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
        ("reward", int),
        ("time", float),
    ]

    return np.asarray(averaged, dtype=dtypes)


def main():
    args = parse_args()

    func = partial(solve, **vars(args))
    func_args = product(glob(args.instance_pattern), range(args.num_seeds))

    tqdm_kwargs = dict(max_workers=args.num_procs, unit="instance")
    data = process_map(func, *zip(*func_args), **tqdm_kwargs)
    data = groupby_mean(data)

    headers = [
        "Instance",
        "Avg. costs",
        "Time (s)",
    ]

    table = tools.tabulate(headers, data)
    print(Path(__file__).name, " ".join(f"--{key} {value}" for key, value in vars(args).items()))
    if args.strategy == "rollout":
        from dynamic.rollout import constants
        print(" ".join(f"--{key} {value}" for key, value in vars(constants).items() if not key.startswith("_")))
    print("\n", table, "\n", sep="")
    print(f"      Avg. objective: {data['reward'].mean():.0f}")
    print(f"   Avg. run-time (s): {data['time'].mean():.2f}")


if __name__ == "__main__":
    main()
