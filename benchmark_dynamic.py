import argparse
from collections import defaultdict
from itertools import product
from functools import partial
from glob import glob
from environment import VRPEnvironment
from pathlib import Path
from time import perf_counter

import numpy as np
from tqdm.contrib.concurrent import process_map

import tools


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--num_seeds", type=int, default=1)
    parser.add_argument("--solver_seed", type=int, default=1)
    parser.add_argument("--num_procs", type=int, default=4)
    parser.add_argument("--strategy", type=str, default="greedy")
    parser.add_argument(
        "--instance_pattern", default="instances/ORTEC-VRPTW-ASYM-*.txt"
    )

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--epoch_tlim", type=int)
    group.add_argument("--phase", choices=["quali", "final"])

    return parser.parse_args()


def solve(loc: str, instance_seed: int, **kwargs):
    path = Path(loc)

    if "phase" in kwargs and kwargs["phase"]:
        tlim = 60 if kwargs["phase"] == "quali" else 120
    else:
        tlim = kwargs["epoch_tlim"]

    env = VRPEnvironment(
        seed=instance_seed, instance=tools.read_vrplib(path), epoch_tlim=tlim
    )

    start = perf_counter()

    if kwargs["strategy"] == "oracle":
        from dynamic.run_oracle import run_oracle

        reward = -run_oracle(env, **kwargs)

    elif kwargs["strategy"] in ["greedy", "random", "lazy"]:
        from dynamic.run_random import run_random

        probs = {"greedy": 100, "random": 50, "lazy": 0}
        reward = -run_random(
            env, **kwargs, dispatch_prob=probs[kwargs["strategy"]]
        )

    elif kwargs["strategy"] == "dqn":
        from dynamic.dqn.run_dqn import run_dqn

        reward = -run_dqn(env, **kwargs)
    elif kwargs["strategy"] == "rollout":
        from dynamic.run_rollout import run_rollout

        reward = -run_rollout(env, **kwargs)

    else:
        raise ValueError(f"Invalid strategy: {kwargs['strategy']}")

    return (path.stem, instance_seed, reward, round(perf_counter() - start, 3))


def groupby_mean(data):
    rewards, runtimes = defaultdict(list), defaultdict(list)

    for (inst, _, reward, runtime) in data:
        rewards[inst].append(reward)
        runtimes[inst].append(runtime)

    averaged = [
        (inst, np.mean(rewards[inst]), np.mean(runtimes[inst]).round(3))
        for inst in rewards.keys()
    ]

    dtypes = [
        ("inst", "U37"),
        ("reward", int),
        ("time", float),
    ]

    return np.array(averaged, dtype=dtypes)


def main():
    args = parse_args()

    func = partial(solve, **vars(args))
    func_args = product(glob(args.instance_pattern), range(args.num_seeds))

    tqdm_kwargs = dict(max_workers=args.num_procs, unit="instance")
    data = process_map(func, *zip(*func_args), **tqdm_kwargs)
    data = groupby_mean(data)

    headers = [
        "Instance",
        "Avg. reward",
        "Time (s)",
    ]
    table = tools.tabulate(headers, data)

    print("\n", table, "\n", sep="")
    print(f"      Avg. objective: {data['reward'].mean():.0f}")
    print(f"   Avg. run-time (s): {data['time'].mean():.2f}")


if __name__ == "__main__":
    main()
