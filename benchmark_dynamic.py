import argparse
from functools import partial
from glob import glob
from environment import VRPEnvironment
from pathlib import Path
from time import perf_counter

import numpy as np
from tqdm.contrib.concurrent import process_map
from itertools import product

import tools
from dynamic import run_baseline


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
    reward = -run_baseline(env, **kwargs)

    return (path.stem, instance_seed, reward, round(perf_counter() - start, 3))


def main():
    args = parse_args()

    func = partial(solve, **vars(args))
    func_args = product(glob(args.instance_pattern), range(args.num_seeds))

    tqdm_kwargs = dict(max_workers=args.num_procs, unit="instance")
    data = process_map(func, *zip(*func_args), **tqdm_kwargs)

    dtypes = [
        ("inst", "U37"),
        ("seed", int),
        ("rew", int),
        ("time", float),
    ]
    data = np.array(data, dtype=dtypes)

    headers = [
        "Instance",
        "Seed",
        "Avg. reward",
        "Time (s)",
    ]
    table = tools.tabulate(headers, data)

    print("\n", table, "\n", sep="")

    obj_all = data["rew"]

    print(f"      Avg. objective: {obj_all.mean():.0f}")
    print(f"   Avg. run-time (s): {data['time'].mean():.2f}")


if __name__ == "__main__":
    main()
