import argparse
from functools import partial
from glob import glob
from pathlib import Path
from time import perf_counter

import numpy as np
from tqdm.contrib.concurrent import process_map

import hgspy
import tools
from strategies.static import hgs
from strategies.config import Config


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--num_procs", type=int, default=4)
    parser.add_argument("--config_loc", default="configs/benchmark.toml")
    parser.add_argument(
        "--instance_pattern", default="instances/ORTEC-VRPTW-ASYM-*.txt"
    )

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--max_runtime", type=float)
    group.add_argument("--max_iterations", type=int)
    group.add_argument("--phase", choices=["quali", "final"])

    return parser.parse_args()


def solve(
    loc: str,
    seed: int,
    config_loc: str,
    max_runtime,
    max_iterations,
    phase,
    **kwargs,
):
    path = Path(loc)

    instance = tools.read_vrplib(path)
    start = perf_counter()

    if phase is not None:
        t_lim = tools.static_time_limit(tools.name2size(loc), phase)
        stop = hgspy.stop.MaxRuntime(t_lim)
    elif max_runtime is not None:
        stop = hgspy.stop.MaxRuntime(max_runtime)
    else:
        stop = hgspy.stop.MaxIterations(max_iterations)

    static_config = Config.from_file(config_loc).static()

    res = hgs(
        instance,
        hgspy.Config(seed=seed, **static_config.solver_params()),
        static_config.node_ops(),
        static_config.route_ops(),
        static_config.crossover_ops(),
        stop,
    )

    best = res.get_best_found()
    routes = [route for route in best.get_routes() if route]
    cost = best.cost()

    try:
        actual_cost = tools.validate_static_solution(instance, routes)
        is_ok = "Y"

        assert np.isclose(actual_cost, cost), "Could not validate objective."
    except AssertionError:
        is_ok = "N"

    return (
        path.stem,
        is_ok,
        int(cost),
        res.get_iterations(),
        round(perf_counter() - start, 3),
    )


def main():
    args = parse_args()

    func = partial(solve, **vars(args))
    func_args = sorted(glob(args.instance_pattern), key=tools.name2size)

    tqdm_kwargs = dict(max_workers=args.num_procs, unit="instance")
    data = process_map(func, func_args, **tqdm_kwargs)

    dtypes = [
        ("inst", "U37"),
        ("ok", "U1"),
        ("obj", int),
        ("iters", int),
        ("time", float),
    ]

    data = np.asarray(data, dtype=dtypes)

    headers = [
        "Instance",
        "OK",
        "Objective",
        "Iters. (#)",
        "Time (s)",
    ]

    table = tools.tabulate(headers, data)

    print("\n", table, "\n", sep="")

    obj_all = data["obj"]
    obj_feas = data[data["ok"] == "Y"]["obj"]

    print(f"      Avg. objective: {obj_all.mean():.0f}", end=" ")
    print(f"(w/o infeas: {obj_feas.mean():.0f})" if obj_feas.size > 0 else "")

    print(f"     Avg. iterations: {data['iters'].mean():.0f}")
    print(f"   Avg. run-time (s): {data['time'].mean():.2f}")
    print(f"        Total not OK: {np.count_nonzero(data['ok'] == 'N')}")


if __name__ == "__main__":
    main()
