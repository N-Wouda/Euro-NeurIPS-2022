import argparse
from functools import partial
from glob import glob
from pathlib import Path
from time import perf_counter

import numpy as np
from tqdm.contrib.concurrent import process_map

import hgspy
import tools
from strategies import solve_static


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--num_procs", type=int, default=4)
    parser.add_argument(
        "--instance_pattern", default="instances/ORTEC-VRPTW-ASYM-*.txt"
    )

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--max_runtime", type=int)
    group.add_argument("--max_iterations", type=int)
    group.add_argument("--phase", choices=["quali", "final"])

    return parser.parse_args()


def solve(loc: str, seed: int, **kwargs):
    path = Path(loc)

    instance = tools.read_vrplib(path)
    start = perf_counter()

    config = hgspy.Config(seed=seed)

    node_ops = [
        hgspy.operators.Exchange10,
        hgspy.operators.Exchange11,
        hgspy.operators.Exchange20,
        hgspy.operators.MoveTwoClientsReversed,
        hgspy.operators.Exchange21,
        hgspy.operators.Exchange22,
        hgspy.operators.TwoOpt,
    ]

    route_ops = [
        hgspy.operators.RelocateStar,
        hgspy.operators.SwapStar,
    ]

    crossover_ops = [
        hgspy.crossover.broken_pairs_exchange,
        hgspy.crossover.selective_route_exchange,
    ]

    if kwargs["phase"] is not None:
        t_lim = tools.static_time_limit(tools.name2size(loc), kwargs["phase"])
        stop = hgspy.stop.MaxRuntime(t_lim)
    elif kwargs["max_runtime"] is not None:
        stop = hgspy.stop.MaxRuntime(kwargs["max_runtime"])
    else:
        stop = hgspy.stop.MaxIterations(kwargs["max_iterations"])

    res = solve_static(
        instance, config, node_ops, route_ops, crossover_ops, stop
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
