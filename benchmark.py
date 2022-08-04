import argparse
from datetime import datetime, timedelta
from functools import partial
from glob import glob
from pathlib import Path

import numpy as np
from tqdm.contrib.concurrent import process_map

import tools
import sisr


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--time_limit", type=int, default=10)
    parser.add_argument("--num_procs", type=int, default=4)
    parser.add_argument(
        "--instance_pattern", default="instances/ORTEC-VRPTW-ASYM-*.txt"
    )

    return parser.parse_args()


def solve(loc: str, seed: int, time_limit: int):
    path = Path(loc)

    hgspy = tools.get_hgspy_module()
    instance = tools.read_vrplib(path)
    start = datetime.now()

    config = hgspy.Config(seed=seed, nbVeh=-1, collectStatistics=True)
    params = hgspy.Params(config, **tools.inst_to_vars(instance))

    rng = hgspy.XorShift128(seed=seed)
    pop = hgspy.Population(params, rng)
    ls = hgspy.LocalSearch(params, rng)
    rnd_state = np.random.default_rng(seed)

    algo = hgspy.GeneticAlgorithm(params, rng, pop, ls)
    algo.add_crossover_operator(hgspy.crossover.ordered_exchange)
    algo.add_crossover_operator(hgspy.crossover.selective_route_exchange)
    algo.add_crossover_operator(hgspy.crossover.string_removal_exchange)

    res = algo.run_until(start + timedelta(seconds=time_limit))

    best = res.get_best_found()
    routes = [route for route in best.get_routes() if route]
    cost = best.cost()

    try:
        actual_cost = tools.validate_static_solution(instance, routes)
        is_ok = "Y"

        assert np.isclose(actual_cost, cost), "Could not validate objective."
    except AssertionError:
        is_ok = "N"

    stats = res.get_statistics()
    kpis = (int(best.cost()), stats.num_iters(), len(stats.best_objectives()))

    return path.stem, is_ok, *kpis


def tabulate(headers, rows) -> str:
    # These lengths are used to space each column properly.
    lengths = [len(header) for header in headers]

    for row in rows:
        for idx, cell in enumerate(row):
            lengths[idx] = max(lengths[idx], len(str(cell)))

    lines = [
        "  ".join(f"{h:<{l}s}" for l, h in zip(lengths, headers)),
        "  ".join("-" * l for l in lengths),
    ]

    for row in rows:
        lines.append(
            "  ".join(f"{str(c):>{l}s}" for l, c in zip(lengths, row))
        )

    return "\n".join(lines)


def main():
    args = parse_args()

    kwargs = dict(seed=args.seed, time_limit=args.time_limit)
    func = partial(solve, **kwargs)
    func_args = sorted(glob(args.instance_pattern))

    tqdm_kwargs = dict(max_workers=args.num_procs, unit="instance")
    data = process_map(func, func_args, **tqdm_kwargs)

    dtypes = [
        ("inst", "U37"),
        ("ok", "U1"),
        ("obj", int),
        ("iters", int),
        ("nb_improv", int),
    ]
    data = np.array(data, dtype=dtypes)

    headers = ["Instance", "OK?", "Objective", "Iters. (#)", "Improv. (#)"]
    table = tabulate(headers, data)

    print("\n", table, "\n", sep="")

    obj_all = data["obj"]
    obj_feas = data[data["ok"] == "Y"]["obj"]

    print(f"      Avg. objective: {obj_all.mean():.0f}", end=" ")
    print(f"(w/o infeas: {obj_feas.mean():.0f})" if obj_feas.size > 0 else "")

    print(f"     Avg. iterations: {data['iters'].mean():.0f}")
    print(f"Avg. improving moves: {data['nb_improv'].mean():.1f}")
    print(f"        Total not OK: {np.count_nonzero(data['ok'] == 'N')}")


if __name__ == "__main__":
    main()
