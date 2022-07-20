import argparse
from datetime import datetime, timedelta
from functools import partial
from glob import glob
from pathlib import Path

import numpy as np
from tqdm.contrib.concurrent import process_map

import tools


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--time_limit", type=int, default=10)
    parser.add_argument("--num_procs", type=int, default=4)
    parser.add_argument("--instance_pattern",
                        default="instances/ORTEC-VRPTW-ASYM-*.txt")

    return parser.parse_args()


def solve(loc: str, seed: int, time_limit: int):
    path = Path(loc)

    hgspy = tools.get_hgspy_module()
    instance = tools.read_vrplib(path)
    start = datetime.now()

    config = hgspy.Config(seed=seed, nbVeh=-1)

    coords = [(x, y) for x, y in instance['coords'].tolist()]
    demands = instance['demands'].tolist()
    capacity = instance['capacity']
    time_windows = [(l, u) for l, u in instance['time_windows'].tolist()]
    service_times = instance['service_times'].tolist()
    duration_matrix = instance['duration_matrix'].tolist()

    params = hgspy.Params(config, coords, demands, capacity, time_windows,
                          service_times, duration_matrix)

    rng = hgspy.XorShift128(seed=seed)
    ls = hgspy.LocalSearch(params, rng)
    pop = hgspy.Population(params, rng, ls)

    algo = hgspy.GeneticAlgorithm(params, rng, pop)
    res = algo.run_until(start + timedelta(seconds=time_limit))

    best = res.get_best_found()
    routes = [route for route in best.get_routes() if route]
    cost = best.cost()

    try:
        actual_cost = tools.validate_static_solution(instance, routes)
        assert np.isclose(actual_cost, cost), "Could not validate objective."
    except AssertionError:
        has_issue = True
    else:
        has_issue = False

    return path.stem, int(cost), int(res.get_num_iters()), has_issue


def tabulate(headers, rows) -> str:
    # These lengths are used to space each column properly.
    lengths = [len(header) for header in headers]

    for row in rows:
        for idx, (cell, _) in enumerate(zip(row, headers)):
            lengths[idx] = max(lengths[idx], len(str(cell)))

    lines = ["  ".join(f"{h:<{l}s}" for l, h in zip(lengths, headers)),
             "  ".join("-" * l for l in lengths)]

    for row in rows:
        lines.append("  ".join(f"{str(c):>{l}s}" for l, c in zip(lengths, row)))

    return "\n".join(lines)


def main():
    args = parse_args()

    func = partial(solve, seed=args.seed, time_limit=args.time_limit)
    func_args = glob(args.instance_pattern)
    tqdm_kwargs = dict(max_workers=args.num_procs, unit="instance")
    data = process_map(func, func_args, **tqdm_kwargs)

    dtype = [('inst', 'U37'), ('obj', int), ('iters', int), ('issues', bool)]
    data = np.array(data, dtype=dtype)
    table = tabulate(["Inst.", "Obj.", "Iters.", "Issue?"], data)

    print('\n', table, '\n', sep="")
    print(f" Avg. objective: {data['obj'].mean():.0f}")
    print(f"Avg. iterations: {data['iters'].mean():.0f}")
    print(f"   Total issues: {data['issues'].sum()}")


if __name__ == "__main__":
    main()
