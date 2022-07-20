import argparse
from datetime import datetime, timedelta
from functools import partial
from glob import iglob
from multiprocessing import Pool
from pathlib import Path

import numpy as np
from tabulate import tabulate

import tools


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--time_limit", type=int, default=15)
    parser.add_argument("--num_procs", type=int, default=4)

    return parser.parse_args()


def solve(loc: str, seed: int, time_limit: int):
    path = Path(loc)

    print(f" * Solving '{path.stem}' with {seed=} and {time_limit=} seconds.")

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

    # This might fail if insufficient time was given for a feasible solution to
    # be found. In that case, increase the time limit!
    assert np.isclose(tools.validate_static_solution(instance, routes),
                      cost)

    return path.stem, cost, res.get_num_iters()


def main():
    args = parse_args()
    print(f"-- Benchmark run [{args.num_procs} processes] --")

    with Pool(args.num_procs) as pool:
        data = pool.map(
            partial(solve, seed=args.seed, time_limit=args.time_limit),
            iglob("instances/ORTEC-*.txt"))

    data = np.array(data, dtype=[('inst', str), ('obj', float), ('iters', int)])
    table = tabulate(data, ["Instance", "Objective", "Iterations"])

    print('\n', table)
    print("\n-- Statistics --")
    print(" Mean objective:", data['obj'].mean())
    print("Mean iterations:", data['iters'].mean())


if __name__ == "__main__":
    main()
