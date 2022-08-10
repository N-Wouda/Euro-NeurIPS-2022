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
    parser.add_argument("--num_procs", type=int, default=4)
    parser.add_argument(
        "--instance_pattern", default="instances/ORTEC-VRPTW-ASYM-*.txt"
    )
    parser.add_argument("--results_dir", type=str)

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--max_runtime", type=int)
    group.add_argument("--max_iterations", type=int)

    return parser.parse_args()


def solve(loc: str, seed: int, **kwargs):
    path = Path(loc)

    hgspy = tools.get_hgspy_module()
    instance = tools.read_vrplib(path)
    start = datetime.now()

    config = hgspy.Config(seed=seed, nbVeh=-1, collectStatistics=True)
    params = hgspy.Params(config, **tools.inst_to_vars(instance))

    rng = hgspy.XorShift128(seed=seed)
    pop = hgspy.Population(params, rng)
    ls = hgspy.LocalSearch(params, rng)

    ls.add_node_operator(hgspy.operators.move_single_client)
    ls.add_node_operator(hgspy.operators.move_two_clients)
    ls.add_node_operator(hgspy.operators.move_two_clients_reversed)
    ls.add_node_operator(hgspy.operators.swap_two_client_pairs)
    ls.add_node_operator(hgspy.operators.swap_two_clients_for_one)
    ls.add_node_operator(hgspy.operators.swap_two_single_clients)
    ls.add_node_operator(hgspy.operators.two_opt_between_routes)
    ls.add_node_operator(hgspy.operators.two_opt_within_route)

    ls.add_route_operator(hgspy.operators.relocate_star)
    ls.add_route_operator(hgspy.operators.swap_star)

    algo = hgspy.GeneticAlgorithm(params, rng, pop, ls)
    algo.add_crossover_operator(hgspy.crossover.ordered_exchange)
    algo.add_crossover_operator(hgspy.crossover.selective_route_exchange)

    if "max_runtime" in kwargs and kwargs["max_runtime"]:
        stop = hgspy.stop.MaxRuntime(kwargs["max_runtime"])
    else:
        stop = hgspy.stop.MaxIterations(kwargs["max_iterations"])

    res = algo.run(stop)
    finish = round((datetime.now() - start).total_seconds(), 3)

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

    if "results_dir" in kwargs and kwargs["results_dir"]:
        export_results(res, kwargs["results_dir"], path.stem, finish)

    return (
        path.stem,
        is_ok,
        int(best.cost()),
        stats.num_iters(),
        finish,
        len(stats.best_objectives()),
    )


def export_results(results, results_dir, inst_name, finish_time):
    """
    Export the results, i.e., the best solution and statistics.
    The solutions are stored as ``<results_dir>/solutions/<inst_name>.sol``.
    The statistics are stored as ``<results_dir>/statistics/<inst_name>.csv``.
    """
    results_dir = Path(results_dir)

    # Export best solutions
    sol_dir = results_dir / "solutions/"
    sol_dir.mkdir(parents=True, exist_ok=True)
    sol_path = sol_dir / (inst_name + ".sol")

    best = results.get_best_found()
    best.export_cvrplib_format(str(sol_path), finish_time)

    # Export statistics
    stats_dir = results_dir / "statistics/"
    stats_dir.mkdir(parents=True, exist_ok=True)
    stats_path = stats_dir / (inst_name + ".csv")

    stats = results.get_statistics()
    stats.export_csv(str(stats_path))


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

    func = partial(solve, **vars(args))
    func_args = sorted(glob(args.instance_pattern))

    tqdm_kwargs = dict(max_workers=args.num_procs, unit="instance")
    data = process_map(func, func_args, **tqdm_kwargs)

    dtypes = [
        ("inst", "U37"),
        ("ok", "U1"),
        ("obj", int),
        ("iters", int),
        ("time", float),
        ("nb_improv", int),
    ]
    data = np.array(data, dtype=dtypes)

    headers = [
        "Instance",
        "OK",
        "Objective",
        "Iters. (#)",
        "Time (s)",
        "Improv. (#)",
    ]
    table = tabulate(headers, data)

    print("\n", table, "\n", sep="")

    obj_all = data["obj"]
    obj_feas = data[data["ok"] == "Y"]["obj"]

    print(f"      Avg. objective: {obj_all.mean():.0f}", end=" ")
    print(f"(w/o infeas: {obj_feas.mean():.0f})" if obj_feas.size > 0 else "")

    print(f"     Avg. iterations: {data['iters'].mean():.0f}")
    print(f"   Avg. run-time (s): {data['time'].mean():.2f}")
    print(f"Avg. improving moves: {data['nb_improv'].mean():.1f}")
    print(f"        Total not OK: {np.count_nonzero(data['ok'] == 'N')}")


if __name__ == "__main__":
    main()
