import argparse
from datetime import datetime
from functools import partial
from glob import glob
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib
import numpy as np
from tqdm.contrib.concurrent import process_map

import plotting
import tools

matplotlib.use("Agg")  # Don't show plots

_SOLS_DIR = "solutions/"
_STATS_DIR = "statistics/"
_FIGS_DIR = "figures/"


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
    group.add_argument("--phase", choices=["quali", "final"])

    return parser.parse_args()


def solve(loc: str, seed: int, **kwargs):
    path = Path(loc)

    hgspy = tools.get_hgspy_module()
    instance = tools.read_vrplib(path)
    start = datetime.now()

    config = hgspy.Config(
        seed=seed,
        nbVeh=tools.n_vehicles_bin_pack(instance),
        collectStatistics=True,
    )
    params = hgspy.Params(config, **tools.inst_to_vars(instance))

    rng = hgspy.XorShift128(seed=seed)
    pop = hgspy.Population(params, rng)
    ls = hgspy.LocalSearch(params, rng)

    node_ops = [
        hgspy.operators.Exchange10(params),
        hgspy.operators.Exchange11(params),
        hgspy.operators.Exchange20(params),
        hgspy.operators.MoveTwoClientsReversed(params),
        hgspy.operators.Exchange21(params),
        hgspy.operators.Exchange22(params),
        hgspy.operators.TwoOpt(params),
    ]

    for op in node_ops:
        ls.add_node_operator(op)

    route_ops = [
        hgspy.operators.RelocateStar(params),
        hgspy.operators.SwapStar(params),
    ]

    for op in route_ops:
        ls.add_route_operator(op)

    algo = hgspy.GeneticAlgorithm(params, rng, pop, ls)

    crossover_ops = [
        hgspy.crossover.alternating_exchange,
        hgspy.crossover.broken_pairs_exchange,
        hgspy.crossover.ordered_exchange,
        hgspy.crossover.selective_route_exchange,
    ]

    for op in crossover_ops:
        algo.add_crossover_operator(op)

    if "phase" in kwargs and kwargs["phase"]:
        t_lim = tools.static_time_limit(tools.name2size(loc), kwargs["phase"])
        stop = hgspy.stop.MaxRuntime(t_lim)
    elif "max_runtime" in kwargs and kwargs["max_runtime"]:
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

    # Only save results for runs with feasible solutions and if results_dir
    # is a non-empty string
    if is_ok == "Y" and "results_dir" in kwargs and kwargs["results_dir"]:
        save_results(res, kwargs["results_dir"], path.stem)

    stats = res.get_statistics()
    return (
        path.stem,
        is_ok,
        int(best.cost()),
        res.get_iterations(),
        finish,
        len(stats.incumbents()),
    )


def save_results(res, results_dir, inst_name):
    """
    Save the best solution, statistics and figures of results.
    - Solutions are stored as ``<results_dir>/solutions/<inst_name>.txt``.
    - Statistics are stored as ``<results_dir>/statistics/<inst_name>.csv``.
    - Figures are stored as ``<results_dir>/figures/<inst_name>.png``.
    """
    res_dir = Path(results_dir)

    def make_path(subdir, extension):
        dir_path = res_dir / subdir
        fi_path = dir_path / (inst_name + "." + extension)
        return str(fi_path)

    # Save best solutions
    sol_path = make_path(_SOLS_DIR, "txt")
    best = res.get_best_found()
    stats = res.get_statistics()
    best.export_cvrplib_format(sol_path, sum(stats.run_times()))

    # Save statistics
    stats_path = make_path(_STATS_DIR, "csv")
    stats.to_csv(stats_path, ",")

    # Save plots
    figs_path = make_path(_FIGS_DIR, "png")
    plot_single_run(figs_path, stats)


def plot_single_run(path, stats):
    _, (ax_pop, ax_objs, ax_inc) = plt.subplots(3, 1, figsize=(8, 12))

    plotting.plot_population(stats, ax_pop)
    plotting.plot_objectives(stats, ax_objs)
    plotting.plot_incumbents(stats, ax_inc)

    plt.tight_layout()
    plt.savefig(path)


def main():
    args = parse_args()

    # Make directories to save results
    if args.results_dir is not None:
        res_dir = Path(args.results_dir)
        res_dir.mkdir()
        (res_dir / _SOLS_DIR).mkdir(exist_ok=True)
        (res_dir / _STATS_DIR).mkdir(exist_ok=True)
        (res_dir / _FIGS_DIR).mkdir(exist_ok=True)

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
    table = tools.tabulate(headers, data)

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
