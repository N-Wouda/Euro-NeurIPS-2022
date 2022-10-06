import argparse
from functools import partial
from glob import glob
from pathlib import Path
from time import perf_counter

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from tqdm.contrib.concurrent import process_map

import plotting
import tools


hgspy = tools.get_hgspy_module()

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
    parser.add_argument("--overwrite", action="store_true")

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--max_runtime", type=int)
    group.add_argument("--max_iterations", type=int)
    group.add_argument("--phase", choices=["quali", "final"])

    return parser.parse_args()


def solve(loc: str, seed: int, **kwargs):
    path = Path(loc)

    instance = tools.read_vrplib(path)
    start = perf_counter()

    config = hgspy.Config(seed=seed, collectStatistics=True)

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
        hgspy.crossover.broken_pairs_exchange,
        hgspy.crossover.selective_route_exchange,
    ]

    for op in crossover_ops:
        algo.add_crossover_operator(op)

    mutation_ops = [
        hgspy.mutation.string_removals,
    ]

    for op in mutation_ops:
        algo.add_mutation_operator(op)

    if kwargs["phase"] is not None:
        t_lim = tools.static_time_limit(tools.name2size(loc), kwargs["phase"])
        stop = hgspy.stop.MaxRuntime(t_lim)
    elif kwargs["max_runtime"] is not None:
        stop = hgspy.stop.MaxRuntime(kwargs["max_runtime"])
    else:
        stop = hgspy.stop.MaxIterations(kwargs["max_iterations"])

    res = algo.run(stop)

    best = res.get_best_found()
    routes = [route for route in best.get_routes() if route]
    cost = best.cost()

    try:
        actual_cost = tools.validate_static_solution(instance, routes)
        is_ok = "Y"

        assert np.isclose(actual_cost, cost), "Could not validate objective."
    except AssertionError:
        is_ok = "N"

    finish = round(perf_counter() - start, 3)

    # Only save results for runs with feasible solutions and if results_dir
    # is a non-empty string
    if is_ok == "Y" and kwargs["results_dir"] is not None:
        save_results(instance, res, kwargs["results_dir"], path.stem)

    stats = res.get_statistics()
    return (
        path.stem,
        is_ok,
        int(cost),
        res.get_iterations(),
        finish,
        len(stats.incumbents()),
    )


def save_results(instance, res, results_dir, inst_name):
    """
    Save the best solution, statistics and figures of results.
    - Solutions are stored as ``<results_dir>/solutions/<inst_name>.txt``.
    - Statistics are stored as ``<results_dir>/statistics/<inst_name>.csv``.
    - Figures are stored as ``<results_dir>/figures/<inst_name>.png``.
    """
    res_path = Path(results_dir)

    def make_path(subdir, extension):
        return (res_path / subdir / inst_name).with_suffix("." + extension)

    # Save best solutions
    best = res.get_best_found()
    sol_path = str(make_path(_SOLS_DIR, "txt"))
    best.export_cvrplib_format(sol_path, res.get_run_time())

    # Save statistics
    stats = res.get_statistics()
    stats_path = str(make_path(_STATS_DIR, "csv"))
    stats.to_csv(stats_path)

    # Save plots
    fig = plt.figure(figsize=(20, 12))
    gs = fig.add_gridspec(3, 2, width_ratios=(2 / 5, 3 / 5))

    plotting.plot_population(fig.add_subplot(gs[0, 0]), stats)
    plotting.plot_objectives(fig.add_subplot(gs[1, 0]), stats)
    plotting.plot_incumbents(fig.add_subplot(gs[2, 0]), stats)

    routes = best.get_routes()
    plotting.plot_instance(fig.add_subplot(gs[:, 1]), instance, routes)

    plt.tight_layout()
    plt.savefig(make_path(_FIGS_DIR, "png"))
    plt.close(fig)


def main():
    args = parse_args()

    # Make directories to save results
    if args.results_dir is not None:
        res_dir = Path(args.results_dir)
        res_dir.mkdir(exist_ok=args.overwrite)
        (res_dir / _SOLS_DIR).mkdir(exist_ok=args.overwrite)
        (res_dir / _STATS_DIR).mkdir(exist_ok=args.overwrite)
        (res_dir / _FIGS_DIR).mkdir(exist_ok=args.overwrite)

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
