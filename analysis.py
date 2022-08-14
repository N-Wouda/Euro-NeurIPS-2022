import argparse
from datetime import datetime
from functools import partial
from glob import glob
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib
import numpy as np
from tqdm.contrib.concurrent import process_map

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
    parser.add_argument("--collect_iters", type=int, default=10)

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--max_runtime", type=int)
    group.add_argument("--max_iterations", type=int)

    return parser.parse_args()


def solve(loc: str, seed: int, **kwargs):
    path = Path(loc)

    hgspy = tools.get_hgspy_module()
    instance = tools.read_vrplib(path)
    start = datetime.now()

    config = hgspy.Config(
        seed=seed,
        nbVeh=-1,
        collectStatistics=True,
        collectNbIter=kwargs["collect_iters"],
    )
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
        len(stats.best_objectives()),
    )


def save_results(res, results_dir, inst_name):
    """
    Save the best solution, statistics and figures of results.
    - Solutions are stored as ``<results_dir>/solutions/<inst_name>.sol``.
    - Statistics are stored as ``<results_dir>/statistics/<inst_name>.csv``.
    - Figures are stored as ``<results_dir>/figures/<inst_name>.png``.
    """
    res_dir = Path(results_dir)

    def make_path(subdir, extension):
        dir_path = res_dir / subdir
        fi_path = dir_path / (inst_name + "." + extension)
        return str(fi_path)

    # Save best solutions
    sol_path = make_path(_SOLS_DIR, "sol")
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
    _, (ax_pop, ax_objs, ax_obj) = plt.subplots(
        nrows=3, ncols=1, figsize=(8, 12)
    )

    iters = stats.curr_iters()

    # Population
    ax_pop.plot(
        iters, stats.feasible_pops(), label="# Feasible", c="tab:orange"
    )

    ax_pop.set_title("Population statistics")
    ax_pop.set_xlabel("Iteration (#)")
    ax_pop.set_ylabel("Individuals (#)")
    ax_pop.legend(frameon=False)

    # Population diversity
    ax_pop_div = ax_pop.twinx()
    ax_pop_div.plot(
        iters, stats.pop_diversity(), label="Diversity", c="tab:red"
    )

    ax_pop_div.set_ylabel("Avg. diversity")
    ax_pop_div.legend(frameon=False)

    # Population objectives
    ax_objs.plot(
        iters, stats.curr_objectives(), label="Current objective", c="tab:blue"
    )
    ax_objs.plot(
        iters,
        stats.feasible_avg_objectives(),
        label="Feasible",
        c="tab:green",
    )
    ax_objs.plot(
        iters,
        stats.infeasible_avg_objectives(),
        label="Infeasible",
        c="tab:red",
    )

    ax_objs.legend(frameon=False)
    # NOTE Use best objectives to set reasonable y-limits
    times, objs = list(zip(*stats.best_objectives()))
    ax_objs.set_ylim(min(objs) * 0.98, min(objs) * 1.05)
    ax_objs.set_ylabel("Objective")

    # Objectives
    times, objs = list(zip(*stats.best_objectives()))
    ax_obj.plot(times, objs)

    ax_obj.set_title("Improving objective values")
    ax_obj.set_xlabel("Run-time (s)")
    ax_obj.set_ylabel("Objective")

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
