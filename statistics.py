import argparse
from datetime import datetime
from functools import partial
from glob import glob
from pathlib import Path

import matplotlib.pyplot as plt
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

    # Only export results for runs with feasible solutions
    if is_ok == "Y" and "results_dir" in kwargs and kwargs["results_dir"]:
        export_results(res, kwargs["results_dir"], path.stem, start, finish)

    stats = res.get_statistics()
    return (
        path.stem,
        is_ok,
        int(best.cost()),
        res.get_iterations(),
        finish,
        len(stats.best_objectives()),
    )


def export_results(results, results_dir, inst_name, start_time, finish_time):
    """
    Exports the best solution, statistics and figures of results.
    - Solutions are stored as ``<results_dir>/solutions/<inst_name>.sol``.
    - Statistics are stored as ``<results_dir>/statistics/<inst_name>.csv``.
    - Figures are stored as ``<results_dir>/figures/<inst_name>.png``.
    """
    results_dir = Path(results_dir)

    def helper(dir_name, extension):
        dir_path = results_dir / dir_name
        dir_path.mkdir(parents=True, exist_ok=True)
        fi_path = dir_path / (inst_name + "." + extension)
        return fi_path

    # Export best solutions
    sol_path = helper("solutions", "sol")
    best = results.get_best_found()
    best.export_cvrplib_format(str(sol_path), finish_time)

    # Export statistics
    stats_path = helper("statistics", "csv")
    stats = results.get_statistics()
    stats.to_csv(str(stats_path), ",")

    # Save plots
    figs_path = helper("figures", "png")
    plot_single_run(figs_path, stats, start_time)


def plot_single_run(path, stats, start):
    _, (ax_pop, ax_obj) = plt.subplots(nrows=2, ncols=1, figsize=(8, 12))

    # Population
    ax_pop.plot(stats.pop_sizes(), label="Population size", c="tab:blue")
    ax_pop.plot(stats.feasible_pops(), label="# Feasible", c="tab:orange")

    ax_pop.set_title("Population statistics")
    ax_pop.set_xlabel("Iteration (#)")
    ax_pop.set_ylabel("Individuals (#)")
    ax_pop.legend(frameon=False)

    # Population diversity
    ax_pop_div = ax_pop.twinx()
    ax_pop_div.plot(stats.pop_diversity(), label="Diversity", c="tab:red")

    ax_pop_div.set_ylabel("Avg. diversity")
    ax_pop_div.legend(frameon=False)

    # Objectives
    times, objs = list(zip(*stats.best_objectives()))
    ax_obj.plot([(x - start).total_seconds() for x in times], objs)

    ax_obj.set_title("Improving objective values")
    ax_obj.set_xlabel("Run-time (s)")
    ax_obj.set_ylabel("Objective")

    plt.tight_layout()
    plt.savefig(path)


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
