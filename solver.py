import argparse
import sys
from datetime import datetime, timedelta

import matplotlib.pyplot as plt
import numpy as np

import tools
from baselines.strategies import STRATEGIES
from environment import ControllerEnvironment, VRPEnvironment


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--strategy", type=str, default="greedy")
    parser.add_argument("--instance")
    parser.add_argument("--instance_seed", type=int, default=1)
    parser.add_argument("--solver_seed", type=int, default=1)
    parser.add_argument("--static", action="store_true")
    parser.add_argument("--epoch_tlim", type=int, default=120)
    parser.add_argument("--plot_statistics", action="store_true")

    return parser.parse_args()


def plot_single_run(stats, start):
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
    plt.savefig(f"tmp/{datetime.now().isoformat()}.png")


def solve_static_vrptw(instance, time_limit=3600, seed=1, plot=False):
    # Instance is a dict that has the following entries:
    # - 'is_depot': boolean np.array. True for depot; False otherwise.
    # - 'coords': np.array of locations (incl. depot)
    # - 'demands': np.array of location demands (incl. depot with demand zero)
    # - 'capacity': int of vehicle capacity
    # - 'time_windows': np.array of [l, u] time windows per client (incl. depot)
    # - 'service_times': np.array of service times at each client (incl. depot)
    # - 'duration_matrix': distance matrix between clients (incl. depot)
    start = datetime.now()

    # Prevent passing empty instances to the static solver, e.g. when
    # strategy decides to not dispatch any requests for the current epoch
    if instance["coords"].shape[0] <= 1:
        yield [], 0
        return

    if instance["coords"].shape[0] <= 2:
        solution = [[1]]
        cost = tools.validate_static_solution(instance, solution)
        yield solution, cost
        return

    hgspy = tools.get_hgspy_module()

    # Need data to plot, so use plot here to control data collection
    config = hgspy.Config(seed=seed, nbVeh=-1, collectStatistics=plot)
    params = hgspy.Params(config, **tools.inst_to_vars(instance))

    rng = hgspy.XorShift128(seed=seed)
    pop = hgspy.Population(params, rng)

    ls = hgspy.LocalSearch(params, rng)

    node_ops = [
        hgspy.operators.MoveSingleClient(),
        hgspy.operators.MoveTwoClients(),
        hgspy.operators.MoveTwoClientsReversed(),
        hgspy.operators.SwapTwoClientPairs(),
        hgspy.operators.SwapTwoClientsForOne(),
        hgspy.operators.SwapTwoSingleClients(),
        hgspy.operators.TwoOpt(),
    ]

    for op in node_ops:
        ls.add_node_operator(op)

    route_ops = [
        hgspy.operators.RelocateStar(),
        hgspy.operators.SwapStar(),
    ]

    for op in route_ops:
        ls.add_route_operator(op)

    algo = hgspy.GeneticAlgorithm(params, rng, pop, ls)
    algo.add_crossover_operator(hgspy.crossover.ordered_exchange)
    algo.add_crossover_operator(hgspy.crossover.selective_route_exchange)

    stop = hgspy.stop.MaxRuntime(time_limit)
    res = algo.run(stop)

    best = res.get_best_found()
    routes = [route for route in best.get_routes() if route]
    cost = best.cost()

    if plot:
        plot_single_run(res.get_statistics(), start)

    assert np.isclose(tools.validate_static_solution(instance, routes), cost)

    yield routes, cost


def run_oracle(args, env):
    # Oracle strategy which looks ahead, this is NOT a feasible strategy but gives a 'bound' on the performance
    # Bound written with quotes because the solution is not optimal so a better solution may exist
    # This oracle can also be used as supervision for training a model to select which requests to dispatch

    # First get hindsight problem (each request will have a release time)
    done = False
    observation, info = env.reset()
    epoch_tlim = info["epoch_tlim"]
    while not done:
        # Dummy solution: 1 route per request
        epoch_solution = [
            [request_idx]
            for request_idx in observation["epoch_instance"]["request_idx"][1:]
        ]
        observation, reward, done, info = env.step(epoch_solution)
    hindsight_problem = env.get_hindsight_problem()

    oracle_solution = min(
        solve_static_vrptw(hindsight_problem, time_limit=epoch_tlim),
        key=lambda x: x[1],
    )[0]
    oracle_cost = tools.validate_static_solution(
        hindsight_problem, oracle_solution
    )

    total_reward = run_baseline(args, env, oracle_solution=oracle_solution)
    assert (
        -total_reward == oracle_cost
    ), "Oracle solution does not match cost according to environment"
    return total_reward


def run_baseline(args, env, oracle_solution=None):
    rng = np.random.default_rng(args.solver_seed)

    total_reward = 0
    done = False
    # Note: info contains additional info that can be used by your solver
    observation, static_info = env.reset()
    epoch_tlim = static_info["epoch_tlim"]

    while not done:
        epoch_instance = observation["epoch_instance"]

        if oracle_solution is not None:
            request_idx = set(epoch_instance["request_idx"])
            epoch_solution = [
                route
                for route in oracle_solution
                if len(request_idx.intersection(route)) == len(route)
            ]
            cost = tools.validate_dynamic_epoch_solution(
                epoch_instance, epoch_solution
            )
        else:
            # Select the requests to dispatch using the strategy
            # TODO improved better strategy (machine learning model?) to decide which non-must requests to dispatch
            epoch_instance_dispatch = STRATEGIES[args.strategy](
                epoch_instance, rng
            )

            # Run HGS with time limit and get last solution (= best solution found)
            # Note we use the same solver_seed in each epoch: this is sufficient as for the static problem
            # we will exactly use the solver_seed whereas in the dynamic problem randomness is in the instance
            solutions = list(
                solve_static_vrptw(
                    epoch_instance_dispatch,
                    time_limit=epoch_tlim,
                    seed=args.solver_seed,
                    plot=args.plot_statistics,
                )
            )
            assert (
                len(solutions) > 0
            ), f"No solution found during epoch {observation['current_epoch']}"
            epoch_solution, cost = solutions[-1]

            # Map HGS solution to indices of corresponding requests
            epoch_solution = [
                epoch_instance_dispatch["request_idx"][route]
                for route in epoch_solution
            ]

        # Submit solution to environment
        observation, reward, done, info = env.step(epoch_solution)
        assert (
            cost is None or reward == -cost
        ), "Reward should be negative cost of solution"
        assert not info["error"], f"Environment error: {info['error']}"

        total_reward += reward

    return total_reward


def main():
    args = parse_args()

    if args.instance is not None:
        env = VRPEnvironment(
            seed=args.instance_seed,
            instance=tools.read_vrplib(args.instance),
            epoch_tlim=args.epoch_tlim,
            is_static=args.static,
        )
    else:
        assert (
            args.strategy != "oracle"
        ), "Oracle can not run with external controller"
        # Run within external controller
        env = ControllerEnvironment(sys.stdin, sys.stdout)

    # Make sure these parameters are not used by your solver
    args.instance = None
    args.instance_seed = None
    args.static = None
    args.epoch_tlim = None

    if args.strategy == "oracle":
        run_oracle(args, env)
    else:
        run_baseline(args, env)


if __name__ == "__main__":
    main()
