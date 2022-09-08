import argparse
import sys

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

    return parser.parse_args()


def solve_static(instance, time_limit=3600, seed=1):
    # Return an empty solution if the instance contains no requests
    if instance["coords"].shape[0] <= 1:
        return [], 0

    hgspy = tools.get_hgspy_module()

    config = hgspy.Config(seed=seed, nbVeh=tools.n_vehicles_bin_pack(instance))
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

    stop = hgspy.stop.MaxRuntime(time_limit)
    res = algo.run(stop)

    best = res.get_best_found()
    routes = [route for route in best.get_routes() if route]
    cost = best.cost()

    breakpoint()
    assert np.isclose(tools.validate_static_solution(instance, routes), cost)

    return routes, cost


def run_oracle(args, env):
    """
    Solve the dynamic VRPTW problem using the oracle strategy, i.e., the
    problem is solved as static VRPTW with release dates using the information
    that is known in hindsight. The found solution is then fed back into the
    environment.
    """
    observation, info = env.reset()
    epoch_tlim = info["epoch_tlim"]
    done = False

    # Submit dummy solutions to obtain the hindsight problem
    while not done:
        request_idcs = observation["epoch_instance"]["request_idx"][1:]
        ep_sol = [[request] for request in request_idcs]
        observation, _, done, _ = env.step(ep_sol)

    hindsight_problem = env.get_hindsight_problem()
    solution, _ = solve_static(hindsight_problem, time_limit=epoch_tlim)

    observation, _ = env.reset()
    total_reward = 0
    done = False

    # Submit the solution from the hindsight problem
    while not done:
        ep_inst = observation["epoch_instance"]
        request_idcs = set(ep_inst["request_idx"])

        # NOTE This is a proxy to extract the routes from the hindsight solution
        # that are dispatched in the current epoch.
        is_ep_route = lambda r: len(request_idcs.intersection(r)) == len(r)

        ep_sol = [route for route in solution if is_ep_route(route)]
        ep_cost = tools.validate_dynamic_epoch_solution(ep_inst, ep_sol)

        observation, reward, done, info = env.step(ep_sol)
        assert reward == -ep_cost, f"{info['error']}"

        total_reward += reward

    return total_reward


def run_baseline(args, env):
    """
    Solve the dynamic VRPTW problem using baseline strategies, filtering
    requests using a greedy, lazy or random strategy.
    """
    rng = np.random.default_rng(args.solver_seed)

    observation, static_info = env.reset()
    ep_tlim = static_info["epoch_tlim"]
    static_inst = static_info["dynamic_context"]

    total_reward = 0
    done = False

    while not done:
        ep_inst = observation["epoch_instance"]
        dispatch_strategy = STRATEGIES[args.strategy]
        dispatch_ep_inst = dispatch_strategy(ep_inst, rng)

        sol, cost = solve_static(
            dispatch_ep_inst,
            time_limit=ep_tlim - 1,  # Margin for grace period
            seed=args.solver_seed,
        )

        ep_sol = sol2ep(sol, dispatch_ep_inst)

        # Submit solution to environment
        observation, reward, done, info = env.step(ep_sol)
        assert cost is None or reward == -cost, f"{info['error']}"

        total_reward += reward

    return total_reward


def sol2ep(solution, ep_inst):
    """Map solution indices to request indices of the epoch instance."""
    return [ep_inst["request_idx"][route] for route in solution]


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
        ), "Oracle incompatible with external controller"

        # Run within external controller
        env = ControllerEnvironment(sys.stdin, sys.stdout)

    # Make sure these parameters are not used by your solver
    args.instance = None
    args.instance_seed = None
    args.static = None
    args.epoch_tlim = None

    if args.strategy == "oracle":
        print(run_oracle(args, env))
    else:
        print(run_baseline(args, env))


if __name__ == "__main__":
    main()
