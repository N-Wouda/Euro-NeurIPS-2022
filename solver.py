import argparse
import glob
import importlib.machinery
import importlib.util
import os
import sys
import uuid

import numpy as np

import tools
from baselines.strategies import STRATEGIES
from environment import ControllerEnvironment, VRPEnvironment


def get_hgspy_module(where: str = 'release/lib/hgspy*.so'):
    lib_path = next(glob.iglob(where))
    loader = importlib.machinery.ExtensionFileLoader('hgspy', lib_path)
    spec = importlib.util.spec_from_loader(loader.name, loader)
    hgspy = importlib.util.module_from_spec(spec)
    loader.exec_module(hgspy)


try:
    from hgspy import Genetic, Params, Split, Population, LocalSearch
except ImportError:
    get_hgspy_module()
    from hgspy import Genetic, Params, Split, Population, LocalSearch  # noqa


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--strategy", type=str, default='greedy')
    parser.add_argument("--instance")
    parser.add_argument("--instance_seed", type=int, default=1)
    parser.add_argument("--solver_seed", type=int, default=1)
    parser.add_argument("--static", action='store_true')
    parser.add_argument("--epoch_tlim", type=int, default=120)
    parser.add_argument("--tmp_dir", type=str, default=None)

    return parser.parse_args()


def solve_static_vrptw(instance, time_limit=3600, tmp_dir="tmp", seed=1):
    # Prevent passing empty instances to the static solver, e.g. when
    # strategy decides to not dispatch any requests for the current epoch
    if instance['coords'].shape[0] <= 1:
        yield [], 0
        return

    if instance['coords'].shape[0] <= 2:
        solution = [[1]]
        cost = tools.validate_static_solution(instance, solution)
        yield solution, cost
        return

    os.makedirs(tmp_dir, exist_ok=True)
    instance_filename = os.path.join(tmp_dir, "problem.vrptw")
    tools.write_vrplib(instance_filename, instance, is_vrptw=True)
    out_filename = os.path.join(tmp_dir, "problem.sol")

    # TODO all this works, but it is not pretty. Clean this up in tandem with
    #  the C++ implementation.

    params = Params(
        instance_filename,
        out_filename,
        timeLimit=max(time_limit - 2, 1),
        seed=seed,
        nbVeh=-1,
        useWallClockTime=True
    )

    split = Split(params)
    ls = LocalSearch(params)
    pop = Population(params, split, ls)

    algo = Genetic(params, split, pop, ls)
    res = algo.run(1_000, 60)  # TODO strange parameters

    best = res.get_best_found()
    routes = [route for route in best.get_routes() if route]
    cost = best.cost

    assert np.isclose(tools.validate_static_solution(instance, routes), cost)

    yield routes, cost


def run_oracle(args, env):
    # Oracle strategy which looks ahead, this is NOT a feasible strategy but gives a 'bound' on the performance
    # Bound written with quotes because the solution is not optimal so a better solution may exist
    # This oracle can also be used as supervision for training a model to select which requests to dispatch

    # First get hindsight problem (each request will have a release time)
    done = False
    observation, info = env.reset()
    epoch_tlim = info['epoch_tlim']
    while not done:
        # Dummy solution: 1 route per request
        epoch_solution = [[request_idx] for request_idx in observation['epoch_instance']['request_idx'][1:]]
        observation, reward, done, info = env.step(epoch_solution)
    hindsight_problem = env.get_hindsight_problem()

    oracle_solution = min(solve_static_vrptw(hindsight_problem, time_limit=epoch_tlim, tmp_dir=args.tmp_dir), key=lambda x: x[1])[0]
    oracle_cost = tools.validate_static_solution(hindsight_problem, oracle_solution)

    total_reward = run_baseline(args, env, oracle_solution=oracle_solution)
    assert -total_reward == oracle_cost, "Oracle solution does not match cost according to environment"
    return total_reward


def run_baseline(args, env, oracle_solution=None):
    rng = np.random.default_rng(args.solver_seed)

    total_reward = 0
    done = False
    # Note: info contains additional info that can be used by your solver
    observation, static_info = env.reset()
    epoch_tlim = static_info['epoch_tlim']

    while not done:
        epoch_instance = observation['epoch_instance']

        if oracle_solution is not None:
            request_idx = set(epoch_instance['request_idx'])
            epoch_solution = [route for route in oracle_solution if len(request_idx.intersection(route)) == len(route)]
            cost = tools.validate_dynamic_epoch_solution(epoch_instance, epoch_solution)
        else:
            # Select the requests to dispatch using the strategy
            # TODO improved better strategy (machine learning model?) to decide which non-must requests to dispatch
            epoch_instance_dispatch = STRATEGIES[args.strategy](epoch_instance, rng)

            # Run HGS with time limit and get last solution (= best solution found)
            # Note we use the same solver_seed in each epoch: this is sufficient as for the static problem
            # we will exactly use the solver_seed whereas in the dynamic problem randomness is in the instance
            solutions = list(solve_static_vrptw(epoch_instance_dispatch, time_limit=epoch_tlim, tmp_dir=args.tmp_dir, seed=args.solver_seed))
            assert len(solutions) > 0, f"No solution found during epoch {observation['current_epoch']}"
            epoch_solution, cost = solutions[-1]

            # Map HGS solution to indices of corresponding requests
            epoch_solution = [epoch_instance_dispatch['request_idx'][route] for route in epoch_solution]

        # Submit solution to environment
        observation, reward, done, info = env.step(epoch_solution)
        assert cost is None or reward == -cost, "Reward should be negative cost of solution"
        assert not info['error'], f"Environment error: {info['error']}"

        total_reward += reward

    return total_reward


def main():
    args = parse_args()

    if args.tmp_dir is None:
        # Generate random tmp directory
        args.tmp_dir = os.path.join("tmp", str(uuid.uuid4()))

    if args.instance is not None:
        env = VRPEnvironment(seed=args.instance_seed, instance=tools.read_vrplib(args.instance), epoch_tlim=args.epoch_tlim, is_static=args.static)
    else:
        assert args.strategy != "oracle", "Oracle can not run with external controller"
        # Run within external controller
        env = ControllerEnvironment(sys.stdin, sys.stdout)

    # Make sure these parameters are not used by your solver
    args.instance = None
    args.instance_seed = None
    args.static = None
    args.epoch_tlim = None

    if args.strategy == 'oracle':
        run_oracle(args, env)
    else:
        run_baseline(args, env)


if __name__ == "__main__":
    main()
