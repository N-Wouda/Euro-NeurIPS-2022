import argparse
import sys

import tools
from environment import ControllerEnvironment, VRPEnvironment

from dynamic import run_random, run_oracle, run_supervised, run_dqn


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--strategy", type=str, default="greedy")
    parser.add_argument("--instance")
    parser.add_argument("--instance_seed", type=int, default=1)
    parser.add_argument("--solver_seed", type=int, default=1)
    parser.add_argument("--static", action="store_true")
    parser.add_argument("--epoch_tlim", type=int, default=120)

    return parser.parse_args()


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
        run_oracle(env, **vars(args))
    elif args.strategy in ["greedy", "random", "lazy"]:
        probs = {"greedy": 100, "random": 50, "lazy": 0}
        run_random(env, **vars(args), dispatch_prob=probs[args.strategy])
    elif args.strategy == "supervised":
        run_supervised(env, **vars(args))
    elif args.strategy == "dqn":
        run_dqn(env, **vars(args))
    else:
        raise (f"Strategy {strategy} unknown.")


if __name__ == "__main__":
    main()
