import argparse
import cProfile
import pstats
import sys
from datetime import datetime

import tools
from environment import ControllerEnvironment, VRPEnvironment
from strategies import solve_dynamic, solve_hindsight
from strategies.dynamic import STRATEGIES


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--instance")
    parser.add_argument("--instance_seed", type=int, default=1)
    parser.add_argument("--solver_seed", type=int, default=1)
    parser.add_argument("--static", action="store_true")
    parser.add_argument("--epoch_tlim", type=int, default=120)
    parser.add_argument("--profile", action="store_true")

    problem_type = parser.add_mutually_exclusive_group(required=True)
    problem_type.add_argument("--hindsight", action="store_true")
    problem_type.add_argument("--strategy",
                              choices=STRATEGIES.keys(),
                              default="rollout")

    return parser.parse_args()


def run(args):
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

    if args.hindsight:
        solve_hindsight(env, **vars(args))
    else:
        solve_dynamic(env, STRATEGIES[args.strategy], **vars(args))


def main():
    args = parse_args()

    if args.profile:
        with cProfile.Profile() as profiler:
            run(args)

        stats = pstats.Stats(profiler).strip_dirs().sort_stats("time")
        stats.print_stats()

        now = datetime.now().isoformat()
        stats.dump_stats(f"tmp/profile-{now}.pstat")
    else:
        run(args)


if __name__ == "__main__":
    main()
