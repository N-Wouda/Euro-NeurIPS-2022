import argparse
import logging
from glob import glob

import numpy as np
from ConfigSpace import (
    ConfigurationSpace,
    EqualsCondition,
    Float,
    Integer,
    NotEqualsCondition,
)
from mpi4py import MPI
from smac import AlgorithmConfigurationFacade as ACFacade, Scenario
from smac.runhistory.dataclasses import TrialValue

import hgspy
import tools
from strategies.config import Config
from strategies.static import hgs

logger = logging.getLogger(__name__)
defaults = Config.from_file("configs/solver.toml").static()


def parse_args():
    parser = argparse.ArgumentParser()

    size = parser.add_mutually_exclusive_group(required=True)
    size.add_argument("--small", action="store_true")
    size.add_argument("--medium", action="store_true")
    size.add_argument("--large", action="store_true")

    stop = parser.add_mutually_exclusive_group(required=True)
    stop.add_argument("--time_limit", type=int)
    stop.add_argument("--num_trials", type=int)

    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--out_dir", default="tmp/smac")

    parser.add_argument("--fold", type=int, required=True)
    parser.add_argument("--num_folds", type=int, required=True)

    return parser.parse_args()


def get_space(seed: int):
    cs = ConfigurationSpace(seed=seed)

    params = [
        # Penalty management
        Integer("initialTimeWarpPenalty", (1, 25), default=1),
        Integer("nbPenaltyManagement", (25, 500), default=100, q=25),
        Float("feasBooster", (1, 10), default=2.0, q=0.1),
        Float("penaltyIncrease", (1, 5), default=1.2, q=0.1),
        Float("penaltyDecrease", (0.25, 1), default=0.85, q=0.05),
        Float("targetFeasible", (0, 1), default=0.4, q=0.05),
        Integer("repairProbability", (0, 100), default=50, q=5),
        Integer("repairBooster", (1, 25), default=10),
        # Population management
        Integer("minPopSize", (5, 100), default=25, q=5),
        Integer("generationSize", (0, 100), default=40, q=5),
        Integer("nbElite", (0, 25), default=4, q=5),
        Integer("nbClose", (1, 25), default=5, q=4),
        # Restart mechanism
        Integer("nbIter", (1_000, 10_000), default=10_000, q=250),
        Integer("nbKeepOnRestart", (0, 5), default=0),
        # Crossover
        Integer("selectProbability", (50, 100), default=90, q=10),
        Integer("destroyPct", (5, 50), default=20, q=5),
        Integer("brokenPairsExchange", (0, 1), default=0),
        Integer("selectiveRouteExchange", (0, 1), default=1),
        # Node ops
        Integer("Exchange10", (0, 1), default=1),
        Integer("Exchange11", (0, 1), default=1),
        Integer("Exchange20", (0, 1), default=1),
        Integer("MoveTwoClientsReversed", (0, 1), default=1),
        Integer("Exchange21", (0, 1), default=1),
        Integer("Exchange22", (0, 1), default=1),
        Integer("TwoOpt", (0, 1), default=1),
        Integer("Exchange31", (0, 1), default=0),
        Integer("Exchange32", (0, 1), default=0),
        Integer("Exchange33", (0, 1), default=0),
        # Granular neighborhoods
        Integer("nbGranular", (10, 100), default=40, q=5),
        Integer("weightWaitTime", (1, 25), default=2),
        Integer("weightTimeWarp", (1, 25), default=10),
        # Intensification (and route ops)
        Integer("shouldIntensify", (0, 1), default=1),
        Integer("circleSectorOverlapTolerance", (0, 359), default=0),
        Integer("minCircleSectorSize", (0, 359), default=15),
        Integer("postProcessPathLength", (1, 8), default=7),
        # In principle RELOCATE* and SWAP* route ops are also parameters, but
        # those are not that expensive. The choice to even intensify in the
        # first place already covers this.
    ]

    cs.add_hyperparameters(params)

    conditions = [
        # Only valid parameters when we do, in fact, intensify
        EqualsCondition(
            cs["circleSectorOverlapTolerance"], cs["shouldIntensify"], 1
        ),
        EqualsCondition(cs["minCircleSectorSize"], cs["shouldIntensify"], 1),
        EqualsCondition(cs["postProcessPathLength"], cs["shouldIntensify"], 1),
        # Repair booster only makes sense when we can actually repair
        NotEqualsCondition(cs["repairBooster"], cs["repairProbability"], 0),
        # Parameter is specific to BPX
        EqualsCondition(cs["destroyPct"], cs["brokenPairsExchange"], 1),
    ]

    cs.add_conditions(conditions)

    return cs


def evaluate(config, instance: str, seed: int):
    # TODO set run time to quali
    run_time = 5  # tools.static_time_limit(tools.name2size(instance), "quali")

    # TODO parse params

    params = defaults.solver_params() | config.get_dictionary()

    res = hgs(
        tools.read_vrplib(instance),
        hgspy.Config(seed=seed, **params),
        defaults.node_ops(),
        defaults.route_ops(),
        defaults.crossover_ops(),
        hgspy.stop.MaxRuntime(run_time),
    )

    return res.get_best_found().cost()


def args2instances(args):
    locs = "instances/ORTEC-VRPTW*n{format}*.txt"

    if args.small:
        return glob(locs.format(format="[0-2]"))
    elif args.medium:
        return glob(locs.format(format="[3-4]"))
    else:
        return glob(locs.format(format="[5-9]"))


def main():
    comm = MPI.COMM_WORLD
    rank = comm.Get_rank()
    size = comm.Get_size()

    args = parse_args()

    # TODO make this work with crossvalidation in distributed fashion
    instances = args2instances(args)[:2]
    assert len(instances) % size == 0, "Cannot cleanly divide work!"
    instances = np.split(np.array(instances), size)
    local_instances = instances[rank]

    settings = {
        "configspace": get_space(args.seed),
        "output_directory": args.out_dir,
    }

    if args.num_trials:
        settings["n_trials"] = args.num_trials
        stop = hgspy.stop.MaxIterations(args.num_trials)
    else:
        settings["walltime_limit"] = args.time_limit
        stop = hgspy.stop.MaxRuntime(args.time_limit)

    if rank == 0:
        # Processor 0 is in charge of the SMAC algorithm; the others only help
        # out with evaluating the requested configurations.
        smac = ACFacade(
            Scenario(**settings), evaluate, overwrite=args.overwrite
        )

    while True:
        done = stop() if rank == 0 else False
        done = comm.bcast(done, root=0)

        if done:
            break

        info = smac.ask() if rank == 0 else None
        info = comm.bcast(info, root=0)

        objs = [
            evaluate(info.config, inst, info.seed) for inst in local_instances
        ]

        objs = comm.gather(np.mean(objs), root=0)  # objectives

        if rank == 0:
            avg_cost = np.mean(objs)
            smac.tell(info, TrialValue(cost=avg_cost))

            logger.info(
                f"Ran {info.config.get_dictionary()} with average cost "
                f"{avg_cost:.0f}"
            )

    if rank == 0:
        print(smac.incumbent)


if __name__ == "__main__":
    main()
