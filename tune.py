import argparse
import logging
from glob import glob

from ConfigSpace import (
    ConfigurationSpace,
    EqualsCondition,
    Float,
    ForbiddenAndConjunction,
    ForbiddenEqualsClause,
    Integer,
    NotEqualsCondition,
)
from mpi4py import MPI
from mpi4py.futures import MPICommExecutor, wait
from smac import AlgorithmConfigurationFacade as ACFacade, Scenario
from smac.runhistory.dataclasses import TrialValue

import hgspy
import tools
from strategies.static import hgs

logger = logging.getLogger(__name__)
mpi_comm = MPI.COMM_WORLD
mpi_rank = mpi_comm.Get_rank()


def parse_args():
    parser = argparse.ArgumentParser()

    size = parser.add_mutually_exclusive_group(required=True)
    size.add_argument("--small", action="store_true")
    size.add_argument("--medium", action="store_true")
    size.add_argument("--large", action="store_true")

    parser.add_argument("--time_limit", type=int)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--out_dir", default="tmp/smac")

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
        Integer("nbElite", (0, 25), default=4),
        Integer("nbClose", (1, 25), default=5),
        # Restart mechanism
        Integer("nbIter", (1_000, 10_000), default=10_000, q=250),
        Integer("nbKeepOnRestart", (0, 5), default=0),
        # Crossover
        Integer("selectProbability", (50, 100), default=90, q=10),
        Integer("destroyPct", (5, 50), default=20, q=5),
        Integer("broken_pairs_exchange", (0, 1), default=0),
        Integer("selective_route_exchange", (0, 1), default=1),
        # Node ops
        # In principle RELOCATE and SWAP are also parameters, but those are so
        # fundamental there's no point really in tuning their presence.
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
        Integer("circleSectorOverlapToleranceDegrees", (0, 359), default=0),
        Integer("minCircleSectorSizeDegrees", (0, 359), default=15),
        Integer("postProcessPathLength", (1, 8), default=7),
        # In principle RELOCATE* and SWAP* route ops are also parameters, but
        # those are not that expensive. The choice to even intensify in the
        # first place already covers this.
    ]

    cs.add_hyperparameters(params)

    conditions = [
        # Only valid parameters when we do, in fact, intensify
        EqualsCondition(
            cs["circleSectorOverlapToleranceDegrees"], cs["shouldIntensify"], 1
        ),
        EqualsCondition(
            cs["minCircleSectorSizeDegrees"], cs["shouldIntensify"], 1
        ),
        EqualsCondition(cs["postProcessPathLength"], cs["shouldIntensify"], 1),
        # Repair booster only makes sense when we can actually repair
        NotEqualsCondition(cs["repairBooster"], cs["repairProbability"], 0),
        # Parameter is specific to BPX
        EqualsCondition(cs["destroyPct"], cs["broken_pairs_exchange"], 1),
    ]

    cs.add_conditions(conditions)

    forbidden_clauses = [
        # BPX and SREX cannot both be zero at the same time
        ForbiddenAndConjunction(
            ForbiddenEqualsClause(cs["broken_pairs_exchange"], 0),
            ForbiddenEqualsClause(cs["selective_route_exchange"], 0),
        ),
    ]

    cs.add_forbidden_clauses(forbidden_clauses)

    return cs


def evaluate(config, instance: str, seed: int):
    logger.debug(f"Evaluating {instance} with {seed = } on rank {mpi_rank}.")

    run_time = tools.static_time_limit(tools.name2size(instance), "quali")
    params = config.get_dictionary().copy()

    node_ops = [
        "Exchange10",
        "Exchange11",
        "Exchange20",
        "MoveTwoClientsReversed",
        "Exchange21",
        "Exchange22",
        "TwoOpt",
        "Exchange31",
        "Exchange32",
        "Exchange33",
    ]

    node_ops = [
        getattr(hgspy.operators, op) for op in node_ops if params.pop(op, True)
    ]

    route_ops = [hgspy.operators.RelocateStar, hgspy.operators.SwapStar]

    crossover_ops = [
        "broken_pairs_exchange",
        "selective_route_exchange",
    ]

    crossover_ops = [
        getattr(hgspy.crossover, op)
        for op in crossover_ops
        if params.pop(op, False)
    ]

    res = hgs(
        tools.read_vrplib(instance),
        hgspy.Config(seed=seed, **params),
        node_ops,
        route_ops,
        crossover_ops,
        hgspy.stop.MaxRuntime(run_time),
    )

    cost = res.get_best_found().cost()
    logger.debug(f"Evaluated {instance}: {cost}.")

    return cost


def args2instances(args):
    locs = "instances/ORTEC-VRPTW*n{format}*.txt"

    if args.small:
        return glob(locs.format(format="[0-2]"))
    elif args.medium:
        return glob(locs.format(format="[3-4]"))
    else:
        return glob(locs.format(format="[5-9]"))


def main():
    with MPICommExecutor() as ex:
        if ex is None:  # then this process is a worker
            return

        args = parse_args()
        num_workers = MPI.COMM_WORLD.Get_size() - 1

        instances = args2instances(args)
        features = {name: [tools.name2size(name)] for name in instances}

        settings = {
            "configspace": get_space(args.seed),
            "instances": instances,
            "instance_features": features,
            "min_budget": 10,
            "output_directory": args.out_dir,
        }

        logger.info(f"SMAC evaluator is running with {num_workers = }.")

        stop = hgspy.stop.MaxRuntime(args.time_limit)
        smac = ACFacade(
            Scenario(**settings), evaluate, overwrite=args.overwrite
        )

        while not stop():
            # First submit work to the worker processes, so those are busy..
            other = [smac.ask() for _ in range(num_workers)]
            futures = [
                ex.submit(evaluate, info.config, info.instance, info.seed)
                for info in other
            ]

            # ..then make sure we're doing something useful too..
            here = smac.ask()
            obj = evaluate(here.config, here.instance, here.seed)
            smac.tell(here, TrialValue(cost=obj))

            # ..wait until all worker jobs have completed...
            wait(futures)

            # ..and update SMAC with their observations
            for info, fut in zip(other, futures):
                smac.tell(info, TrialValue(cost=fut.result()))

        print(smac.incumbent)


if __name__ == "__main__":
    main()