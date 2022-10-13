import argparse

import numpy as np
from ConfigSpace import ConfigurationSpace, UniformIntegerHyperparameter
from mpi4py import MPI
from smac import HyperparameterOptimizationFacade, Scenario
from smac.runhistory.dataclasses import TrialValue
from glob import glob

import hgspy
import tools
from strategies.config import Config
from strategies.static import hgs


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

    # Population management search space
    # TODO get this from a file?
    params = [
        UniformIntegerHyperparameter("minPopSize", 5, 100, 25),
        UniformIntegerHyperparameter("generationSize", 1, 100, 40),
        UniformIntegerHyperparameter("nbElite", 0, 25, 4),
        UniformIntegerHyperparameter("nbClose", 1, 25, 5),
    ]

    for param in params:
        cs.add_hyperparameter(param)

    return cs


def evaluate(config, instance: str, seed: int):
    # TODO  tools.static_time_limit(tools.name2size(inst), "quali")
    run_time = 5
    params = defaults.solver_params() | config.get_dictionary()

    res = hgs(
        tools.read_vrplib(instance),
        hgspy.Config(seed=seed, **params),
        defaults.node_ops(),
        defaults.route_ops(),
        defaults.crossover_ops(),
        hgspy.stop.MaxRuntime(run_time)
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

    args = parse_args()

    # TODO make this work with actual instances / kfold crossvalidation?
    instances = args2instances(args)
    features = {name: [tools.name2size(name)] for name in instances}

    settings = {
        "configspace": get_space(args.seed),
        "instances": instances,
        "instance_features": features,
        "output_directory": args.out_dir,
    }

    if args.num_trials:
        settings["n_trials"] = args.num_trials
        stop = hgspy.stop.MaxIterations(args.num_trials)
    else:
        settings["walltime_limit"] = args.time_limit
        stop = hgspy.stop.MaxRuntime(args.time_limit)

    smac = HyperparameterOptimizationFacade(
        Scenario(**settings),
        evaluate,
        overwrite=args.overwrite
    )

    # The processor at rank 0 is in charge of the SMAC algorithm; the other
    # processors only help out with evaluating the requested configurations.
    while True:
        done = stop() if rank == 0 else False
        done = comm.bcast(done, root=0)

        if done:
            break

        info = smac.ask() if rank == 0 else None
        info = comm.bcast(info, root=0)

        objs = [evaluate(info.config, inst, info.seed)
                for inst in instances]

        objs = comm.gather(np.mean(objs), root=0)  # objectives

        if rank == 0:
            avg_cost = np.mean(objs)
            smac.tell(info, TrialValue(cost=avg_cost))

    if rank == 0:
        incumbent = smac.optimize()
        print(incumbent)


if __name__ == "__main__":
    main()
