import argparse
import json
from dataclasses import dataclass

from scipy.stats import qmc


@dataclass
class Integer:
    interval: tuple
    default: int

    def ppf(self, q: float) -> int:
        lo, hi = self.interval
        return round(lo + q * (hi - lo))


@dataclass
class Float:
    interval: tuple
    default: float

    def ppf(self, q: float) -> float:
        lo, hi = self.interval
        return lo + q * (hi - lo)


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--num_samples", type=int, default=100)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--out_dir", default="data/tune")

    return parser.parse_args()


def write(where: str, scenario, exp: int):
    with open(where + f"/{exp}.json", "w") as fh:
        json.dump(scenario, fh)


def main():
    args = parse_args()

    # Population management
    space = dict(
        initialTimeWarpPenalty=Integer((1, 25), 1),
        nbPenaltyManagement=Integer((25, 500), 100),
        feasBooster=Float((1, 10), 2.0),
        penaltyIncrease=Float((1, 5), 1.2),
        penaltyDecrease=Float((0.25, 1), 0.85),
        targetFeasible=Float((0, 1), 0.4),
        repairProbability=Integer((0, 100), 50),
        repairBooster=Integer((1, 25), 10),
    )

    default = {name: val.default for name, val in space.items()}
    write(args.out_dir, default, 1)

    sampler = qmc.LatinHypercube(d=len(space), centered=True, seed=args.seed)
    samples = sampler.random(args.num_samples - 1)

    for exp, sample in enumerate(samples, 2):
        values = [param.ppf(val) for param, val in zip(space.values(), sample)]
        scenario = {name: val for name, val in zip(space.keys(), values)}
        write(args.out_dir, scenario, exp)


if __name__ == "__main__":
    main()
