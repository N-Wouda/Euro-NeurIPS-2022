import argparse
from dataclasses import dataclass

import tomli_w
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


def write(where: str, params, exp: int):
    static = dict(
        node_ops=[
            "Exchange10",
            "Exchange11",
            "Exchange20",
            "MoveTwoClientsReversed",
            "Exchange21",
            "Exchange22",
            "TwoOpt",
        ],
        route_ops=[
            "RelocateStar",
            "SwapStar",
        ],
        crossover_ops=[
            "selective_route_exchange",
        ],
        params=params
    )

    with open(where + f"/{exp}.toml", "wb") as fh:
        tomli_w.dump(dict(static=static), fh)


def main():
    args = parse_args()

    # Population management
    space = dict(
        minPopSize=Integer((5, 100), 25),
        generationSize=Integer((1, 100), 40),
        nbElite=Integer((0, 25), 4),
        lbDiversity=Float((0, 0.25), 0.1),
        ubDiversity=Float((0.25, 1), 0.5),
        nbClose=Integer((1, 25), 5),
        nbIter=Integer((1_000, 10_000), 10_000),
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
