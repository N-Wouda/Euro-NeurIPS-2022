import argparse
from dataclasses import dataclass

import tomli_w
from scipy.stats import qmc


@dataclass
class Integer:
    interval: tuple[int, int]
    default: int

    def ppf(self, q: float) -> int:
        lo, hi = self.interval
        return round(lo + q * (hi - lo))


@dataclass
class Float:
    interval: tuple[float, float]
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
        params=params,
    )

    with open(where + f"/{exp}.toml", "wb") as fh:
        tomli_w.dump(dict(static=static), fh)


def main():
    args = parse_args()

    # LS management
    space = dict(
        nbGranular=Integer((10, 100), 40),
        weightWaitTime=Integer((1, 25), 2),
        weightTimeWarp=Integer((1, 25), 10),
        circleSectorOverlapToleranceDegrees=Integer((0, 359), 0),
        minCircleSectorSizeDegrees=Integer((0, 359), 15),
        postProcessPathLength=Integer((1, 8), 6),
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
