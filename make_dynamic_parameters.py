import argparse
from dataclasses import dataclass
from random import seed, uniform

import tomli_w
from scipy.stats import qmc

from strategies.config import Config


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
    default = Config.from_file("configs/benchmark_dynamic.toml").dynamic()
    default["strategy_params"].update(**params)

    with open(where + f"/{exp}.toml", "wb") as fh:
        tomli_w.dump(dict(dynamic=default), fh)


def main():
    args = parse_args()
    seed(args.seed)

    space = dict(
        simulate_tlim_factor=Float((0.5, 0.9), 0.7),
        n_cycles=Integer((1, 3), 1),
        n_simulations=Integer((25, 100), 50),
        n_lookahead=Integer((1, 5), 1),
        n_requests=Integer((50, 100), 100),
    )

    default = {name: val.default for name, val in space.items()}
    default["postpone_thresholds"] = [0.65]

    write(args.out_dir, default, 1)

    sampler = qmc.LatinHypercube(d=len(space), centered=True, seed=args.seed)
    samples = sampler.random(args.num_samples - 1)

    for exp, sample in enumerate(samples, 2):
        values = [param.ppf(val) for param, val in zip(space.values(), sample)]
        scenario = {name: val for name, val in zip(space.keys(), values)}

        thresholds = [
            uniform(0.5, 0.8),
            uniform(0.6, 0.9),
            uniform(0.70, 0.95),
            uniform(0.80, 0.95),
        ]

        scenario["postpone_thresholds"] = thresholds

        write(args.out_dir, scenario, exp)


if __name__ == "__main__":
    main()
