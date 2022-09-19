import os
import argparse
import stable_baselines3

from pathlib import Path
from datetime import datetime

from dynamic_environment import DynamicVRPEnvironment


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--model", choices=["DQN", "A2C", "PPO"], default="DQN")
    parser.add_argument("--policy", choices=["MlpPolicy", "CnnPolicy"], default="MlpPolicy")
    parser.add_argument("--reward_type", type=int, choices=range(4), default=0)
    parser.add_argument("--total_timesteps", type=int)
    parser.add_argument("--epoch_tlim", type=int, default=120)
    parser.add_argument("--solver_tlim", type=int, default=30)
    parser.add_argument("--verbose", type=int, default=2)
    parser.add_argument("--output_dir", default="models")

    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    env = DynamicVRPEnvironment(args.epoch_tlim, args.solver_tlim, args.reward_type, verbose=args.verbose)
    model = getattr(stable_baselines3, args.model)(args.policy, env, verbose=args.verbose)

    model_name = "%s-%s-r%d-%d" % (args.model, args.policy, args.reward_type, datetime.timestamp(datetime.now()))

    model.learn(total_timesteps=args.total_timesteps)
    model.save(Path(args.output_dir) / (model_name + ".model"))

    print("Done for reward type model %s" % model_name)
