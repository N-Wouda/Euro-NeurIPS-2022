import os
import gym
import warnings
import numpy as np

import tools
from . import dynamic_tools

from environment import VRPEnvironment
from .solve_static import solve_static


warnings.filterwarnings("ignore", append=True, message="Repeatedly resetting the environment without providing a seed will use the same default seed again")


class DynamicVRPEnvironment(gym.Env):
    N_CLUSTERS = 10

    def __init__(self, epoch_tlim, solver_tlim, reward_type=0, instance=None, seed=None, verbose=0):
        self.epoch_tlim = epoch_tlim
        self.solver_tlim = solver_tlim

        self.reward_type = reward_type

        self.instance = instance
        self.seed = seed

        self.verbose = verbose

        self.is_done = False
        self.epoch_reward = np.nan

        self.action_space = gym.spaces.Discrete(2)

        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (107 + 56 * (self.N_CLUSTERS + 1),))

    @property
    def info(self):
        return {"Instance": self.instance_name,
                "Node": self.epoch_instance['request_idx'][self.node],
                "Epoch": self.env.current_epoch,
                "Reward": self.epoch_reward}

    def reset(self):
        seed = np.random.randint(1e3) if self.seed is None else self.seed
        instance_file = np.random.choice(os.listdir("instances")) if self.instance is None else self.instance

        self.instance_name = "%s-%03d" % (instance_file.rstrip('.txt'), seed)

        instance = tools.read_vrplib("instances/" + instance_file)

        self.env = VRPEnvironment(
            seed,
            instance,
            epoch_tlim=self.epoch_tlim,
            is_static=False
        )

        # ////// Get greedy & hindsight solutions & costs
        if self.verbose: print("Solving greedy", end="\n" if self.verbose > 1 else "\r")

        self.greedy_node_costs = {}

        epoch_info, static_info = self.env.reset()
        while not self.env.is_done:
            epoch_instance = epoch_info["epoch_instance"]

            *_, (idx_routes, _) = solve_static(epoch_instance, self.solver_tlim)
            routes = dynamic_tools.idx2request(idx_routes, epoch_instance, postpone_routes=True)

            self.greedy_node_costs.update(tools.get_node_costs(idx_routes, epoch_instance['duration_matrix']))

            epoch_info, epoch_reward, is_done, solver_info = self.env.step(routes)

            if solver_info["error"] is not None:
                warnings.warn(solver_info["error"])

        if self.verbose: print("Solving hindsight", end="\n" if self.verbose > 1 else "\r")

        hindsight_instance = self.env.get_hindsight_problem()
        *_, (hindsight_routes, _) = solve_static(hindsight_instance, self.solver_tlim)

        self.hindsight_node_costs = tools.get_node_costs(hindsight_routes, hindsight_instance['duration_matrix'])
        # //////

        self.epoch_info, self.static_info = self.env.reset()

        self.static_instance = self.static_info["dynamic_context"]
        self.static_instance_description = dynamic_tools.describe_instance(self.static_instance)

        return self._step_epoch()

    def step(self, action):
        if self.verbose:
            print(self.info, end="\n" if self.verbose > 1 else "\r")
        self.actions[self.node] = action

        if self.node < self.n_nodes - 1:
            return self._step_node(), 0, False, self.info
        else:
            mask = np.append(1, self.actions).astype(bool)
            instance = dynamic_tools.filter_instance(self.epoch_instance, mask)

            *_, (idx_routes, epoch_cost) = solve_static(instance, self.solver_tlim)
            routes = dynamic_tools.idx2request(idx_routes, instance, postpone_routes=True)

            self.epoch_info, self.epoch_reward, self.is_done, self.solver_info = self.env.step(routes)

            if self.solver_info["error"] is not None:
                warnings.warn(self.solver_info["error"])

            node_costs = tools.get_node_costs(routes, instance['duration_matrix'])

            if self.reward_type == 1:
                self.epoch_reward = -sum(node_costs[node] - self.hindsight_node_costs[node] for node in node_costs)
            elif self.reward_type == 2:
                self.epoch_reward = -sum(abs(node_costs[node] - self.hindsight_node_costs[node]) for node in node_costs)
            elif self.reward_type == 3:
                self.epoch_reward = -sum((node_costs[node] - self.hindsight_node_costs[node])**2 for node in node_costs)

            return self._step_epoch() if not self.is_done else None, self.epoch_reward, self.is_done, self.info

    def _state(self):
        return np.concatenate((
            self.current_node_descriptions[self.node],
            self.static_instance_description,
            self.epoch_instance_description,
            self.static_node_descriptions[self.node],
            self.epoch_node_descriptions[self.node]
        ))

    def _step_node(self):
        self.node += 1
        return self._state()

    def _step_epoch(self):
        self.epoch_instance = self.epoch_info["epoch_instance"]
        self.epoch_instance_description = dynamic_tools.describe_instance(self.epoch_instance)

        self.current_node_descriptions = dynamic_tools.describe_current_nodes(self.epoch_instance)
        self.epoch_node_descriptions = dynamic_tools.describe_nodes(self.epoch_instance, n_clusters=self.N_CLUSTERS)
        self.static_node_descriptions = dynamic_tools.describe_nodes(self.epoch_instance, self.static_instance, n_clusters=self.N_CLUSTERS)

        self.node = -1
        self.n_nodes = np.sum(~self.epoch_instance["is_depot"])
        self.actions = np.ones(self.n_nodes, dtype=int)
        return self._step_node()
