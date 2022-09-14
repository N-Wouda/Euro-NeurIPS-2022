import numpy as np
import functools

from ..solve_static import solve_static
from .. import utils as dynamic_utils
from .utils import load_model, get_request_features


def filter_dqn(observation, rng: np.random.Generator, net):
    import torch

    actions = []
    ep_inst = observation
    observation, static_info = ep_inst.pop("observation"), ep_inst.pop(
        "static_info"
    )
    request_features, global_features = get_request_features(
        observation, static_info, net.k_nearest
    )
    all_features = torch.cat(
        (
            request_features,
            global_features[None, :].repeat(request_features.shape[0], 1),
        ),
        -1,
    )
    actions = net(all_features).argmax(-1).detach().cpu().tolist()
    mask = ep_inst["must_dispatch"] | (np.array(actions) == 0)
    mask[0] = True  # Depot always included in scheduling
    return dynamic_utils.filter_instance(ep_inst, mask)


def run_dqn(env, **kwargs):
    # Load DQN model
    model_path = "dynamic/dqn/pretrained/"
    net = load_model(model_path, device="cpu")
    policy = functools.partial(filter_dqn, net=net)

    rng = np.random.default_rng(kwargs["solver_seed"])

    observation, static_info = env.reset()
    ep_tlim = static_info["epoch_tlim"]
    total_reward = 0

    done = False
    while not done:
        ep_inst = observation["epoch_instance"]

        # DQN strategy requires more data than just the epoch instance
        data = {"observation": observation, "static_info": static_info}
        dispatch_inst = policy({**ep_inst, **data}, rng)

        sol, cost = solve_static(dispatch_inst, ep_tlim - 1)
        ep_sol = dynamic_utils.sol2ep(sol, ep_inst)

        # Submit solution to environment
        observation, reward, done, info = env.step(ep_sol)
        assert cost is None or reward == -cost, f"{info['error']}"

        total_reward += reward

    return total_reward
