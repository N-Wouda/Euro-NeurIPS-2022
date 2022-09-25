from .. import utils


def random_dispatch(prob):
    """
    Factory for a random dispatch strategy.
    """

    def strategy(info, obs, rng):
        return _dispatch_decision(info, obs, rng, prob)

    return strategy


def _dispatch_decision(info, observation, rng, prob):
    """
    Decide which requests to dispach in the current epoch. Non-must dispatch
    requests are dispatched with probability ``prob``.
    """
    instance = observation["epoch_instance"]
    disp_now = rng.random(instance["must_dispatch"].shape) < prob
    disp_idcs = instance["is_depot"] | instance["must_dispatch"] | disp_now
    return utils.filter_instance(instance, disp_idcs)
