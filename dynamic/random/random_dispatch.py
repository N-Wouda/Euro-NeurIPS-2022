import functools

from .. import utils


def random_dispatch(prob):
    """
    Return the random dispatch strategy where requests are dispatched with
    probability ``prob``.
    """
    return functools.partial(_dispatch_decision, prob=prob)


def _dispatch_decision(info, observation, rng, prob):
    """
    Decide which requests to dispach in the current epoch. Non-must dispatch
    requests are dispatched with probability ``prob``.
    """
    instance = observation["epoch_instance"]
    to_dispatch = instance["is_depot"] | instance["must_dispatch"] | rng.random(instance["must_dispatch"].shape) < prob
    return utils.filter_instance(instance, to_dispatch)
