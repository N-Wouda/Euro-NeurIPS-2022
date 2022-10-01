from dynamic import utils


def _dispatch_random(info, observation, rng, prob):
    """
    Decide which requests to dispach in the current epoch. Non-must dispatch
    requests are dispatched with probability ``prob``.
    """
    instance = observation["epoch_instance"]
    to_dispatch = (
        instance["is_depot"]
        | instance["must_dispatch"]
        | (rng.random(instance["must_dispatch"].shape) < prob)
    )
    return utils.filter_instance(instance, to_dispatch)


def greedy(info, observation, rng):
    return _dispatch_random(info, observation, rng, prob=1)


def random(info, observation, rng):
    return _dispatch_random(info, observation, rng, prob=0.5)


def lazy(info, observation, rng):
    return _dispatch_random(info, observation, rng, prob=0)
