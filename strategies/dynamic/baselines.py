from functools import partial

from strategies.utils import filter_instance


def _random_dispatch(info, obs, rng, prob, **kwargs):
    """
    Decide which requests to dispatch in the current epoch. Requests that are
    not "must dispatch" are dispatched with probability ``prob``.
    """
    instance = obs["epoch_instance"]
    to_dispatch = (
        instance["is_depot"]
        | instance["must_dispatch"]
        | (rng.random(instance["must_dispatch"].shape) < prob)
    )
    return filter_instance(instance, to_dispatch)


greedy = partial(_random_dispatch, prob=1)
lazy = partial(_random_dispatch, prob=0)
random = partial(_random_dispatch, prob=0.5)
