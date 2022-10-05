import strategies.utils as utils


def random_dispatch(info, obs, rng, prob):
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
    return utils.filter_instance(instance, to_dispatch)
