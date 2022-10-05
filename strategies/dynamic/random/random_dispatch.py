import strategies.utils as utils


def random_dispatch(info, observation, rng, prob):
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
