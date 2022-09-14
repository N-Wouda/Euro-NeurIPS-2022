import numpy as np


def sol2ep(solution, ep_inst):
    """Map solution indices to request indices of the epoch instance."""
    return [ep_inst["request_idx"][route] for route in solution]


def _filter_instance_value(value: np.ndarray, mask: np.ndarray):
    """Filter an n dimensional value based on a provided mask"""
    ndim = np.ndim(value)

    if ndim == 0:
        return value
    elif ndim == 1:
        return value[mask]
    elif ndim == 2:
        shape = np.shape(value)
        if shape[0] != shape[1]:
            return value[mask]
        else:
            return value[mask][:, mask]

    raise NotImplementedError()


def filter_instance(instance: dict, mask: np.ndarray):
    """
    Filter all items of an instance using the passed-in mask.
    """
    return {
        key: _filter_instance_value(value, mask)
        for key, value in instance.items()
    }


def dispatch_decision(instance, prob, rng: np.random.Generator) -> np.array:
    """
    Decide which nodes are dispatched in the current epoch. Non-must dispatch
    nodes are dispatched with probability prob.
    """
    dispatch_now = rng.random(instance["must_dispatch"].shape) < prob
    return instance["is_depot"] | instance["must_dispatch"] | dispatch_now
