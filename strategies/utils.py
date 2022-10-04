import numpy as np


def sol2ep(solution, ep_inst, postpone_routes=True):
    """Map solution indices to request indices of the epoch instance."""
    return [
        ep_inst["request_idx"][route]
        for route in solution
        if not postpone_routes or np.any(ep_inst["must_dispatch"][route])
    ]


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


def delta_cost(route, dist):
    """
    Return the delta cost for each client on the route.
    """
    if len(route) == 0:
        return []

    pred = np.array([0] + route[:-1], dtype=int)
    succ = np.array(route[1:] + [0], dtype=int)
    return dist[pred, route] + dist[route, succ] - dist[pred, succ]
