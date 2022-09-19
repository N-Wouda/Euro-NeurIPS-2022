import numpy as np


def sol2ep(solution, ep_inst, postpone_routes=True):
    """Map solution indices to request indices of the epoch instance."""
    return [ep_inst["request_idx"][route] for route in solution if not postpone_routes or np.any(ep_inst['must_dispatch'][route])]
