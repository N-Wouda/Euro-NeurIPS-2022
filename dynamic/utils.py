def sol2ep(solution, ep_inst):
    """Map solution indices to request indices of the epoch instance."""
    return [ep_inst["request_idx"][route] for route in solution]
