import numpy as np

from scipy import stats
from scipy.spatial.distance import squareform


def _filter_instance_value(value: np.ndarray, mask: np.ndarray):
    """ Filter an n dimensional value based on a provided mask """
    ndim = np.ndim(value)

    if ndim == 0:
        return value

    if ndim == 1:
        return value[mask]

    if ndim == 2:
        shape = np.shape(value)
        if shape[0] != shape[1]:
            return value[mask]
        else:
            return value[mask][:, mask]

    raise NotImplementedError()


def filter_instance(instance: dict, mask: np.ndarray):
    """ Filter all items of an instance based on a provided mask """
    return {key: _filter_instance_value(value, mask) for key, value in instance.items()}


def describe_current_nodes(instance: dict):
    return np.stack((instance['must_dispatch'],
                     instance["demands"],
                     instance['service_times'],
                     *instance['coords'].T,
                     *instance['time_windows'].T,
                     )).T[1:]


def _describe_item(values, axis: int = None):
    """ Get statistic features of the input data """
    size = np.size(values)

    if size == 0:
        if np.ndim(values) == 1:
            return np.array([0, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan])
        else:
            return np.tile(np.array([np.nan, np.nan, np.nan, np.nan, np.nan, np.nan]), (np.shape(values)[0], 1)).T

    if size == 1:
        return np.array([1, np.min(values), np.max(values), np.mean(values), 0, 0, -3])

    description = stats.describe(values, axis=axis)

    description_values = (*description.minmax,
                          description.mean,
                          description.variance,
                          description.skewness,
                          description.kurtosis)

    if np.ndim(values) == 1:
        description_values = (description.nobs,) + description_values

    return np.asarray(description_values)


def describe_instance(instance: dict):
    """ Get static features describing the characteristics of an entire instance """
    n_nodes = np.sum(~instance['is_depot'])
    capacity = instance['capacity']

    description = _describe_item((instance['coords'][:, 0],
                                  instance['coords'][:, 1],
                                  instance['demands'],
                                  instance['time_windows'][:, 0],
                                  instance['time_windows'][:, 1],
                                  instance['service_times']
                                  ), axis=1)

    duration_description = _describe_item((squareform(instance['duration_matrix'], checks=False),
                                           squareform(instance['duration_matrix'].T, checks=False)), axis=1)

    description_values = (n_nodes,
                          capacity,
                          *description.T.flatten(),
                          *duration_description.T.flatten())

    return np.asarray(description_values)


def _describe_clusters(outgoing, incoming, clusters, tws=None):
    """ Get statistic features of outgoing and incoming distances per cluster (filtered on time window feasibility)"""
    if tws is not None:
        (node_tw_open, node_tw_close), (tws_open, tws_close) = tws

        outgoing = outgoing[node_tw_open + outgoing <= tws_close]
        incoming = incoming[tws_open + incoming <= node_tw_close]

    yield _describe_item(outgoing)
    yield _describe_item(incoming)
    for lower, upper in zip(clusters, clusters[1:]):
        yield _describe_item(outgoing[(lower <= outgoing) & (outgoing < upper)])
        yield _describe_item(incoming[(lower <= incoming) & (incoming < upper)])


def _describe_node(n, nodes, instance, n_clusters):
    idx = np.arange(instance['is_depot'].size)

    if 'customer_idx' in nodes and 'customer_idx' not in instance:
        n = nodes['customer_idx'][n]

    durations = instance['duration_matrix'][n][idx != n], \
                instance['duration_matrix'][:, n][idx != n]

    clusters = np.quantile(np.concatenate(durations), np.linspace(0, 1, n_clusters + 1))

    params = *durations, clusters

    tw_params = instance['time_windows'][n], \
                instance['time_windows'][idx != n].T

    unfiltered = _describe_clusters(*params)
    filtered = _describe_clusters(*params, tw_params)

    return np.concatenate((*unfiltered, *filtered))


def describe_nodes(nodes: dict, instance: dict = None, n_clusters: int = 0):
    """ Get statistic features of each node in relation to an instance """
    if instance is None:
        instance = nodes

    if len(nodes['is_depot']) <= 1:
        return np.empty((0, 28 * (n_clusters + 1)))
    return np.stack([_describe_node(node, nodes, instance, n_clusters)
                     for node, depot in enumerate(nodes['is_depot'])
                     if not depot])


def dispatch_decision(instance, p):
    """ Decide which nodes are dispatched in the current epoch. Each optional node is picked with probability p """
    return instance["is_depot"] | instance["must_dispatch"] | (np.random.random(instance["must_dispatch"].shape) < p)


def idx2request(solution, instance, postpone_routes=True):
    """Map solution indices to request indices of the epoch instance."""
    return [instance["request_idx"][route] for route in solution if not postpone_routes or np.any(instance['must_dispatch'][route])]
