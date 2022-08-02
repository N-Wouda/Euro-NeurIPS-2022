from copy import copy, deepcopy
import numpy as np

import logging

import tools


def sisr_exchange(parents, params, xor_shift, instance, rnd_state):
    """
    Perform two crossovers a la Slack Induction by String Removal.
    """
    hgspy = tools.get_hgspy_module()
    one, two = parents

    # custs = list(range(1, len(instance["coords"])))
    # rnd_state.shuffle(custs)
    # temp = hgspy.Individual(params, [custs])
    # debug(temp)
    # return temp

    # temp = hgspy.Individual(params, xor_shift)
    # debug(temp)
    # return temp

    # Compute the center client
    center = rnd_state.integers(1, len(instance["coords"]))

    # Remove substrings from the parents
    dest1, rem1 = string_removal(one.get_routes(), center, rnd_state, instance)
    dest2, rem2 = string_removal(two.get_routes(), center, rnd_state, instance)

    print(rem1, rem2)
    # breakpoint()

    # # Remove the substrings from the other parents
    dest1 = remove(dest1, rem2)
    dest2 = remove(dest2, rem1)

    rem = list(set(rem1 + rem2))
    n_overlap = len(rem1) + len(rem2) - len(rem)
    print(
        f"{len(rem)=:02d}, {n_overlap=:02d}, {len(rem1)=:02d}, {len(rem2)=:02d}"
    )

    cand1 = greedy_repair_with_blinks(
        dest1, rem, rnd_state, instance, params, hgspy
    )
    cand2 = greedy_repair_with_blinks(
        dest2, rem, rnd_state, instance, params, hgspy
    )

    # debug(cand1)
    # debug(cand2)

    if cand1.cost() < cand2.cost():
        return cand1
    else:
        return cand2


def debug(cand):
    print(f"{cand.cost()=}")
    print(f"{cand.get_routes()=}")
    print(f"{cand.get_tour()=}")
    print(f"{len(cand.get_tour())=}")


# Algorithm parameters
NUM_DESTRUCTION = 6  # Average number of customers to be removed per iteration
MAX_CARDINALITY = 10  # L^\max: the maximum cardinality of the removed strings
SPLIT_RATE = 0.7  # alpha
SPLIT_DEPTH = 0.01  # beta
BLINK_RATE = 0.01  # gamma


def string_removal(routes, center, rnd_state, instance):
    """
    Destroy operator of the Slack Induction by String Removal method by [1].
    Strings, i.e., partial routes, are iteratively removed from the solution
    around a randomly chosen customer.

    References
    ----------
    [1] Christiaens, J., & Vanden Berghe, G. (2020). Slack induction by string
    removals for vehicle routing problems. Transportation Science, 54(2), 417-433.
    """
    destroyed = deepcopy(routes)
    unassigned = []

    max_str_card = max_string_cardinality(destroyed)
    max_destroyed_routes = n_strings_to_remove(max_str_card, rnd_state)
    destroyed_routes = []

    for customer in neighbors(center, instance):

        if len(destroyed_routes) >= max_destroyed_routes:
            break

        if customer in unassigned:
            continue

        route = find_route(destroyed, customer)
        if route in destroyed_routes:
            continue

        destroyed_custs = remove_string(
            route, customer, max_str_card, rnd_state
        )

        unassigned.extend(destroyed_custs)
        destroyed_routes.append(route)

    return destroyed, unassigned


def max_string_cardinality(solution):
    """
    Compute the maximum cardinality of strings to be removed.

    Eq. (5) in [1].
    """
    avg_card = int(np.mean([len(route) for route in solution]))
    return min(MAX_CARDINALITY, avg_card)


def n_strings_to_remove(max_string_card, rnd_state):
    """
    Compute the number of strings to remove.

    Eq. (6) and (7) in [1].
    """
    max_n_strings_to_remove = max(
        1, (4 * NUM_DESTRUCTION) / (1 + max_string_card) - 1
    )
    return rnd_state.integers(1, max_n_strings_to_remove + 1)


def neighbors(customer, instance):
    """Return the nearest neighbors of the customer, excluding the depot."""
    locations = np.argsort(instance["duration_matrix"][customer])
    return locations[locations != 0]


def find_route(solution, customer):
    """Return the route that contains the customer."""
    for route in solution:
        if customer in route:
            return route

    raise ValueError(f"Customer {customer} not in solution.")


def remove_string(route, cust, max_string_card, rnd_state):
    """
    Remove a string from the passed-in route using either the `string` procedure
    or the `split-string` procedure. Procedure is selected randomly based on
    `SPLIT_RATE`.

    `removeSelected` in [1].
    """
    card = string_cardinality(route, max_string_card, rnd_state)

    # "String" removal procedure
    if rnd_state.random() <= SPLIT_RATE:
        rem_idcs = select_string(route, cust, card, rnd_state)

    # "Split-string" removal procedure
    else:
        substr_len = 1
        while (
            rnd_state.random() > SPLIT_DEPTH and substr_len < len(route) - card
        ):
            substr_len += 1

        str_idcs = select_string(route, cust, card + substr_len, rnd_state)

        # Remove a substring of length m from the string
        start = rnd_state.integers(len(str_idcs) - substr_len)
        rem_idcs = str_idcs[:start] + str_idcs[start + substr_len :]

    rem_custs = [cust for idx, cust in enumerate(route) if idx in rem_idcs]

    for cust in rem_custs:
        route.remove(cust)

    return rem_custs


def string_cardinality(route, max_card, rnd_state):
    """
    Compute the cardinality of the string to be removed from the route.

    Eq. (8) and (9) in [1].
    """
    return rnd_state.integers(1, min(len(route), max_card) + 1)


def select_string(route, customer, cardinality, rnd_state):
    """
    Return the route indices of a randomly selected string of length
    cardinality, containing the passed-in customer.
    """
    route_idx = route.index(customer)
    customer_pos = rnd_state.integers(cardinality)
    start = route_idx - customer_pos
    string_indices = [
        idx % len(route) for idx in range(start, start + cardinality)
    ]
    return string_indices


def remove(solution, unassigned):
    for cust in unassigned:
        for route in solution:
            if cust in route:
                route.remove(cust)

    return solution


###########################


def greedy_repair_with_blinks(
    routes, unassigned, rnd_state, instance, params, hgspy
):
    """
    Greedily insert the unassigned customers back with blinking [1].
    """
    unassigned = _sort_sisr(unassigned, rnd_state, instance)

    while len(unassigned) != 0:
        customer = unassigned.pop()
        route, idx = blink_insert(routes, customer, rnd_state, params, hgspy)

        if route is not None:
            route.insert(idx, customer)
        else:
            routes.append([customer])

    hgspy = tools.get_hgspy_module()
    return hgspy.Individual(params, routes)


def _sort_sisr(unassigned, rnd_state, instance):
    """
    Sort the unassigned customers in-place based on different proprties.
    """
    weights = [4, 4, 2, 1, 3, 3, 3]

    order = rnd_state.choice(
        [
            "random",
            "demand",
            "far",
            "close",
            "tw_length",
            "tw_start",
            "tw_end",
        ],
        p=[w / sum(weights) for w in weights],
    )

    if order == "random":
        rnd_state.shuffle(unassigned)
        return unassigned[:]

    elif order == "demand":
        return sorted(unassigned, key=lambda cust: -instance["demands"][cust])

    elif order == "far":
        return sorted(
            unassigned, key=lambda cust: -instance["duration_matrix"][0][cust]
        )

    elif order == "close":
        return sorted(
            unassigned, key=lambda cust: instance["duration_matrix"][0][cust]
        )

    # increasing tw length
    elif order == "tw_length":
        return sorted(
            unassigned,
            key=lambda cust: instance["time_windows"][cust][1]
            - instance["time_windows"][cust][0],
        )

    # Increasing tw start
    elif order == "tw_start":
        return sorted(
            unassigned, key=lambda cust: instance["time_windows"][cust][0]
        )

    elif order == "tw_end":
        return sorted(
            unassigned, key=lambda cust: -instance["time_windows"][cust][1]
        )


def blink_insert(routes, customer, rnd_state, params, hgspy):
    """
    Return the best route and best insertion index. All insertion points are
    enumerated over, but some insertion checks are skipped with probability
    `BLINK_RATE`.
    """
    best_cost, best_route, best_idx = None, None, None

    for route_idx, route in enumerate(routes):
        old = compute_cost(routes, params, hgspy)

        for idx in range(len(route) + 1):
            if rnd_state.random() > BLINK_RATE:
                new_sol = copy(routes)
                new_sol[route_idx] = (
                    route[:idx] + [customer] + route[idx + 1 :]
                )
                cost = compute_cost(new_sol, params, hgspy) - old

                if best_cost is None or cost < best_cost:
                    best_cost, best_route, best_idx = cost, route, idx

    return best_route, best_idx


def compute_cost(routes, params, hgspy):
    """
    Temporary function to compute a solution's penalized cost.
    """
    indiv = hgspy.Individual(params, routes)
    return indiv.cost()
