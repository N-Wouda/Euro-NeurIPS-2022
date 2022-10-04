import static
import tools


def solve_static(instance, seed=1, max_runtime=60, initial_solutions=(), **kwargs):
    # Return empty solution if the instance contains no clients
    if instance["is_depot"].size <= 1:
        return [], 0

    # Return singleton solution if the instance contains a single client
    if instance["is_depot"].size <= 2:
        solution = [[1]]
        cost = tools.validate_static_solution(instance, solution)
        return solution, cost

    config = static.Config("dynamic.toml")
    res = static.solve(
        instance,
        config,
        max_runtime=max_runtime,
        initial_solutions=initial_solutions,
        seed=seed,
    )

    routes, cost, is_feasible = static.get_solution(res)

    assert is_feasible

    return routes, cost
