import static


def solve_simulation(instance, max_iterations=None, **kwargs):
    config = static.Config("simulate.toml")

    res = static.solve(instance, config, max_iterations=max_iterations, **kwargs)

    return static.get_solution(res)
