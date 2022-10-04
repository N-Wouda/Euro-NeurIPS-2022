import static

config = static.Config("simulate.toml")


def solve_simulation(instance, seed=1, max_iterations=None):
    res = static.solve(
        instance, config, max_iterations=max_iterations, seed=seed
    )

    return static.get_solution(res)
