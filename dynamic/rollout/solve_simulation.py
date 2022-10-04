import static

config = static.Config("simulate.toml")


def solve_simulation(instance, max_iterations=None, seed=1):
    res = static.solve(
        instance, config, max_iterations=max_iterations, seed=seed
    )

    return static.get_solution(res)
