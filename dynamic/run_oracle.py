from .solve_static import solve_static


def run_oracle(env, **kwargs):
    """
    Solve the dynamic VRPTW problem using the oracle strategy, i.e., the
    problem is solved as static VRPTW with release dates using the information
    that is known in hindsight. The found solution is then submitted to the
    environment.
    """
    observation, info = env.reset()
    ep_tlim = info["epoch_tlim"]
    done = False

    # Submit dummy solutions to obtain the hindsight problem
    while not done:
        request_idcs = observation["epoch_instance"]["request_idx"][1:]
        ep_sol = [[request] for request in request_idcs]
        observation, _, done, _ = env.step(ep_sol)

    hindsight_inst = env.get_hindsight_problem()
    solution, _ = solve_static(hindsight_inst, time_limit=ep_tlim)

    observation, _ = env.reset()
    total_reward = 0
    done = False

    # Submit the solution from the hindsight problem
    while not done:
        ep_inst = observation["epoch_instance"]
        request_idcs = set(ep_inst["request_idx"])

        # NOTE This is a proxy to extract the routes from the hindsight
        # solution that are dispatched in the current epoch.
        ep_sol = [
            route
            for route in solution
            if len(request_idcs.intersection(route)) == len(route)
        ]

        observation, reward, done, info = env.step(ep_sol)
        assert info["error"] is None, f"{info['error']}"

        total_reward += reward

    return total_reward
