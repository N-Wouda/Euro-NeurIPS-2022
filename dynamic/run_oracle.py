from .solve_static import solve_static
import tools


def run_oracle(args, env):
    """
    Solve the dynamic VRPTW problem using the oracle strategy, i.e., the
    problem is solved as static VRPTW with release dates using the information
    that is known in hindsight. The found solution is then fed back into the
    environment.
    """
    observation, info = env.reset()
    epoch_tlim = info["epoch_tlim"]
    done = False

    # Submit dummy solutions to obtain the hindsight problem
    while not done:
        request_idcs = observation["epoch_instance"]["request_idx"][1:]
        ep_sol = [[request] for request in request_idcs]
        observation, _, done, _ = env.step(ep_sol)

    hindsight_problem = env.get_hindsight_problem()
    solution, _ = solve_static(hindsight_problem, time_limit=epoch_tlim)

    observation, _ = env.reset()
    total_reward = 0
    done = False

    # Submit the solution from the hindsight problem
    while not done:
        ep_inst = observation["epoch_instance"]
        request_idcs = set(ep_inst["request_idx"])

        # NOTE This is a proxy to extract the routes from the hindsight solution
        # that are dispatched in the current epoch.
        is_ep_route = lambda r: len(request_idcs.intersection(r)) == len(r)

        ep_sol = [route for route in solution if is_ep_route(route)]
        ep_cost = tools.validate_dynamic_epoch_solution(ep_inst, ep_sol)

        observation, reward, done, info = env.step(ep_sol)
        assert reward == -ep_cost, f"{info['error']}"

        total_reward += reward

    return total_reward
