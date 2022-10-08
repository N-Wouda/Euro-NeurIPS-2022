import hgspy
from .solve_static import solve_static


def solve_hindsight(env, solver_seed):
    """
    Solve the dynamic VRPTW problem using the oracle strategy, i.e., the
    problem is solved as static VRPTW with release dates using the information
    that is known in hindsight. The found solution is then submitted to the
    environment. The given seed is passed to the static solver.
    """
    observation, info = env.reset()
    ep_tlim = info["epoch_tlim"]
    done = False

    # Submit dummy solutions to obtain the hindsight problem
    while not done:
        requests = observation["epoch_instance"]["request_idx"][1:]
        ep_sol = [[request] for request in requests]
        observation, _, done, _ = env.step(ep_sol)

    hindsight_inst = env.get_hindsight_problem()

    config = hgspy.Config(seed=solver_seed)
    stop = hgspy.stop.MaxRuntime(ep_tlim)

    node_ops = [
        hgspy.operators.Exchange10,
        hgspy.operators.Exchange11,
        hgspy.operators.Exchange20,
        hgspy.operators.MoveTwoClientsReversed,
        hgspy.operators.Exchange21,
        hgspy.operators.Exchange22,
        hgspy.operators.TwoOpt,
    ]

    route_ops = [
        hgspy.operators.RelocateStar,
        hgspy.operators.SwapStar,
    ]

    crossover_ops = [
        hgspy.crossover.selective_route_exchange,
    ]

    res = solve_static(
        hindsight_inst, config, node_ops, route_ops, crossover_ops, stop
    )

    best = res.get_best_found()
    routes = [route for route in best.get_routes() if route]
    observation, _ = env.reset()

    # Submit the solution from the hindsight problem
    while not env.is_done:
        ep_inst = observation["epoch_instance"]
        requests = set(ep_inst["request_idx"])

        # This is a proxy to extract the routes from the hindsight
        # solution that are dispatched in the current epoch.
        ep_sol = [
            route
            for route in routes
            if len(requests.intersection(route)) == len(route)
        ]

        observation, reward, done, info = env.step(ep_sol)
        assert info["error"] is None, f"{info['error']}"

    return env.final_costs, env.final_solutions
