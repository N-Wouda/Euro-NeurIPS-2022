import numpy as np

_N_POINTS = 100


def x_axis(stats, step, plot_runtimes):
    if plot_runtimes:
        return stats.run_times()[::step], "Run-time (s)"
    else:
        return np.arange(0, stats.num_iters(), step), "Iteration (#)"


def plot_instance(ax, instance, routes=()):
    """
    Plot an instance and optionally a solution.
    This plot contains the depot location (yellow star) and customer locations.
    A customer is represented with a blue dot, with a size relative to when it's time window opens.
    Around this dot, the relative size of the blue circle represents when a time windows closes.
    Required parameters are the instance name and dictionary.
    Optionally, a list of routes can be provided to be plotted.
    When a save_in location is specified, the plot is saved there,
    otherwise the plot will be shown during execution.
    """

    is_client = ~instance['is_depot']
    coords = instance['coords'][is_client].T
    tws_open = instance["time_windows"][is_client, 0]
    tws_close = instance["time_windows"][is_client, 1]
    depot_coords = instance['coords'][~is_client].T

    ax.scatter(*coords, c="blue", s=(0.0005 * tws_close) ** 2, alpha=0.1)
    ax.scatter(*coords, c="blue", s=(0.0001 * tws_open) ** 2)
    ax.scatter(*depot_coords, c="orange", s=500, marker="*")

    ax.set_xticks((0, np.max(instance['coords'])))
    ax.set_yticks((0, np.max(instance['coords'])))

    ax.set_xlim(0, np.max(instance['coords']))
    ax.set_ylim(0, np.max(instance['coords']))

    ax.set_aspect('equal', 'box')

    for route in routes:
        ax.plot(*instance['coords'][[0] + route + [0]].T, linewidth=0.1)


def plot_population(ax, stats, step=None, plot_runtimes=False):

    if step is None:
        step = min(1, stats.num_iters() // _N_POINTS)

    x_vals, x_label = x_axis(stats, step, plot_runtimes)

    # Population size
    line_pop_sizes = ax.plot(
        x_vals,
        stats.pop_sizes()[::step],
        label="Population size",
        c="tab:blue",
    )

    # Number feasible individuals
    line_num_feasible_pop = ax.plot(
        x_vals,
        stats.num_feasible_pop()[::step],
        label="# Feasible",
        c="tab:orange",
    )

    ax.set_title("Population statistics")
    ax.set_xlabel(x_label)
    ax.set_ylabel("Individuals (#)")

    # Population diversity
    ax_div = ax.twinx()
    line_pop_diversity = ax_div.plot(
        x_vals, stats.pop_diversity()[::step], label="Diversity", c="tab:red"
    )
    ax_div.set_ylabel("Avg. diversity")

    # Place different ax labels in one legend
    lines = line_pop_sizes + line_num_feasible_pop + line_pop_diversity
    labels = [line.get_label() for line in lines]
    ax.legend(lines, labels, frameon=False)


def plot_objectives(ax, stats, step=None, plot_runtimes=False):

    if step is None:
        step = stats.num_iters() // _N_POINTS

    x_vals, x_label = x_axis(stats, step, plot_runtimes)

    global_best = np.minimum.accumulate(stats.feas_best())
    ax.plot(x_vals, global_best[::step], label="Global best", c="tab:blue")

    ax.plot(
        x_vals, stats.feas_best()[::step], label="Feas best", c="tab:green"
    )
    ax.plot(
        x_vals,
        stats.feas_average()[::step],
        label="Feas avg.",
        c="tab:green",
        alpha=0.3,
        linestyle="dashed",
    )
    ax.plot(
        x_vals,
        stats.infeas_best()[::step],
        label="Infeas best.",
        c="tab:red",
    )
    ax.plot(
        x_vals,
        stats.infeas_average()[::step],
        label="Infeas avg.",
        c="tab:red",
        alpha=0.3,
        linestyle="dashed",
    )

    ax.set_title("Population objectives")
    ax.set_xlabel(x_label)
    ax.set_ylabel("Objective")

    # Use global best objectives to set reasonable y-limits
    best = min(global_best)
    ax.set_ylim(best * 0.995, best * 1.03)
    ax.legend(frameon=False)


def plot_incumbents(ax, stats):
    times, objs = list(zip(*stats.incumbents()))
    ax.plot(times, objs)

    ax.set_title("Improving objective values")
    ax.set_xlabel("Run-time (s)")
    ax.set_ylabel("Objective")

    ax.set_ylim(min(objs) * 0.995, min(objs) * 1.03)
