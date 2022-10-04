import numpy as np

_N_POINTS = 100


def x_axis(stats, step, plot_runtimes):
    if plot_runtimes:
        return stats.run_times()[::step], "Run-time (s)"
    else:
        return np.arange(0, stats.num_iters(), step), "Iteration (#)"


def plot_instance(ax, instance, routes=()):
    """
    Plot an instance and optionally a solution. This plot contains the depot
    location (yellow star) and customer locations. A client is represented by a
    blue dot, with a size relative to when its time window opens. Around this
    dot, the relative size of the blue circle represents when a time windows
    closes.

    A given list of routes can also be plotted, if provided.
    """
    is_client = ~instance["is_depot"]
    coords = instance["coords"][is_client].T
    tws_open = instance["time_windows"][is_client, 0]
    tws_close = instance["time_windows"][is_client, 1]
    depot_coords = instance["coords"][~is_client].T

    kwargs = dict(s=(0.0003 * tws_open) ** 2, zorder=3)
    ax.scatter(*coords, c="tab:blue", label="TW open", **kwargs)

    kwargs = dict(s=(0.0008 * tws_close) ** 2, alpha=0.1, zorder=3)
    ax.scatter(*coords, c="tab:blue", label="TW close", **kwargs)

    kwargs = dict(marker="*", zorder=3, s=750)
    ax.scatter(*depot_coords, c="tab:red", label="Depot", **kwargs)

    for route in routes:
        ax.plot(*instance["coords"][[0] + route + [0]].T)

    ax.grid(color="grey", linestyle="--", linewidth=0.25)

    ax.set_title("Solution" if routes else "Instance")
    ax.set_aspect("equal", "datalim")
    ax.legend(frameon=False, ncol=3)


def plot_population(ax, stats, step=None, plot_runtimes=False):
    if step is None:
        step = max(1, stats.num_iters() // _N_POINTS)

    x_vals, x_label = x_axis(stats, step, plot_runtimes)

    ax.set_title("Population diversity")
    ax.set_xlabel(x_label)
    ax.set_ylabel("Avg. diversity")

    ax.plot(
        x_vals,
        stats.feas_avg_diversity()[::step],
        label="Feas. diversity",
        c="tab:green",
    )
    ax.plot(
        x_vals,
        stats.infeas_avg_diversity()[::step],
        label="Infeas. diversity",
        c="tab:red",
    )

    ax.legend(frameon=False)


def plot_objectives(ax, stats, step=None, plot_runtimes=False):
    if step is None:
        step = max(1, stats.num_iters() // _N_POINTS)

    x_vals, x_label = x_axis(stats, step, plot_runtimes)

    global_best = np.minimum.accumulate(stats.feas_best_cost())
    ax.plot(x_vals, global_best[::step], label="Global best", c="tab:blue")

    ax.plot(
        x_vals,
        stats.feas_best_cost()[::step],
        label="Feas best",
        c="tab:green",
    )
    ax.plot(
        x_vals,
        stats.feas_avg_cost()[::step],
        label="Feas avg.",
        c="tab:green",
        alpha=0.3,
        linestyle="dashed",
    )
    ax.plot(
        x_vals,
        stats.infeas_best_cost()[::step],
        label="Infeas best",
        c="tab:red",
    )
    ax.plot(
        x_vals,
        stats.infeas_avg_cost()[::step],
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
