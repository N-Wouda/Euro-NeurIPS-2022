import numpy as np

_N_POINTS = 100


def x_axis(stats, step, plot_runtimes):
    if plot_runtimes:
        return stats.run_times()[::step], "Run-time (s)"
    else:
        return np.arange(0, stats.num_iters(), step), "Iteration (#)"


def plot_population(stats, ax, step=None, plot_runtimes=False):

    if step is None:
        step = max(1, stats.num_iters() // _N_POINTS)

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
    line_feas_diversity = ax_div.plot(
        x_vals,
        stats.feas_diversity()[::step],
        label="Feas. diversity",
        c="tab:green",
    )
    line_infeas_diversity = ax_div.plot(
        x_vals,
        stats.infeas_diversity()[::step],
        label="Infeas. diversity",
        c="tab:red",
    )
    ax_div.set_ylabel("Avg. diversity")

    # Place different ax labels in one legend
    lines = (
        line_pop_sizes
        + line_num_feasible_pop
        + line_feas_diversity
        + line_infeas_diversity
    )

    labels = [line.get_label() for line in lines]
    ax.legend(lines, labels, frameon=False)


def plot_objectives(stats, ax, step=None, plot_runtimes=False):

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


def plot_incumbents(stats, ax):
    times, objs = list(zip(*stats.incumbents()))
    ax.plot(times, objs)

    ax.set_title("Improving objective values")
    ax.set_xlabel("Run-time (s)")
    ax.set_ylabel("Objective")

    ax.set_ylim(min(objs) * 0.995, min(objs) * 1.03)
