def x_axis(stats, plot_runtimes):
    if plot_runtimes:
        return stats.run_times(), "Run-time (s)"
    else:
        return stats.curr_iters(), "Iteration (#)"


def plot_population(stats, ax, plot_runtimes=False):
    x_vals, x_label = x_axis(stats, plot_runtimes)

    # Number feasible individuals
    ax.plot(
        x_vals, stats.num_feasible_pop(), label="# Feasible", c="tab:orange"
    )

    ax.set_title("Population statistics")
    ax.set_xlabel(x_label)
    ax.set_ylabel("Individuals (#)")
    ax.legend(frameon=False)

    # Population diversity
    ax_div = ax.twinx()
    ax_div.plot(x_vals, stats.pop_diversity(), label="Diversity", c="tab:red")
    ax_div.set_ylabel("Avg. diversity")
    ax_div.legend(frameon=False)


def plot_objectives(stats, ax, plot_runtimes=False):
    x_vals, x_label = x_axis(stats, plot_runtimes)

    ax.plot(x_vals, stats.best_objectives(), label="Best", c="tab:blue")
    ax.plot(x_vals, stats.feas_objectives(), label="Feasible", c="tab:green")
    ax.plot(x_vals, stats.infeas_objectives(), label="Infeasible", c="tab:red")

    ax.set_title("Population objectives")
    ax.set_xlabel(x_label)
    ax.set_ylabel("Objective")

    # Use best objectives to set reasonable y-limits
    best = min(stats.best_objectives())
    ax.set_ylim(best * 0.995, best * 1.03)
    ax.legend(frameon=False)


def plot_incumbents(stats, ax):
    times, objs = list(zip(*stats.incumbents()))
    ax.plot(times, objs)

    ax.set_title("Improving objective values")
    ax.set_xlabel("Run-time (s)")
    ax.set_ylabel("Objective")

    ax.set_ylim(min(objs) * 0.995, min(objs) * 1.03)
