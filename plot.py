def x_axis(stats, value):
    if value == "iterations":
        return stats.curr_iters(), "Iteration (#)"
    else:
        return stats.run_times(), "Run-time (s)"


def plot_population(stats, ax, x_type="iterations"):
    x, x_label = x_axis(stats, x_type)

    ax.set_title("Population statistics")

    # Number feasible individuals
    ax.plot(x, stats.num_feasible_pop(), label="# Feasible", c="tab:orange")
    ax.set_xlabel(x_label)
    ax.set_ylabel("Individuals (#)")
    ax.legend(frameon=False)

    # Population diversity
    ax_div = ax.twinx()
    ax_div.plot(x, stats.pop_diversity(), label="Diversity", c="tab:red")
    ax_div.set_ylabel("Avg. diversity")
    ax_div.legend(frameon=False)


def plot_objectives(stats, ax, x_type="iterations"):
    x, x_label = x_axis(stats, x_type)

    ax.plot(
        x, stats.curr_objectives(), label="Current objective", c="tab:blue"
    )
    ax.plot(x, stats.feas_objectives(), label="Feasible", c="tab:green")
    ax.plot(x, stats.infeas_objectives(), label="Infeasible", c="tab:red")

    ax.set_title("Population average objective")
    ax.set_xlabel(x_label)

    # Use best objectives to set reasonable y-limits
    best = min(stats.curr_objectives())
    ax.set_ylim(best * 0.995, best * 1.03)
    ax.set_ylabel("Objective")

    ax.legend(frameon=False)


def plot_incumbents(stats, ax):
    times, objs = list(zip(*stats.best_objectives()))
    ax.plot(times, objs)

    ax.set_title("Improving objective values")
    ax.set_xlabel("Run-time (s)")
    ax.set_ylabel("Objective")
    ax.set_ylim(min(objs) * 0.995, min(objs) * 1.03)
