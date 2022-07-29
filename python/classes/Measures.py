class Measures:
    """
    Simple class that supports a (growing) number of collected run-time
    statistics.
    """

    def legend(self):
        return ["Objective", "Iters. (#)", "Improv. (#)"]

    def dtypes(self):
        return [('obj', int), ('iters', int), ('nb_improv', int)]

    def __call__(self, result):
        best = result.get_best_found()
        stats = result.get_statistics()

        return int(best.cost()), stats.num_iters(), len(stats.best_objectives())
