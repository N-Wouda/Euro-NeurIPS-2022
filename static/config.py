import toml


class Config:
    def __init__(self, file=None, **kwargs):
        config = {}

        if file is not None:
            config = toml.load(file)

        config.update(kwargs)

        self.node_operators = config.pop("node_operators")
        self.route_operators = config.pop("route_operators")
        self.crossover_operators = config.pop("crossover_operators")

        self.solver_config = config.pop("params", {})
        self.solver_config.update(config)
