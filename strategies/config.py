from __future__ import annotations

import tomli

import tools

hgspy = tools.get_hgspy_module()


class Config(dict):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # Ensures these keys at least exist
        self["static"] = self.get("static", {})
        self["dynamic"] = self.get("dynamic", {})
        self["hindsight"] = self.get("hindsight", {})

    @classmethod
    def from_file(cls, where: str) -> Config:
        with open(where, "rb") as fh:
            data = tomli.load(fh)
            return cls(**data)

    # Static related attributes

    def static_node_ops(self):
        node_ops = self["static"].get("node_ops", [])
        return [getattr(hgspy.operators, op) for op in node_ops]

    def static_route_ops(self):
        route_ops = self["static"].get("route_ops", [])
        return [getattr(hgspy.operators, op) for op in route_ops]

    def static_crossover_ops(self):
        crossover_ops = self["static"].get("crossover_ops", [])
        return [getattr(hgspy.crossover, op) for op in crossover_ops]

    def static_solver_params(self):
        return self["static"].get("params", {})

    # Dynamic related attributes

    def dynamic_node_ops(self):
        node_ops = self["dynamic"].get("node_ops", [])
        return [getattr(hgspy.operators, op) for op in node_ops]

    def dynamic_route_ops(self):
        route_ops = self["dynamic"].get("route_ops", [])
        return [getattr(hgspy.operators, op) for op in route_ops]

    def dynamic_crossover_ops(self):
        crossover_ops = self["dynamic"].get("crossover_ops", [])
        return [getattr(hgspy.crossover, op) for op in crossover_ops]

    def dynamic_solver_params(self):
        return self["dynamic"].get("params", {})

    def dynamic_strategy_params(self):
        return self["dynamic"].get("strategy", {})

    # Hindsight related attributes

    def hindsight_node_ops(self):
        node_ops = self["hindsight"].get("node_ops", [])
        return [getattr(hgspy.operators, op) for op in node_ops]

    def hindsight_route_ops(self):
        route_ops = self["hindsight"].get("route_ops", [])
        return [getattr(hgspy.operators, op) for op in route_ops]

    def hindsight_crossover_ops(self):
        crossover_ops = self["hindsight"].get("crossover_ops", [])
        return [getattr(hgspy.crossover, op) for op in crossover_ops]

    def hindsight_solver_params(self):
        return self["hindsight"].get("params", {})
